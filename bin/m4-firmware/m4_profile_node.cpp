#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <m4_base/WheelVelocityCmd.h>
#include <m4_base/DerivedVelocity.h>
#include "yaml-cpp/yaml.h"
#include "m4_base.h"

// Contact: Kevin Wu <wu.kevi@northeastern.edu>
// Last updated: February 25, 2026
// C++ and memory segmentation faults make me sad
// TODO: may want to specify wheel rates, but m4_base uses xvel, yvel for now
// EXAMPLE: roslaunch m4_base profile.launch profile:=straight xvel:=0.5 duration:=2.0 count:=5
// EXAMPLE: roslaunch m4_base profile.launch profile:=rotate yvel:=0.5 duration:=5.0 count:=5
// EXAMPLE: roslaunch m4_base profile.launch profile:=circle xvel:=0.3 yvel:=0.1 duration:=20.0 count:=1

// (lin_vel, ang_vel, duration) -> drive at (lin_vel, ang_vel) for (duration)
// NOTE: input is (xvel, yvel) and passed directly to drive()
struct MotionSegment
{
    float xvel;
    float yvel;
    float duration;   // seconds
    std::string label;
};

// Conversion between command signals (i.e., xvel, yvel) and physical units
// -------------------------------- NOTES --------------------------------
//  execute_drive():
//   phidotr = ((xvel - 4 * yvel * wheel_base/2) / wheel_radius) * 360/(2*pi)
//   phidotl = ((xvel + 4 * yvel * wheel_base/2) / wheel_radius) * 360/(2*pi)
//
//  average wheel speed:
//   (rad/s) xvel / wheel_radius
//   (deg/s) xvel / wheel_radius * 360/(2*pi)
//
//  differential wheel speed (deg/s):
//   4 * yvel * wheel_base / (2 * wheel_radius) * 360/(2*pi)
//
//  body angular velocity (rad/s):
//   (phidotr - phidotl) * wheel_radius / wheel_base
//      = (-2 * 4 * yvel * wheel_base/2 / wheel_radius) * wheel_radius
//        / wheel_base * (2*pi/360) ...
//
//  v_linear  = xvel
//  v_angular = 4 * yvel * wheel_base / (2.0) ...
// -----------------------------------------------------------------------
struct RobotGeometry
{
    float wheel_base;
    float wheel_radius;
};

float cmd_to_phidotr(float xvel, float yvel, float wheel_radius, float wheel_base)
{
	return ((xvel - 4 * yvel * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);
}

float cmd_to_phidotl(float xvel, float yvel, float wheel_radius, float wheel_base)
{
    return ((xvel + 4 * yvel * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);
}

float phidot_to_vk(float phidotr, float phidotl, float r)
{
    return (r / 2.0) * (phidotr + phidotl);
}

float phidot_to_wk(float phidotr, float phidotl, float r, float W_eff)
{
    return (r / W_eff) * (phidotr - phidotl);
}

// Profile from a list of motion segments
std::vector<MotionSegment> build_motion_profile(
    const std::vector<MotionSegment>& base_segments,
    int segment_count)
{
    std::vector<MotionSegment> profile;
    for (int ii = 0; ii < segment_count; ii++)
    {
        for (const auto& seg : base_segments)
            profile.push_back(seg);
    }
    return profile;
}

// Profile 1: (speed, duration, count) -> straight forward/backward
std::vector<MotionSegment> profile_straight_line(
    float xvel, float duration, int count)
{
    std::vector<MotionSegment> base = {
        { xvel,  0.0f, duration, "Forward"  },
        { 0.0f,  0.0f, 2.0f,     "Wait 2s"  },
        {-xvel,  0.0f, duration, "Backward" },
        { 0.0f,  0.0f, 5.0f,     "Wait 5s"  }
    };

    return build_motion_profile(base, count);
}

// Profile 2: (speed, duration, count) -> rotate in place
std::vector<MotionSegment> profile_rotate_in_place(
    float yvel, float duration, int count)
{
    std::vector<MotionSegment> base = {
        { 0.0f,  yvel, duration, "Rotate CCW" },
        { 0.0f,  0.0f, 2.0f,     "Wait 2s"    },
        { 0.0f, -yvel, duration, "Rotate CW"  },
        { 0.0f,  0.0f, 5.0f,     "Wait 5s"    }
    };

    return build_motion_profile(base, count);
}

// Profile 3: (lin_speed, ang_speed, count) -> drive in a circle
std::vector<MotionSegment> profile_circle(
    float xvel, float yvel, float duration, int count)
{
    std::vector<MotionSegment> profile;
    
    profile.push_back({0.0f, 0.0f, 5.0f, "Initial pause"});
    
    for (int ii = 0; ii < count; ii++)
    {
        profile.push_back({xvel, yvel, duration,
                           "Circle " + std::to_string(ii + 1)});
        profile.push_back({0.0f, 0.0f, 5.0f, "Pause"});
    }
    
    // Stop
    profile.push_back({0.0f, 0.0f, 5.0f, "Stop"});
    
    return profile;
}

// Execute
void execute_profile(M4Base& m4, ros::NodeHandle& nh,
                     const std::vector<MotionSegment>& profile,
                     const RobotGeometry& geom)
{
    // Publishers for recording
    ros::Publisher joint_state_pub =
        nh.advertise<sensor_msgs::JointState>("/m4/joint_states", 10);
    ros::Publisher drive_cmd_pub =
        nh.advertise<geometry_msgs::TwistStamped>("/m4/drive_cmd", 10);
    ros::Publisher wheel_vel_pub =
        nh.advertise<m4_base::WheelVelocityCmd>("/m4/wheel_velocity_cmd", 10);
    ros::Publisher derived_vel_pub =
        nh.advertise<m4_base::DerivedVelocity>("/m4/derived_vel", 10);
    ros::Publisher status_pub =
        nh.advertise<std_msgs::String>("/m4/profile_status", 10);

    ros::Rate publish_rate(50); // 50 Hz publishing while executing

    for (size_t ii = 0; ii < profile.size() && ros::ok(); ii++)
    {
        const MotionSegment& seg = profile[ii];
        
        // Derive quantities
        float phidotr = cmd_to_phidotr(seg.xvel, seg.yvel, geom.wheel_radius, geom.wheel_base);
        float phidotl = cmd_to_phidotl(seg.xvel, seg.yvel, geom.wheel_radius, geom.wheel_base);
        float vk = phidot_to_vk(phidotr, phidotl, geom.wheel_radius);
        float wk = phidot_to_wk(phidotr, phidotl, geom.wheel_radius, geom.wheel_base);
        
        ROS_INFO("Segment %zu/%zu: %s (xvel=%.3f, yvel=%.3f, dur=%.2fs)",
                 ii + 1, profile.size(), seg.label.c_str(),
                 seg.xvel, seg.yvel, seg.duration);
        ROS_INFO("  Derived: phidotr=%.3f, phidotl=%.3f, vk=%.3f, wk=%.3f",
                 phidotr, phidotl, vk, wk);

        // Publish status
        std_msgs::String status_msg;
        status_msg.data = seg.label;
        status_pub.publish(status_msg);

        // Send command
        m4.drive(seg.xvel, seg.yvel);

        // Hold command for segment duration
        auto seg_start = std::chrono::steady_clock::now();
        while (ros::ok())
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - seg_start).count();
            if (elapsed >= seg.duration)
                break;

            // Publish joint states and drive command
            #ifndef DRY_RUN
                if (m4.state_available())
                {
                    RobotState state = m4.get_state();
                    sensor_msgs::JointState js_msg;
                    js_msg.header.stamp = ros::Time::now();
                    // state.positions, state.velocities std::vector<float>
                    js_msg.position.assign(state.positions.begin(), state.positions.end());
                    js_msg.velocity.assign(state.velocities.begin(), state.velocities.end());
                    js_msg.effort.assign(state.currents.begin(), state.currents.end());
                    joint_state_pub.publish(js_msg);
                    // lin_vel_cmd, ang_vel_cmd std::vector<float>
                    geometry_msgs::TwistStamped drive_msg;
                    drive_msg.header.stamp = ros::Time::now();
                    drive_msg.twist.linear.x = m4.get_linear_velocity_cmd();
                    drive_msg.twist.angular.z = m4.get_angular_velocity_cmd();
                    drive_cmd_pub.publish(drive_msg);
                }
            #endif

            // Publish wheel velocity command
            #ifndef DRY_RUN
            {
                // right_wheel_velocity, left_wheel_velocity std::vector<float>
                std::vector<float> wv = m4.get_wheel_velocity_cmd();
                if (wv.size() >= 4) // guard
                {
                    m4_base::WheelVelocityCmd wheel_msg;
                    wheel_msg.header.stamp = ros::Time::now();
                    wheel_msg.FL = wv[0];
                    wheel_msg.FR = wv[1];
                    wheel_msg.BL = wv[2];
                    wheel_msg.BR = wv[3];
                    wheel_vel_pub.publish(wheel_msg);
                }
            }
            #endif
            
            // Publish derived quantities
            // TODO: custom msg
            m4_base::DerivedVelocity derived_msg;
            derived_msg.header.stamp = ros::Time::now();
            derived_msg.vk = vk;
            derived_msg.wk = wk;
            derived_msg.phidotr = phidotr;
            derived_msg.phidotl = phidotl;
            derived_vel_pub.publish(derived_msg);
            
            ros::spinOnce();
            publish_rate.sleep();
        }
    }

    // Stop when profile is complete
    m4.drive(0.0f, 0.0f);
    ROS_INFO("Motion profile complete.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m4_profile_node");
    ros::NodeHandle nh("~"); // private namespace for parameters

    // Read parameters
    std::string profile_type;
    nh.param<std::string>("profile", profile_type, "straight");

    float xvel, yvel, duration;
    int count;

    nh.param<float>("xvel",        xvel,        0.0f);
    nh.param<float>("yvel",        yvel,        0.0f);
    nh.param<float>("duration",    duration,    2.0f);   // s
    nh.param<int>("count",         count,       1);

    // Load robot geometry
    RobotGeometry geom;
    std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
    YAML::Node config = YAML::LoadFile(configFilePath);
    geom.wheel_base = config["wheel_base"].as<float>();
    geom.wheel_radius = config["wheel_radius"].as<float>();
    
    ROS_INFO("Robot geometry: wheel_base=%.4f, wheel_radius=%.4f",
             geom.wheel_base, geom.wheel_radius);

    // Build profile
    std::vector<MotionSegment> profile;

    if (profile_type == "straight")
    {
        ROS_INFO("Profile: Straight line, xvel=%.3f, duration=%.2fs, count=%d",
                 xvel, duration, count);
        profile = profile_straight_line(xvel, duration, count);
    }
    else if (profile_type == "rotate")
    {
        ROS_INFO("Profile: Rotate in place, yvel=%.3f, duration=%.2fs, count=%d",
                 yvel, duration, count);
        profile = profile_rotate_in_place(yvel, duration, count);
    }
    else if (profile_type == "circle")
    {
        ROS_INFO("Profile: Circle, xvel=%.3f, yvel=%.3f, duration=%.2fs, count=%d",
                 xvel, yvel, duration, count);
        profile = profile_circle(xvel, yvel, duration, count);
    }
    else
    {
        ROS_ERROR("Unknown profile type: %s", profile_type.c_str());
        return 1;
    }

    // Initialize robot
    std::vector<int> active_joint_ids = {1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> active_wheel_ids = {9, 10, 11, 12};
    M4Base m4(active_joint_ids, active_wheel_ids, true); //false);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    #ifndef DRY_RUN
		m4.sit();
		std::cout<<"Sit complete!\n";
		while(m4.get_current_configuration() != M4Configurations::SIT)
		    std::this_thread::sleep_for(std::chrono::milliseconds(100));
		std::cout<<"Sit wait complete!\n";
	#endif
	
	#ifndef DRY_RUN
		m4.stand();
		while(m4.get_current_configuration() != M4Configurations::STAND)
		    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	#endif

    // --- Not sure if this is needed ---
	ros::Rate loop_rate(200);

	M4Configurations current_configuration=M4Configurations::INIT;
	#ifdef DRY_RUN
		current_configuration = M4Configurations::STAND;
	#endif
	// ----------------------------------

    ROS_INFO("Robot ready. Starting profile in 3 seconds...");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Execute 
    execute_profile(m4, nh, profile, geom);

    // Shutdown
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	#ifndef DRY_RUN
		m4.sit();
		while(m4.get_current_configuration() != M4Configurations::SIT)
		    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	#endif

    return 0;
}

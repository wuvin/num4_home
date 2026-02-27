#include <string>
#include <vector>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "DynamixelInterface.h"
#include "m4_base.h"

#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)

M4Base::M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status):active_joint_ids(active_joint_ids), active_wheel_ids(active_wheel_ids), linear_velocity_cmd(0.0), angular_velocity_cmd(0.0), log_status(false), quit(false), state_updated(false), command_queue(20), current_configuration(M4Configurations::INIT)
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);
	
	CONTROL_LOOP_RATE = config["control_loop_rate"].as<float>();
	LOG_PATH = config["log_path"].as<std::string>();

	LOG_BUFFER_SIZE = config["log_buffer_size"].as<int>();
	robot_state.reserve(LOG_BUFFER_SIZE);

	std::string interface_port = config["interface_port"].as<std::string>();
	int baudrate = config["baudrate"].as<int>();

	all_joint_ids = config["all_joint_ids"].as<std::vector<int>>();
	all_joint_names = config["joint_names"].as<std::vector<std::string>>();
	all_wheel_ids = config["all_wheel_ids"].as<std::vector<int>>();
	all_wheel_names = config["wheel_names"].as<std::vector<std::string>>();

	active_joint_names = get_active_joint_names();
	active_wheel_names = get_active_wheel_names();

	stand_configuration = get_active_joint_positions(config["config_stand"].as<std::vector<float>>());
	crouch1_configuration = get_active_joint_positions(config["config_crouch1"].as<std::vector<float>>());
	crouch2_configuration = get_active_joint_positions(config["config_crouch2"].as<std::vector<float>>());
	sit_configuration = get_active_joint_positions(config["config_sit"].as<std::vector<float>>());
	uav_step1_configuration = get_active_joint_positions(config["config_uav_step1"].as<std::vector<float>>());
	uav_step2_configuration = get_active_joint_positions(config["config_uav_step2"].as<std::vector<float>>());
	mip_configuration = get_active_joint_positions(config["config_mip"].as<std::vector<float>>());
	wheel_base = config["wheel_base"].as<float>();
	wheel_radius = config["wheel_radius"].as<float>();
	float transition_vel = config["config_transition_wheel_vel"].as<float>();
	transition_wheel_vel_in = get_active_wheel_velocities({-transition_vel, -transition_vel, transition_vel, transition_vel});
	transition_wheel_vel_out = get_active_wheel_velocities({transition_vel, transition_vel, -transition_vel, -transition_vel});

	dxl.init(interface_port.c_str(), baudrate, active_joint_ids, active_wheel_ids);

	// Initialize so parameter is not accessed while empty
	wheel_velocity_cmd = std::vector<float>(active_wheel_ids.size(), 0.0f);
	// need wheel_velocity_cmd_ids here too? or is ok just in mutex lock

	control_thread = std::thread(&M4Base::control_loop, this);
    set_log_status(log_status);
}

M4Base::~M4Base()
{
	quit = true;
	control_thread.join();
	if(log_status)
		log_thread.join();
	dxl.stop();
}

void M4Base::control_loop()
{
	printf("%s[INFO] Starting control loop...\n", CONSOLE_MSG_INFO);

	auto t0 = std::chrono::system_clock::now();
	auto t1 = std::chrono::system_clock::now();
	auto gait_start = std::chrono::system_clock::now();
	double dt, sleep_for;

	M4Commands next_cmd;
	std::vector<std::string> commands = {"STAND", "CROUCH1", "CROUCH2", "SIT", "UAV", "MIP"};
	bool command_active = false;
	int command_step = 0;
	
	printf("[INFO] Reached prelude to control loop.\n");
	
	if(!quit)
	{
	    printf("%s[INFO] Able to enter control loop.\n", CONSOLE_MSG_INFO);
	}

    while(!quit)
    {
		if(!log_status && robot_state.size() >= LOG_BUFFER_SIZE)
		{
			robot_state.clear();
			state_updated = false;
		}

		update_state();

		// Execute commands
		if(!command_queue.empty() || command_active)
		{
			if(!command_active)
			{
				next_cmd = command_queue.pop();
				printf("%s[WARN]: Got command %s%s\n", CONSOLE_MSG_WARN, commands[next_cmd].c_str(), CONSOLE_DEFAULT);
			}

			switch(next_cmd)
			{
				case M4Commands::CMD_STAND:
					execute_stand(command_step, command_active, gait_start);
		                        break;
				case M4Commands::CMD_CROUCH1:
					execute_crouch1(command_step, command_active, gait_start);
					break;
				case M4Commands::CMD_CROUCH2:
					execute_crouch2(command_step, command_active, gait_start);
					break;
				case M4Commands::CMD_SIT:
					execute_sit(command_step, command_active, gait_start);
					break;
				case M4Commands::CMD_UAV:
					execute_uav(command_step, command_active, gait_start);
					break;
				case M4Commands::CMD_MIP:
					execute_mip(command_step, command_active, gait_start);
					break;
			}
		}

		else
		{
		    execute_drive();
		}

		// Sleep until next loop time
		t1 = std::chrono::system_clock::now();
		dt = get_duration_secs(t0, t1);
		sleep_for = (1.0/CONTROL_LOOP_RATE - dt) * 1.0e3;
		sleep_for = std::max(sleep_for,0.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_for)));	
		t0 = std::chrono::system_clock::now();

    }
}

void M4Base::execute_stand(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::STAND:
			command_step = 0;
			command_active = false;
			return;
		case M4Configurations::CROUCH1:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
			    dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				printf("[INFO] Stand Transformation complete!\n");
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}

			break;

		case M4Configurations::CROUCH2:
		case M4Configurations::INIT:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
			    dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				printf("[INFO] Stand Transformation complete!\n");
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}

			break;
		case M4Configurations::SIT:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Stand Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}
			break;

		// ---------------------------------------------------------
		// Transformation from UAV to STANDING
		// ---------------------------------------------------------
		case M4Configurations::MIP:
			break;
		case M4Configurations::UAV:
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to stand
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				command_step = 4;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Stand Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}

			break;
	}
}

void M4Base::execute_crouch1(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::CROUCH1:
			command_step = 0;
			command_active = false;
			return;
		case M4Configurations::CROUCH2:
		case M4Configurations::SIT:
//Taoran adding transition_wheel_vel_in
			// Step 1: Move to crouch1 position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, crouch1_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;

		case M4Configurations::STAND:

			// Step 1: Move to crouch1 position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, crouch1_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;

		case M4Configurations::MIP:
			break;
		case M4Configurations::UAV:
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to crouch1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				dxl.write_position(active_joint_ids, crouch1_configuration);
// Taoran adding fixed should be wheel in
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				gait_start = std::chrono::system_clock::now();
				command_step = 4;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.8 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;
	}
	
}

void M4Base::execute_crouch2(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::CROUCH2:
			command_step = 0;
			command_active = false;
			return;
		case M4Configurations::CROUCH1:
					// Step 1: Move to crouch2 position
			if(command_step == 0)
			{
				dxl.write_position(active_joint_ids, crouch2_configuration);
//Taoran adding wheel spining
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;
		case M4Configurations::SIT:
// SIT should not be here
		case M4Configurations::STAND:

			// Step 1: Move to crouch2 position
			if(command_step == 0)
			{
				dxl.write_position(active_joint_ids, crouch2_configuration);
//Taoran adding wheel spining
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.8 && command_step == 1)
			{
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;

		case M4Configurations::MIP:
			break;
		case M4Configurations::UAV:
			// Step 1: Move to uav step2 position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 3)
			{
				command_step = 4;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 5: Move to crouch2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, crouch2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 5;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.2 && command_step == 5)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;
	}
}

void M4Base::execute_sit(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::SIT:
			command_step = 0;
			command_active = false;
			return;
		case M4Configurations::INIT:
		case M4Configurations::STAND:
		case M4Configurations::CROUCH1:
		case M4Configurations::CROUCH2:
		case M4Configurations::MIP:
			// Step 1: Move to sit position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Sit Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::SIT;
			}

			break;

		case M4Configurations::UAV:
			// Step 1: Move to uav step2 position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] SIT Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::SIT;
			}
	}
}

void M4Base::execute_uav(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::UAV:
			command_step = 0;
			command_active = false;
			return;
			
		case M4Configurations::CROUCH1:
					// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;

		case M4Configurations::STAND:			// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;
		case M4Configurations::CROUCH2:
		case M4Configurations::SIT:
			// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.2 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;
	}
}

void M4Base::execute_mip(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start)
{
	switch(current_configuration)
	{
		case M4Configurations::MIP:
			command_step = 0;
			command_active = false;
			return;
		case M4Configurations::CROUCH1:
		case M4Configurations::CROUCH2:
		case M4Configurations::STAND:
		case M4Configurations::SIT:
			// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to MIP position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, mip_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 2)
			{
				printf("[INFO] MIP Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::MIP;
			}

			break;
	}
}

void M4Base::init_log()
{
	printf("%s[INFO] Logging to %s\n", CONSOLE_MSG_INFO, LOG_PATH.c_str());

	// Account for additional wheel states
	std::vector<std::string> all_active_names;
	all_active_names.insert(all_active_names.end(), active_joint_names.begin(), active_joint_names.end());
	all_active_names.insert(all_active_names.end(), active_wheel_names.begin(), active_wheel_names.end());

	position_logger = new Logger("position.csv", active_joint_names, LOG_PATH.c_str());
	velocity_logger = new Logger("velocity.csv", active_joint_names, LOG_PATH.c_str());
	current_logger = new Logger("current.csv", active_joint_names, LOG_PATH.c_str());

	log_thread = std::thread(&M4Base::log_status_loop, this);
}

void M4Base::clear_logs()
{
	delete position_logger;
	delete velocity_logger;
	delete current_logger;
}

void M4Base::set_log_status(bool log_status)
{
    printf("[INFO] Reached set_log_status.\n");
    
	if(log_status && !this->log_status) // Setting logging on - creates new logs
	{
		printf("%s[INFO] Turning on logging...\n", CONSOLE_MSG_INFO);
		init_log();
		this->log_status = true;
	}

	else if(!log_status && this->log_status) // Setting logging off - frees up memory
	{
		printf("%s[INFO] Turning off logging\n", CONSOLE_MSG_INFO);
		this->log_status = log_status;
		log_thread.join();
		clear_logs();
	}
}

void M4Base::log_status_loop()
{
	printf("%s[INFO] Logging loop...\n", CONSOLE_MSG_INFO);
	int sleep_time = int((LOG_BUFFER_SIZE/CONTROL_LOOP_RATE) * 1000);
	while(log_status && !quit)
	{		
		if(robot_state.size() >= LOG_BUFFER_SIZE)
		{
			printf("%s[INFO] Logging buffer...\n", CONSOLE_MSG_INFO);
			std::vector<std::vector<float>> data_position;
			std::vector<std::vector<float>> data_velocity;
			std::vector<std::vector<float>> data_current;

			{// MUTEX LOCK
				std::lock_guard<std::mutex> lock(m_state);
				for(int i = 0; i < robot_state.size(); i++)
				{
					std::vector<float> q = robot_state[i].positions;
					q.insert(q.begin(), robot_state[i].timestamp);
					data_position.push_back(q);
				
					std::vector<float> qdot = robot_state[i].velocities;
					qdot.insert(qdot.begin(), robot_state[i].timestamp);
					data_velocity.push_back(qdot);
				
					std::vector<float> current = robot_state[i].currents;
					current.insert(current.begin(), robot_state[i].timestamp);
					data_current.push_back(current);
				}
			}// MUTEX UNLOCK

			position_logger->log_batch_entry(data_position);
			velocity_logger->log_batch_entry(data_velocity);
			current_logger->log_batch_entry(data_current);
			
			{// MUTEX LOCK
				std::lock_guard<std::mutex> lock(m_state);
				robot_state.clear();
			}// MUTEX UNLOCK

			state_updated = false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
	}

	// Save the remaining data from buffer into file before stopping logging thread
	printf("[INFO] Finishing logging...\n");
	
	if(!robot_state.empty())
	{
		printf("[INFO] Saving remaining buffer...\n");
		std::vector<std::vector<float>> data_position;
		std::vector<std::vector<float>> data_velocity;
		std::vector<std::vector<float>> data_current;

		for(int i = 0; i < robot_state.size(); i++)
		{
			std::vector<float> q = robot_state[i].positions;
			q.insert(q.begin(), robot_state[i].timestamp);
			data_position.push_back(q);
		
			std::vector<float> qdot = robot_state[i].velocities;
			qdot.insert(qdot.begin(), robot_state[i].timestamp);
			data_velocity.push_back(qdot);
		
			std::vector<float> current = robot_state[i].currents;
			current.insert(current.begin(), robot_state[i].timestamp);
			data_current.push_back(current);
		}

		position_logger->log_batch_entry(data_position);
		velocity_logger->log_batch_entry(data_velocity);
		current_logger->log_batch_entry(data_current);
		{
			std::lock_guard<std::mutex> lock(m_state);
			robot_state.clear();
			state_updated = false;
		}
		
	}
}

void M4Base::update_state()
{
	DynamixelStatus status = dxl.read_status();

	// Skip update if read failed or returned garbage
	if (status.position.empty() || status.velocity.empty() || status.current.empty())
		return;

	// Expect joint count + wheel count
	if (status.position.size() < active_joint_ids.size() + active_wheel_ids.size())
		return;

	RobotState r;
	r.timestamp = status.timestamp;
	r.positions = status.position;
	r.velocities = status.velocity;
	r.currents = status.current;
	
	{// MUTEX LOCK
		std::lock_guard<std::mutex> lock(m_state);
		robot_state.push_back(r);
	}// MUTEX UNLOCK
	
	state_updated = true;
}

bool M4Base::state_available()
{
	// Original code:
	// return state_updated;

	// Update for thread safety + edge case with log thread just cleared
	std::lock_guard<std::mutex> lock(m_state);
	return state_updated && !robot_state.empty();
}

RobotState M4Base::get_state()
{
	// Original code:
	// state_updated = false;
	// return robot_state.back();

	// Original does not hold mutex while update_state() pushes with it
	// This is potential segfault when .back() is called during mid-mod
	// Fix to ensure safety of control thread
	std::lock_guard<std::mutex> lock(m_state);
	state_updated = false;
	return robot_state.back();
}

void M4Base::drive(float lin_vel_cmd, float ang_vel_cmd)
{
	{// MUTEX LOCK
		std::lock_guard<std::mutex> lock(drive_cmd);
		linear_velocity_cmd = lin_vel_cmd;
		angular_velocity_cmd = ang_vel_cmd;
	}// MUTEX UNLOCK
}

void M4Base::execute_drive()
{
	// Differential drive controller
	//float right_wheel_velocity, left_wheel_velocity = 0;
	{// MUTEX LOCK
		std::lock_guard<std::mutex> lock(drive_cmd);
		right_wheel_velocity = ((linear_velocity_cmd - 4 * angular_velocity_cmd * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);
		left_wheel_velocity = ((linear_velocity_cmd + 4 * angular_velocity_cmd * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);

		// Expand mutex to cover assignment and avoid reading mid-change
		std::vector<float> wheel_velocity_list = {left_wheel_velocity, right_wheel_velocity, left_wheel_velocity, right_wheel_velocity};
		wheel_velocity_cmd = get_active_wheel_velocities(wheel_velocity_list);
		wheel_velocity_cmd_ids = active_wheel_ids;
	}// MUTEX UNLOCK

	// Keep hardware write outside lock
	dxl.write_velocity(wheel_velocity_cmd_ids, wheel_velocity_cmd);

}

void M4Base::crouch1(){
	command_queue.push(M4Commands::CMD_CROUCH1);
	// Wait until it completes
	while(current_configuration != M4Configurations::CROUCH1){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::crouch2(){
	command_queue.push(M4Commands::CMD_CROUCH2);
	// Wait until it completes
	while(current_configuration != M4Configurations::CROUCH2){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::stand(){
    command_queue.push(M4Commands::CMD_STAND);
	// Wait until it completes
	while(current_configuration != M4Configurations::STAND){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::sit(){
    command_queue.push(M4Commands::CMD_SIT);
	// Wait until it completes
	while(current_configuration != M4Configurations::SIT){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::uav(){
    command_queue.push(M4Commands::CMD_UAV);
	// Wait until it completes
	while(current_configuration != M4Configurations::UAV){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::mip(){
    command_queue.push(M4Commands::CMD_MIP);
	// Wait until it completes
	while(current_configuration != M4Configurations::MIP){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

M4Configurations M4Base::get_current_configuration()
{
	return current_configuration;
}

void RobotState::print()
{
	std::cout << "Robot state " << timestamp << std::endl;
	std::cout << "Position (deg): "; for(int i=0; i<positions.size(); i++) std::cout << positions[i] << " "; std::cout << std::endl;
	std::cout << "Velocities (deg/sec): "; for(int i=0; i<velocities.size(); i++) std::cout << velocities[i] << " "; std::cout << std::endl;
	std::cout << "Current (A): "; for(int i=0; i<currents.size(); i++) std::cout << currents[i] << " "; std::cout << std::endl;
	std::cout << std::endl;
}

std::vector<float> M4Base::get_active_joint_positions(const std::vector<float>& joint_positions) 
{
    std::vector<float> active_joint_positions;
    
    // Using a map for efficient look-up of joint position based on joint ID
    std::unordered_map<int, float> joint_id_to_position;
    for (size_t i = 0; i < all_joint_ids.size(); ++i) {
        joint_id_to_position[all_joint_ids[i]] = joint_positions[i];
    }
    
    // Populating the active_joint_positions vector
    for (int id : active_joint_ids) {
        active_joint_positions.push_back(joint_id_to_position[id]);
    }
    
    return active_joint_positions;
}

std::vector<std::string> M4Base::get_active_joint_names() 
{
    std::vector<std::string> active_joint_names;
    
    // Using a map for efficient look-up of joint name based on joint ID
    std::unordered_map<int, std::string> joint_id_to_name;
    for (size_t i = 0; i < all_joint_ids.size(); ++i) {
        joint_id_to_name[all_joint_ids[i]] = all_joint_names[i];
    }
    
    // Populating the active_joint_names vector
    for (int id : active_joint_ids) {
        active_joint_names.push_back(joint_id_to_name[id]);
    }
    
    return active_joint_names;
}

std::vector<float> M4Base::get_active_wheel_velocities(const std::vector<float>& wheel_velocities) 
{
    std::vector<float> get_active_wheel_velocities;
    
    // Using a map for efficient look-up of wheel position based on wheel ID
    std::unordered_map<int, float> wheel_id_to_velocity;
    for (size_t i = 0; i < all_wheel_ids.size(); ++i) {
        wheel_id_to_velocity[all_wheel_ids[i]] = wheel_velocities[i];
    }
    
    // Populating the active_wheel_positions vector
    for (int id : active_wheel_ids) {
        get_active_wheel_velocities.push_back(wheel_id_to_velocity[id]);
    }
    
    return get_active_wheel_velocities;
}

std::vector<std::string> M4Base::get_active_wheel_names() 
{
    std::vector<std::string> active_wheel_names;
    
    // Using a map for efficient look-up of	wheel name based on wheel id
	std::unordered_map<int, std::string> wheel_id_to_name;
    for (size_t i = 0; i < all_wheel_ids.size(); ++i) {
        wheel_id_to_name[all_wheel_ids[i]] = all_wheel_names[i];
    }
    
    // Populating the active_wheel_names vector
    for (int id : active_wheel_ids) {
        active_wheel_names.push_back(wheel_id_to_name[id]);
    }
    
    return active_wheel_names;
}

// Lock for getters declared in m4_base.h
std::vector<float> M4Base::get_wheel_velocity_cmd()
{
	std::lock_guard<std::mutex> lock(drive_cmd);
	return wheel_velocity_cmd;
}

float M4Base::get_right_wheel_velocity()
{
	std::lock_guard<std::mutex> lock(drive_cmd);
	return right_wheel_velocity;
}

float M4Base::get_left_wheel_velocity()
{
	std::lock_guard<std::mutex> lock(drive_cmd);
	return left_wheel_velocity;
}

float M4Base::get_linear_velocity_cmd()
{
	std::lock_guard<std::mutex> lock(drive_cmd);
	return linear_velocity_cmd;
}

float M4Base::get_angular_velocity_cmd()
{
	std::lock_guard<std::mutex> lock(drive_cmd);
	return angular_velocity_cmd;
}
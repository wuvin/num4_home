#include <stdio.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "utils.h"
#include "ConsoleFormatting.h"
#include "DynamixelInterface.h"


DynamixelInterface::DynamixelInterface()
{
	initialized = false;
}

DynamixelInterface::DynamixelInterface(const char* port, const int &baudrate, const std::vector<int> &joint_ids, const std::vector<int> &wheel_ids)
{
	init(port, baudrate, joint_ids, wheel_ids);
}

void DynamixelInterface::init(const char* port, const int &baudrate, const std::vector<int> &joint_ids, const std::vector<int> &wheel_ids)
/*
	port: Serial port name
	baudrate: Serial port baudrate set on Dynamixel using Dynamixel Wizard
	ids: List of motor IDs to connect to (IDs set on Dynamixel using Dynamixel Wizard).
*/
{
	active_joint_ids = joint_ids;
	active_wheel_ids = wheel_ids;
	joint_configs.resize(active_joint_ids.size());
	wheel_configs.resize(active_wheel_ids.size());
	active_joint_names.resize(active_joint_ids.size());
	active_wheel_names.resize(active_wheel_ids.size());
	servo_names.resize(active_joint_ids.size() + active_wheel_ids.size());
	
	// Load configuration variables
	//---------------------------------------------------------------------------------------------------------------
	// PROJECT_ROOT_DIR set by CMakeLists.txt

	printf("[INFO] Loading Dynamixel Params...\n");

	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/motor_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);
	
	ADDR_GOAL_POSITION = config["ADDR_GOAL_POSITION"].as<int>();
	ADDR_GOAL_VELOCITY = config["ADDR_GOAL_VELOCITY"].as<int>();
	ADDR_TORQUE_ENABLE = config["ADDR_TORQUE_ENABLE"].as<int>();
	ADDR_MOVEMENT_SPEED = config["ADDR_MOVEMENT_SPEED"].as<int>();
	ADDR_MOVING_STATUS = config["ADDR_MOVING_STATUS"].as<int>();
	ADDR_PRESENT_POSITION = config["ADDR_PRESENT_POSITION"].as<int>();
	ADDR_PRESENT_VELOCITY = config["ADDR_PRESENT_VELOCITY"].as<int>();
	ADDR_PRESENT_CURRENT = config["ADDR_PRESENT_CURRENT"].as<int>();
	LEN_TORQUE_ENABLE = config["LEN_TORQUE_ENABLE"].as<int>();
	LEN_GOAL_POSITION = config["LEN_GOAL_POSITION"].as<int>();
	LEN_GOAL_VELOCITY = config["LEN_GOAL_VELOCITY"].as<int>();
	LEN_MOVEMENT_SPEED = config["LEN_MOVEMENT_SPEED"].as<int>();
	LEN_MOVING_STATUS = config["LEN_MOVING_STATUS"].as<int>();
	LEN_PRESENT_POSITION = config["LEN_PRESENT_POSITION"].as<int>();
	LEN_PRESENT_VELOCITY = config["LEN_PRESENT_VELOCITY"].as<int>();
	LEN_PRESENT_CURRENT = config["LEN_PRESENT_CURRENT"].as<int>();
	MULT_SPEED = config["MULT_SPEED"].as<float>();
	MULT_POSITION = config["MULT_POSITION"].as<float>();
	MULT_CURRENT = config["MULT_CURRENT"].as<float>();
	PROTOCOL_VERSION = config["PROTOCOL_VERSION"].as<float>();
	TORQUE_ENABLE = config["TORQUE_ENABLE"].as<int>();
	TORQUE_DISABLE = config["TORQUE_DISABLE"].as<int>();

	printf("[INFO] Loaded Dynamixel Params.\n");

	printf("[INFO] Loading Robot Joint Params...\n");

	configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	config = YAML::LoadFile(configFilePath);


	// Load joint configs
	std::vector<std::string> joint_names = config["joint_names"].as<std::vector<std::string>>();

	for (int i = 0; i < joint_names.size(); i++)
	{
		int jid = config["joints"][joint_names[i]]["id"].as<int>();
		// Check if the joint ID is in the list of active joints
		auto j_index = std::find(active_joint_ids.begin(), active_joint_ids.end(), jid);
        if(j_index != active_joint_ids.end())
		{
			JointConfig jconfig;
			jconfig.name = joint_names[i];
			jconfig.id = jid;
            jconfig.zero_tick = config["joints"][joint_names[i]]["zero_tick"].as<int>();
			jconfig.max_tick = config["joints"][joint_names[i]]["max_tick"].as<int>();
			jconfig.min_tick = config["joints"][joint_names[i]]["min_tick"].as<int>();
			jconfig.speed = config["joints"][joint_names[i]]["speed"].as<float>();
			joint_configs[j_index - active_joint_ids.begin()] = jconfig;
			active_joint_names[j_index - active_joint_ids.begin()] = joint_names[i];
		}
	}

	// Load wheel configs
	std::vector<std::string> wheel_names = config["wheel_names"].as<std::vector<std::string>>();

	for (int i = 0; i < wheel_names.size(); i++)
	{
		int wid = config["wheels"][wheel_names[i]]["id"].as<int>();
		// Check if the wheel ID is in the list of active wheels
		auto w_index = std::find(active_wheel_ids.begin(), active_wheel_ids.end(), wid);
		if(w_index != active_wheel_ids.end())
		{
			WheelConfig wconfig;
			wconfig.name = wheel_names[i];
			wconfig.id = wid;
			wconfig.max_speed = config["wheels"][wheel_names[i]]["max_speed"].as<int>();
			wconfig.min_speed = config["wheels"][wheel_names[i]]["min_speed"].as<int>();
			wheel_configs[w_index - active_wheel_ids.begin()] = wconfig;
			active_wheel_names[w_index - active_wheel_ids.begin()] = wheel_names[i];
		}
	}

	printf("[INFO] Loaded Robot Joint Params.\n");
	
	printf("%sInitializing dynamixel with settings:\n%s", CONSOLE_GREEN, CONSOLE_DEFAULT);
	printf("\tADDR_TORQUE_ENABLE: %d\n", ADDR_TORQUE_ENABLE);
	printf("\tADDR_GOAL_POSITION: %d\n", ADDR_GOAL_POSITION);
	printf("\tADDR_GOAL_VELOCITY: %d\n", ADDR_GOAL_VELOCITY);
	printf("\tADDR_MOVEMENT_SPEED: %d\n", ADDR_MOVEMENT_SPEED);
	printf("\tADDR_MOVING_STATUS: %d\n", ADDR_MOVING_STATUS);
	printf("\tADDR_PRESENT_POSITION: %d\n", ADDR_PRESENT_POSITION);
	printf("\tADDR_PRESENT_VELOCITY: %d\n", ADDR_PRESENT_VELOCITY);
	printf("\tADDR_PRESENT_CURRENT: %d\n", ADDR_PRESENT_CURRENT);
	printf("\tLEN_TORQUE_ENABLE: %d\n", LEN_TORQUE_ENABLE);
	printf("\tLEN_GOAL_POSITION: %d\n", LEN_GOAL_POSITION);
	printf("\tLEN_GOAL_VELOCITY: %d\n", LEN_GOAL_VELOCITY);
	printf("\tLEN_MOVEMENT_SPEED: %d\n", LEN_MOVEMENT_SPEED);
	printf("\tLEN_MOVING_STATUS: %d\n", LEN_MOVING_STATUS);
	printf("\tLEN_PRESENT_POSITION: %d\n", LEN_PRESENT_POSITION);
	printf("\tLEN_PRESENT_VELOCITY: %d\n", LEN_PRESENT_VELOCITY);
	printf("\tLEN_PRESENT_CURRENT: %d\n", LEN_PRESENT_CURRENT);
	printf("\tMULT_SPEED: %f\n", MULT_SPEED);
	printf("\tMULT_POSITION: %f\n", MULT_POSITION);
	printf("\tMULT_VELOCITY: %f\n", MULT_VELOCITY);
	printf("\tMULT_CURRENT: %f\n", MULT_CURRENT);
	printf("\tPROTOCOL_VERSION: %f\n", PROTOCOL_VERSION);
	printf("\tTORQUE_ENABLE: %d\n", TORQUE_ENABLE);
	printf("\tTORQUE_DISABLE: %d\n", TORQUE_DISABLE);
	printf("\n%sJoint configurations:\n%s", CONSOLE_GREEN, CONSOLE_DEFAULT);
	for(int i=0; i < joint_configs.size(); i++)
	{
		printf("\t%s:\n",joint_configs[i].name.c_str());
		printf("\t\tID: %d\n\t\tzero_tick: %d\n\t\tmin_tick: %d\n\t\tmax_tick: %d\n\t\tspeed: %f\n\n",
			joint_configs[i].id,
			joint_configs[i].zero_tick,
			joint_configs[i].min_tick,
			joint_configs[i].max_tick,
			joint_configs[i].speed);
	}
	printf("\n%sWheel configurations:\n%s", CONSOLE_GREEN, CONSOLE_DEFAULT);
	for(int i=0; i < wheel_configs.size(); i++)
	{
		printf("\t%s:\n",wheel_configs[i].name.c_str());
		printf("\t\tID: %d\n\t\tmin_speed: %d\n\t\tmax_speed: %d\n\n",
			wheel_configs[i].id,
			wheel_configs[i].min_speed,
			wheel_configs[i].max_speed);
	}

	servo_names.insert(servo_names.end(), active_joint_names.begin(), active_joint_names.end());
	servo_names.insert(servo_names.end(), active_wheel_names.begin(), active_wheel_names.end());
	
	//---------------------------------------------------------------------------------------------------------------

	portHandler = dynamixel::PortHandler::getPortHandler(port);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Connect to all motors and enable torque
	if(!connect(baudrate))
	{
		throw std::string("Failed to connect!\n");
        return;
	}
	
	// Set up read handler
	read_data_length = LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT;
	read_start_address =  ADDR_PRESENT_CURRENT;
	bool dxl_addparam_result = false;

	groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, read_start_address, read_data_length);

	// Add other motors read
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_addparam_result = groupSyncRead->addParam(active_joint_ids[i]);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", active_joint_ids[i]);
			exit(1);
		}
	}

	// Also register wheel IDs with groupSyncRead and add wheels
	for(int ii = 0; ii < active_wheel_ids.size(); ii++)
	{
		dxl_addparam_result = groupSyncRead->addParam(active_wheel_ids[ii]);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", active_wheel_ids[ii]);
			exit(1);
		}
	}

	groupSyncWritePosition = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
	groupSyncWriteMoveSpeed = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MOVEMENT_SPEED, LEN_MOVEMENT_SPEED);
	groupSyncWriteVelocity = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

	initialized = true;
	running = true;
	clock_start = std::chrono::high_resolution_clock::now();

	// Set the speed of each motor
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		write_movement_speed(active_joint_ids[i], joint_configs[i].speed);
	}

	initialized = true;
	running = true;
	clock_start = std::chrono::high_resolution_clock::now();
}

void DynamixelInterface::stop()
{
	disconnect();
	running = false;
}

DynamixelInterface::~DynamixelInterface()
{	
	if(running)
		stop();
}

int DynamixelInterface::connect(const int &baudrate)
{
	// open port
	if(portHandler->openPort())
		printf("Succeeded to open the port\n");
	else
	{
        printf("Failed to open the port\n");
        return 0;
    }

    // set baudrate
	if(portHandler->setBaudRate(baudrate))
        printf("Succeeded to change the baudrate\n");
	else
	{
        printf("Failed to change the baudrate\n");
        return 0;
    }

	// enable torque and connect to servos
	uint8_t dxl_error = 0;  
	int dxl_comm_result = COMM_TX_FAIL;

	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_joint_ids[i]);
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			return 0;
		}
		else if (dxl_error != 0)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_joint_ids[i]);
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			return 0;
		}
		else
		{
			printf("Dynamixel#%2d has been successfully connected \n", active_joint_ids[i]);
		}
	}

	for(int i = 0; i < active_wheel_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_wheel_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_wheel_ids[i]);
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			return 0;
		}
		else if (dxl_error != 0)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_wheel_ids[i]);
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			return 0;
		}
		else
		{
			printf("Dynamixel#%2d has been successfully connected \n", active_wheel_ids[i]);
		}
	}

	running = true;

	return 1;
}

int DynamixelInterface::disconnect()
{
	printf("\nDisconnecting from interface!\n");
	// disable torque and disconnect from servos
    uint8_t dxl_error = 0;  
    int dxl_comm_result = COMM_TX_FAIL;
    
	// Disconnect all other motors
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result!= COMM_SUCCESS)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_joint_ids[i]);
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error!= 0)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_joint_ids[i]);
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
		{
            printf("Dynamixel#%d has been successfully disconnected \n", active_joint_ids[i]);
        }	
	}

	for(int i = 0; i < active_wheel_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_wheel_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result!= COMM_SUCCESS)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_wheel_ids[i]);
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error!= 0)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_wheel_ids[i]);
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
		{
            printf("Dynamixel#%d has been successfully disconnected \n", active_wheel_ids[i]);
        }	
	}

	portHandler->closePort();

	running = false;

	return 1;
}

int DynamixelInterface::toggle_torque(std::vector<int> ids, bool state)
{
	uint8_t dxl_error = 0;  
    int dxl_comm_result = COMM_TX_FAIL;

	int torque_state;
	if(state)
		torque_state = TORQUE_ENABLE;
	else
		torque_state = TORQUE_DISABLE;

	if(!IsSubset(active_joint_ids, ids) && !IsSubset(active_wheel_ids, ids))
	{
		printf("\nSome IDs were not in active IDs! Ignoring command.\n");
		return 0;
	}

	for(int i = 0; i < ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ids[i], ADDR_TORQUE_ENABLE, torque_state, &dxl_error);
        if (dxl_comm_result!= COMM_SUCCESS)
        {
            printf("\nUnable to change torque for Dynamixel %d\n\n", ids[i]);
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error!= 0)
        {
            printf("\nUnable to change torque for Dynamixel %d\n\n", ids[i]);
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
		{
            printf("Dynamixel#%d torque has been successfully toggled. \n", ids[i]);
        }	
	}

	return 1;
}

DynamixelStatus DynamixelInterface::read_status()
{
	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;
	bool dxl_getdata_result = false;

	// Perform a sync read to get status
	dxl_comm_result = groupSyncRead->txRxPacket();
	auto now = std::chrono::high_resolution_clock::now();
	double timestamp = get_duration_secs(clock_start, now);

	DynamixelStatus status;
	status.timestamp = timestamp;
	status.position.resize(active_joint_ids.size() + active_wheel_ids.size());
	status.velocity.resize(active_joint_ids.size() + active_wheel_ids.size());
	status.current.resize(active_joint_ids.size() + active_wheel_ids.size());
	
	// Check for comms success
	if (dxl_comm_result != COMM_SUCCESS)
	{
		fprintf(stderr, "[groupSyncRead] txRxPacket read failed\n\n");
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}

	// Check for servo errors
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		if(groupSyncRead->getError(active_joint_ids[i], &dxl_error))
		{
			printf("[ID:%03d] %s\n", active_joint_ids[i], packetHandler->getRxPacketError(dxl_error));
		}
	}

	// Save data from buffer
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		 dxl_getdata_result = groupSyncRead->isAvailable(active_joint_ids[i], read_start_address, read_data_length);
		 if (dxl_getdata_result != true)
		 {
			fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", active_joint_ids[i]);
		 }

		JointConfig joint_config = get_joint_config(active_joint_ids[i]);

		status.position[i] = MULT_POSITION * (twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), LEN_PRESENT_POSITION) - joint_config.zero_tick);
		status.velocity[i] = MULT_SPEED * twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY), LEN_PRESENT_VELOCITY);
		status.current[i] = MULT_CURRENT * short(twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT), LEN_PRESENT_CURRENT));
	}

    for(int ii = 0; ii < active_wheel_ids.size(); ii++)
    {
        dxl_getdata_result = groupSyncRead->isAvailable(active_wheel_ids[ii], read_start_address, read_data_length);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", active_wheel_ids[ii]);
        }

        WheelConfig wheel_config = get_wheel_config(active_wheel_ids[ii]);

		status.position[active_joint_ids.size() + ii] = MULT_POSITION * (twos_complement(groupSyncRead->getData(active_wheel_ids[ii], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), LEN_PRESENT_POSITION));
		status.velocity[active_joint_ids.size() + ii] = MULT_SPEED * twos_complement(groupSyncRead->getData(active_wheel_ids[ii], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY), LEN_PRESENT_VELOCITY);
		status.current[active_joint_ids.size() + ii] = MULT_CURRENT * short(twos_complement(groupSyncRead->getData(active_wheel_ids[ii], ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT), LEN_PRESENT_CURRENT));
	}

	// for(int i = 0; i < active_wheel_ids.size(); i++)
	// {
	// 	 dxl_getdata_result = groupSyncRead->isAvailable(active_wheel_ids[i], read_start_address, read_data_length);
	// 	 if (dxl_getdata_result != true)
	// 	 {
	// 		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", active_wheel_ids[i]);
	// 	 }

	// 	WheelConfig wheel_configs = get_wheel_config(active_wheel_ids[i]);

	// 	status.position[active_joint_ids.size() + i] = MULT_POSITION * (twos_complement(groupSyncRead->getData(active_wheel_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), LEN_PRESENT_POSITION));
	// 	status.velocity[active_joint_ids.size() + i] = MULT_SPEED * twos_complement(groupSyncRead->getData(active_wheel_ids[i], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY), LEN_PRESENT_VELOCITY);
	// 	status.current[active_joint_ids.size() + i] = MULT_CURRENT * short(twos_complement(groupSyncRead->getData(active_wheel_ids[i], ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT), LEN_PRESENT_CURRENT));
	// }
	
	status.names = servo_names;

	return status;
}

JointConfig DynamixelInterface::get_joint_config(int id)
{
	for(JointConfig joint : joint_configs)
	{
		if(joint.id == id)
			return joint;
	}


	return JointConfig();
}

WheelConfig DynamixelInterface::get_wheel_config(int id)
{
	for(WheelConfig wheel : wheel_configs)
	{
		if(wheel.id == id)
			return wheel;
	}


	return WheelConfig();
}

int DynamixelInterface::write_position(std::vector<int> ids, std::vector<float> positions)
{
	if(ids.size() != positions.size())
	{
		fprintf(stderr, "[set_goal_position] Size mismatch between list of IDs and Positions.\n");
        return 0;
	}
	if(IsSubset(active_joint_ids, ids) != true)
	{
		fprintf(stderr, "[set_goal_position] Some motor IDs provided were not initialized.\n");
        return 0;
	}

	// Convert positions from degrees to ticks and add zero offset
	for(int i = 0; i < positions.size(); i++)
	{
		JointConfig jconfig = joint_configs[std::find(active_joint_ids.begin(), active_joint_ids.end(), ids[i]) - active_joint_ids.begin()];
		positions[i] = positions[i] / MULT_POSITION + jconfig.zero_tick;
		// Warn if positions are out of range
		if(positions[i] < jconfig.min_tick)
		{
			printf("%s[WARN]: [set_goal_position] Position %d out of range for Joint %s%s\n", CONSOLE_MSG_WARN, int(positions[i]), jconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving to min position %d instead", jconfig.min_tick);
			positions[i] = jconfig.min_tick;
		}
		else if(positions[i] > jconfig.max_tick)
		{
			printf("%s[WARN]: [set_goal_position] Position %d out of range for Joint %s%s\n", CONSOLE_MSG_WARN, int(positions[i]), jconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving to max position %d instead\n", jconfig.max_tick);
			positions[i] = jconfig.max_tick;
		}
		
		// printf("[INFO]: Writing position %d=%f deg to joint %s\n", int(positions[i]), (positions[i] - jconfig.zero_tick)*MULT_POSITION, jconfig.name.c_str());
	}

	int dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	uint8_t param_goal_position[4];

	// Load goal positions to buffer
    for(int i = 0; i < ids.size(); i++)
	{
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));

		dxl_addparam_result = groupSyncWritePosition->addParam(ids[i], param_goal_position);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed\n", ids[i]);
			return 0;
		}
	}

	// Send buffer to servos
	dxl_comm_result = groupSyncWritePosition->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) 
	{
        fprintf(stderr, "[groupSyncWritePosition] txPacket write failed\n");
        return 0;
    }

	// Clear syncwrite parameter storage buffer
	groupSyncWritePosition->clearParam();
	return 1;
}

int DynamixelInterface::write_movement_speed(int id, float movement_speed)
{
	printf("[INFO] Setting movement speed for joint id %d to %03f degrees per second\n", id, movement_speed);

	// Convert speed from degrees per second to rotations per minute
	movement_speed = movement_speed / MULT_SPEED;

	int dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	uint8_t param_goal_velocity[4];

	// Load goal velocity to buffer
   	param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(movement_speed));
	param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(movement_speed));
	param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(movement_speed));
	param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(movement_speed));

	dxl_addparam_result = groupSyncWriteMoveSpeed->addParam(id, param_goal_velocity);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWriteMoveSpeed addparam failed", id);
		return 0;
	}
	

	// Send buffer to servos
	dxl_comm_result = groupSyncWriteMoveSpeed->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) 
	{
        fprintf(stderr, "[groupSyncWriteMoveSpeed] txPacket write failed");
        return 0;
    }

	// Clear syncwrite parameter storage buffer
	groupSyncWriteMoveSpeed->clearParam();

	printf("Speed set!\n");

	return 1;
}

int DynamixelInterface::write_velocity(std::vector<int> ids, std::vector<float> velocities)
{
	if(ids.size() != velocities.size())
	{
		fprintf(stderr, "[set_goal_velocity] Size mismatch between list of IDs and Velocities.\n");
        return 0;
	}
	if(IsSubset(active_wheel_ids, ids) != true)
	{
		fprintf(stderr, "[set_goal_velocity] Some motor IDs provided were not initialized.\n");
        return 0;
	}

	// Convert velocities from degrees/s to ticks/s
	for(int i = 0; i < velocities.size(); i++)
	{
		WheelConfig wconfig = wheel_configs[std::find(active_wheel_ids.begin(), active_wheel_ids.end(), ids[i]) - active_wheel_ids.begin()];
		velocities[i] = velocities[i] / MULT_SPEED;
		// Warn if velocities are out of range
		if(velocities[i] < wconfig.min_speed)
		{
			printf("%s[WARN]: [set_goal_velocity] Velocity %d out of range for Joint %s%s\n", CONSOLE_MSG_WARN, int(velocities[i]), wconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving at min velocity %d instead", wconfig.min_speed);
			velocities[i] = wconfig.min_speed;
		}
		else if(velocities[i] > wconfig.max_speed)
		{
			printf("%s[WARN]: [set_goal_velocities] Velocity %d out of range for Joint %s%s\n", CONSOLE_MSG_WARN, int(velocities[i]), wconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving at max velocity %d instead\n", wconfig.max_speed);
			velocities[i] = wconfig.max_speed;
		}
		
		std::cout << velocities[i] << " ";
		// printf("[INFO]: Writing position %d=%f deg to joint %s\n", int(positions[i]), (positions[i] - jconfig.zero_tick)*MULT_POSITION, jconfig.name.c_str());
	}
	std::cout << std::endl;

	int dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	uint8_t param_goal_velocity[4];

	// Load goal positions to buffer
    for(int i = 0; i < ids.size(); i++)
	{
		uint32_t vel_unsigned = static_cast<int32_t>(velocities[i]) & 0xFFFFFFFF;
		param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel_unsigned));
		param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel_unsigned));
		param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel_unsigned));
		param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel_unsigned));
		
		dxl_addparam_result = groupSyncWriteVelocity->addParam(ids[i], param_goal_velocity);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWriteVelocity addparam failed\n", ids[i]);
			return 0;
		}
	}

	// Send buffer to servos
	dxl_comm_result = groupSyncWriteVelocity->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) 
	{
        fprintf(stderr, "[groupSyncWriteVelocity] txPacket write failed\n");
        return 0;
    }

	// Clear syncwrite parameter storage buffer
	groupSyncWriteVelocity->clearParam();
	return 1;
}

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_controls.h"
#include "SCServo.h"

namespace waveshare_ros2_control
{
class WaveshareHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(WaveshareHardwareInterface)

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    WAVESHARE_SERVOS_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    WAVESHARE_SERVOS_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

private:
    // helper motor functions
    double get_position(int ID);
    double get_velocity(int ID);
    double get_torque(int ID);
    double get_temperature(int ID);

    void write_pos();
    void write_vel();
	

    // motor variables
    int baudrate_ = 1000000;
    std::string port_ = "/dev/ttyACM0"; // /dev/ttyTHS1 if using UART
    SMS_STS sm_st;
    double KT_ = 9.0; // torque constant (kg*cm / A)
    int steps_ = 4096;
    u16 max_speed_ = 6000; // 6000;
    u8 max_acc_ = 150; // 150;
    // id group variables
    std::vector<u8> all_ids_;
    std::vector<u8> pos_ids_;
    std::vector<u8> vel_ids_;
    std::vector<int> pos_is_;
    std::vector<int> vel_is_;
    // command interface variables
    std::vector<double> pos_cmds_;
    std::vector<double> vel_cmds_;
    // state interface variables
    std::vector<double> pos_states_;
    std::vector<double> vel_states_;
    std::vector<double> torq_states_;
    std::vector<double> temp_states_;
    // vector for position offsets
    std::vector<double> pos_offsets_;
	// vector for gear ratios
	std::vector<double> gear_ratios_; // ###
    // array variables for motors
    u8*  p_ids_pnt_;
    u8*  v_ids_pnt_;
    s16* p_pos_ar_;
    u16* p_vel_ar_;
    u8*  p_acc_ar_;
    s16* v_vel_ar_;
    u8*  v_acc_ar_;
};

} // namespace waveshare_ros2_control

// THE MAIN CODE BEGINS HERE
namespace waveshare_ros2_control
{
hardware_interface::CallbackReturn WaveshareHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
	if (
		hardware_interface::SystemInterface::on_init(info) !=
    	hardware_interface::CallbackReturn::SUCCESS)
  	{
    	return hardware_interface::CallbackReturn::ERROR;
  	}
	// check urdf definitions
	pos_offsets_.resize(info_.joints.size(), 0.0);
	gear_ratios_.resize(info_.joints.size(), 1.0); // default to 1.0 (no gearing)
	int i = 0;
	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		all_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
		// check num, order, and type of state interfaces
		if (joint.state_interfaces.size() != 4)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
				"joint has the wrong number of state interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
				"a joint does not have the position state interface first");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
				"a joint does not have the velocity state interface second");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[2].name != "torque")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
				"a joint does not have the torque state interface third");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[3].name != "temperature")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
				"a joint does not have the temperature state interface fourth");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// check presence and types of command interfaces
		if (joint.command_interfaces.size() < 1)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"), 
				"a joint does not have a command interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		for (long unsigned int ci = 0; ci < joint.command_interfaces.size(); ci++)
		{
			if (joint.command_interfaces[ci].name != hardware_interface::HW_IF_POSITION &&
				joint.command_interfaces[ci].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"),
					"a joint is using a command interface that isn't position or velocity");
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		// store ids in different vectors by type
		if (joint.parameters.find("type")->second == "pos")
		{
			pos_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			pos_is_.emplace_back(i);
		} 
		else if (joint.parameters.find("type")->second == "vel") 
		{
			vel_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			vel_is_.emplace_back(i);
		}
		else 
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"), 
				"a joint has the wrong type, it should be vel or pos");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// save pose offsets to work around motor movement limitations
		auto offset = joint.parameters.find("offset");
    	if (offset != joint.parameters.end())
    	{
      		pos_offsets_[i] = std::stod(offset->second);  
    	}
		// save gear ratios
        auto gear_ratio_param = joint.parameters.find("gear_ratio"); // 
        if (gear_ratio_param != joint.parameters.end()) // 
        { // 
            gear_ratios_[i] = std::stod(gear_ratio_param->second); // 
            if (gear_ratios_[i] == 0) // 
            { // 
                RCLCPP_FATAL(rclcpp::get_logger("waveshare_ros2_control"), // 
                    "gear_ratio for joint '%s' cannot be zero.", joint.name.c_str()); // 
                return hardware_interface::CallbackReturn::ERROR; // 
            } // 
        } // 
		i++;
	}
	// init vectors for state interfaces
	pos_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	vel_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	torq_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	temp_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	// create vectors for command interfaces
	pos_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	vel_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareHardwareInterface::on_configure(
  	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// start servo communication
	if (!sm_st.begin(baudrate_, port_.c_str()))
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	// ping motors
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		if (sm_st.Ping(all_ids_[i]) == -1)
		{
			RCLCPP_WARN(rclcpp::get_logger("waveshare_ros2_control"), 
				"unable to ping motor id '%d'", all_ids_[i]);
		}
	}
	// pointers to ids for motor control
	p_ids_pnt_ = &pos_ids_[0];
	v_ids_pnt_ = &vel_ids_[0];
	// arrays for servo commands
	p_pos_ar_ = new s16[pos_ids_.size()];
	p_vel_ar_ = new u16[pos_ids_.size()];
	p_acc_ar_ = new  u8[pos_ids_.size()];
	v_vel_ar_ = new s16[vel_ids_.size()];
	v_acc_ar_ = new  u8[vel_ids_.size()];
	// set motor modes: 0 = servo, 1 = closed loop wheel; set max acceleration
	for (u8 i = 0; i < pos_ids_.size(); i++)
	{
		sm_st.Mode(pos_ids_[i], 0); 
		p_acc_ar_[i] = 1; // smoother motion with no acceleration
	}
	for (u8 i = 0; i < vel_ids_.size(); i++)
	{
		sm_st.Mode(vel_ids_[i], 1);
		v_acc_ar_[i] = max_acc_;
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareHardwareInterface::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "torque", &torq_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "temperature", &temp_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareHardwareInterface::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		for (long unsigned int ci = 0; ci < info_.joints[i].command_interfaces.size(); ci++)
		{
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_POSITION) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmds_[i]));
			}
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_VELOCITY) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_cmds_[i]));
			}
		}
	}
	return command_interfaces;
}

hardware_interface::CallbackReturn WaveshareHardwareInterface::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set position commands to current positions before any movement to not move on start
	for (size_t i = 0; i < all_ids_.size(); i++)
  	{
    	//double init_raw_pos = get_position(all_ids_[i]);
    	//pos_cmds_[i] = init_raw_pos - pos_offsets_[i]; //old
		double init_raw_pos = get_position(all_ids_[i]);
		pos_cmds_[i] = (init_raw_pos - pos_offsets_[i]);

		// print statements for debugging
		RCLCPP_INFO(rclcpp::get_logger("waveshare_ros2_control"),
		"Motor ID: %d, Initial Position: %f, Offset: %f, Gear Ratio: %f, Commanded Position: %f",
		all_ids_[i], init_raw_pos, pos_offsets_[i], gear_ratios_[i], pos_cmds_[i]);
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareHardwareInterface::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set velocities to 0 on close, doesn't work on ctrl-C
	for (size_t i = 0; i < vel_cmds_.size(); i++)
	{
    	vel_cmds_[i] = 0.0;
  	}
	auto now    = rclcpp::Clock().now();
  	auto period = rclcpp::Duration(0, 0);  // zero duration
  	this->write(now, period);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WaveshareHardwareInterface::read(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		double raw_motor_pos = get_position(all_ids_[i]); // motor position in radians
		// pos_states_[i] = raw_pos - pos_offsets_[i]; // 
		// pos_states_[i] = (raw_motor_pos / gear_ratios_[i]) - pos_offsets_[i]; // 
		pos_states_[i] = (raw_motor_pos - pos_offsets_[i]) / gear_ratios_[i] ; // 
		// vel_states_[i] = get_velocity(all_ids_[i]); // 
		vel_states_[i] = get_velocity(all_ids_[i]) / gear_ratios_[i]; // 

		// torq_states_[i] = get_torque(all_ids_[i]); // 
		torq_states_[i] = get_torque(all_ids_[i]) * gear_ratios_[i]; // 
		temp_states_[i] = get_temperature(all_ids_[i]);
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareHardwareInterface::write(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < pos_is_.size(); i++)
    {
        int joint_idx = pos_is_[i];
        // double pos = pos_cmds_[pos_is_[i]] + pos_offsets_[pos_is_[i]]; //
        // double motor_target_pos_rad = (pos_cmds_[joint_idx] + pos_offsets_[joint_idx]) * gear_ratios_[joint_idx]; //
		double motor_target_pos_rad = pos_cmds_[joint_idx] * gear_ratios_[joint_idx] + pos_offsets_[joint_idx];
        // p_pos_ar_[i] = (pos * steps_) / (2 * M_PI); // 
        p_pos_ar_[i] = (motor_target_pos_rad * steps_) / (2 * M_PI); // 

        // p_vel_ar_[i] = (vel_cmds_[pos_is_[i]] * steps_) / (2 * M_PI); // 
        double motor_target_vel_rad_s = vel_cmds_[joint_idx] * gear_ratios_[joint_idx]; //
        p_vel_ar_[i] = (motor_target_vel_rad_s * steps_) / (2 * M_PI); //
    }
    for (size_t i = 0; i < vel_is_.size(); i++)
    {
        int joint_idx = vel_is_[i];
        // v_vel_ar_[i] = (vel_cmds_[vel_is_[i]] * steps_) / (2 * M_PI); // 
        double motor_target_vel_rad_s = vel_cmds_[joint_idx] * gear_ratios_[joint_idx]; // 
        v_vel_ar_[i] = (motor_target_vel_rad_s * steps_) / (2 * M_PI); //
    }
    sm_st.SyncWritePosEx(p_ids_pnt_, static_cast<u8>(pos_ids_.size()), 
        p_pos_ar_, p_vel_ar_, p_acc_ar_); 
    sm_st.SyncWriteSpe(v_ids_pnt_, static_cast<u8>(vel_ids_.size()), 
        v_vel_ar_, v_acc_ar_); 
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn WaveshareHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
	sm_st.end();
	delete[] p_pos_ar_;
	delete[] p_vel_ar_;
	delete[] p_acc_ar_;
	delete[] v_vel_ar_;
	delete[] v_acc_ar_;
	return hardware_interface::CallbackReturn::SUCCESS;
}

double WaveshareHardwareInterface::get_position(int ID)
{
    double pos = sm_st.ReadPos(ID) * 2 * M_PI / steps_;
    return pos;
}

double WaveshareHardwareInterface::get_velocity(int ID)
{
    double vel = sm_st.ReadSpeed(ID) * 2 * M_PI / steps_; // rads / s
    return vel;
}


double WaveshareHardwareInterface::get_torque(int ID)
{
    // ReadCurrent(ID) return unitless value, multiply by static current (6mA)
    int current = sm_st.ReadCurrent(ID) * 6.0 / 1000.0;
    double torque = current * KT_;
    return torque;
}

double WaveshareHardwareInterface::get_temperature(int ID)
{
    double temp = static_cast<double>(sm_st.ReadTemper(ID));
    return temp;
}

}  // namespace waveshare_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    waveshare_ros2_control::WaveshareHardwareInterface, hardware_interface::SystemInterface)

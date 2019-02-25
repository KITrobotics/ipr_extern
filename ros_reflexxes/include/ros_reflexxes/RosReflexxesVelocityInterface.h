#ifndef ROS_REFLEXXES_VELOCITY_INTERFACE_H
#define ROS_REFLEXXES_VELOCITY_INTERFACE_H

#include <ros/ros.h>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <libreflexxestype2/RMLVelocityFlags.h>
#include <libreflexxestype2/RMLVelocityInputParameters.h>
#include <libreflexxestype2/RMLVelocityOutputParameters.h>

#include "ros_reflexxes/RosReflexxesInterface.h"

class RosReflexxesVelocityInterface : public RosReflexxesInterface
{
private:
    ros::NodeHandle nh_;
    //paramers
    int n_dim_;
    double period_;
    std::vector<double> max_velocities_;
    std::vector<double> max_acceleration_;
    std::vector<double> max_jerk_;

    RMLVelocityFlags flags_;
    boost::shared_ptr<RMLVelocityInputParameters> input_params_;
    boost::shared_ptr<RMLVelocityOutputParameters> output_params_;
    boost::shared_ptr<ReflexxesAPI> rml_;
    bool position_initialized_;
    bool load_parameters(std::string ns);

public:

    RosReflexxesVelocityInterface() {};
    RosReflexxesVelocityInterface(ros::NodeHandle nh) { init(nh); }
    RosReflexxesVelocityInterface(std::string nh) { init(ros::NodeHandle(nh)); }

    /**********************************************
     * methods implemented for  RosReflexxesInterface
     */
    bool init(ros::NodeHandle nh) override;
    void starting(const std::vector<double> &initial_command) override;
    std::vector<double> update() override; // advance reflexxes and return current velocity

    std::vector<double> get_target_command() override {
        return get_target_velocity();
    };

    void set_target_command(const std::vector<double> &command) override {
        set_target_velocity(command);
    };

    std::vector<double> get_current_position() override;
    std::vector<double> get_current_velocity() override;
    std::vector<double> get_current_acceleration() override;

    /**********************************************
     * custom methods
     */

    void advance_reflexxes();//advance reflexxes if initialized and position is known
    void reset_to_previous_state(RMLVelocityInputParameters previous_state);

    std::vector<double> get_target_velocity();
    double get_period();
    RMLVelocityInputParameters get_current_state();
    
    //setter functions
    void set_target_velocity(const std::vector<double> &t_vel) override;
    void set_target_position(const std::vector<double> &t_pos) override {
        ROS_ERROR("Cannot set position in RosReflexxesVelocityInterface");
    }
};

#endif

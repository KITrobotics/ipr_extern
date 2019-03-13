#ifndef ROS_REFLEXXES_POSITION_INTERFACE_H
#define ROS_REFLEXXES_POSITION_INTERFACE_H

#include <ros/ros.h>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <libreflexxestype2/RMLPositionFlags.h>
#include <libreflexxestype2/RMLPositionInputParameters.h>
#include <libreflexxestype2/RMLPositionOutputParameters.h>

#include "ros_reflexxes/RosReflexxesInterface.h"

class RosReflexxesPositionInterface : public RosReflexxesInterface
{
private:
    ros::NodeHandle nh_;
    //paramers
    int n_dim_ = 0;
    double period_ = 0;
    std::vector<double> max_velocities_;
    std::vector<double> max_acceleration_;
    std::vector<double> max_jerk_;
    
    
    RMLPositionFlags flags_;
    boost::shared_ptr<RMLPositionInputParameters> input_params_;
    boost::shared_ptr<RMLPositionOutputParameters> output_params_;
    boost::shared_ptr<ReflexxesAPI> rml_;
    bool position_initialized_ = false;;
    
    //functions
    bool load_parameters(std::string ns);
    
public:

    /**********************************************
     * methods implemented for  RosReflexxesInterface
     */

    RosReflexxesPositionInterface() : max_acceleration_(0), max_velocities_(0), max_jerk_(0) {};
    virtual ~RosReflexxesPositionInterface() {};
    RosReflexxesPositionInterface(ros::NodeHandle nh) { init(nh); }
    RosReflexxesPositionInterface(std::string nh) { init(ros::NodeHandle(nh)); }

    bool init(ros::NodeHandle nh) override;
    void starting(const std::vector<double> &c_pos) override;
    std::vector<double> update() override;//advance reflexxes and returns next position

    std::vector<double> get_target_command() override {
        return get_target_position();
    };

    void set_target_command(const std::vector<double> &command) override {
        set_target_position(command);
    };


    std::vector<double> get_current_position() override;
    std::vector<double> get_current_velocity() override;
    std::vector<double> get_current_acceleration() override;

    /**********************************************
     * custom methods
     */

    void reset_to_previous_state(RMLPositionInputParameters previous_state);
    void advance_reflexxes();//advance reflexxes if initialized and position is known
    
    //getter functions
    std::vector<double> get_target_position();
    RMLPositionInputParameters get_current_state();
    ros::Duration get_time_to_target_completedness();
    
    //setter functions
    void set_target_position(const std::vector<double> &t_pos) override;
    void set_target_velocity(const std::vector<double> &t_vel) override;
    
};


#endif

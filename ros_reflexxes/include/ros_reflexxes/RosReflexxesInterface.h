//
// Created by oke on 15.11.18.
//

#ifndef ROS_REFLEXXES_ROSREFLEXXESINTERFACE_H
#define ROS_REFLEXXES_ROSREFLEXXESINTERFACE_H

#include <vector>

#include <ros/ros.h>

class RosReflexxesInterface {

public:

	virtual ~RosReflexxesInterface() {};
    /**
     * ros methods
     */
    virtual bool init(ros::NodeHandle nh) = 0;
    virtual void starting(const std::vector<double> &initial_command) = 0;
    virtual std::vector<double> update() = 0; //advance reflexxes and return current state

    std::vector<double> update(const std::vector<double> &command) {
        set_target_command(command);
        return update();
    }

    /**
     * make ros interface generic
     */
    virtual std::vector<double> get_target_command() = 0;
    virtual void set_target_command(const std::vector<double> &command) = 0;
    virtual void set_target_position(const std::vector<double> &t_pos) = 0;
    virtual void set_target_velocity(const std::vector<double> &t_vel) = 0;

    /**
     * getter
     */
    virtual std::vector<double> get_current_position() = 0;
    virtual std::vector<double> get_current_velocity() = 0;
    virtual std::vector<double> get_current_acceleration() = 0;
};


#endif //ROS_REFLEXXES_ROSREFLEXXESINTERFACE_H

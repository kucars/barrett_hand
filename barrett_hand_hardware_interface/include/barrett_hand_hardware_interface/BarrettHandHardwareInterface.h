#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <boost/thread/mutex.hpp>
#include <math.h>
#include <sstream>

#include "openwam/CANbus.hh"

#include "openwamdriver.h"
#include "ft.hh"
#include "tactile.hh"
#include "bhd280.hh"


#define PI 3.14159265359
#define DEG_TO_RAD PI/180.0

bool isEqual(double & a, double & b, double threshold)
{
    return fabs(a-b)<threshold;
}

class BarrettHandHardwareInterface : public hardware_interface::RobotHW
{
public:
    BarrettHandHardwareInterface(const int & canbus_number_, bool & forcetorque_, bool & tactile_,  std::string & calibration_filename_);
    ~BarrettHandHardwareInterface();
    void readHW();
    void writeHW();
    void readTactile()
    {
        //bus->tactile_get_data
    }

    boost::shared_ptr<OWD::WamDriver> wamdriver;


private:
    static const unsigned int joint_number=5;
    hardware_interface::JointStateInterface jnt_state_interface;

    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;


    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<double> pos_cmd;
    std::vector<double> vel_cmd;
    std::vector<double> eff_cmd;
    std::vector<double> pos_cmd_previous;
    std::vector<double> vel_cmd_previous;
    std::vector<double> eff_cmd_previous;


    void readPosition();

    void writePosition();
    void writeVelocity();
    void writeEffort();



    ros::NodeHandle n;

    boost::shared_ptr<BHD_280> bhd;
    //boost::shared_ptr<FT> ft;
    boost::shared_ptr<Tactile> tact;

    // OWD parameters
    std::string calibration_filename;
    int canbus_number;
    std::string hand_type;
    bool forcetorque;
    int pub_freq;  // DEPRECATED
    int wam_pub_freq;
    int hand_pub_freq;
    int ft_pub_freq;
    int tactile_pub_freq;
    bool tactile;

};


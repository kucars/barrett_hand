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
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define PI_ 3.14159265359
#define DEG_TO_RAD PI/180.0

class BarrettHandHardwareInterface : public hardware_interface::RobotHW
{
public:
    BarrettHandHardwareInterface(const int & canbus_number_, bool & forcetorque_, bool & tactile_,  std::string & calibration_filename_, double & ft_pub_freq, double & tactile_pub_freq);
    BarrettHandHardwareInterface()
    {};

    bool init();

    bool init(hardware_interface::JointStateInterface &jnt_state_interface_,
              hardware_interface::PositionJointInterface &jnt_pos_interface_,
              hardware_interface::VelocityJointInterface &jnt_vel_interface_,
              hardware_interface::EffortJointInterface &jnt_eff_interface_
              );


    ~BarrettHandHardwareInterface();
    void readHW();
    void writeHW();


    boost::shared_ptr<OWD::WamDriver> wamdriver;


private:
    static const unsigned int joint_number=8;
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
    void readTorque();
    void readPositionAndComputeVelocity();

    void writePosition();
    void writeVelocity();
    void writeEffort();

    bool ftPublish(const ros::TimerEvent& e);
    void tactilePublish(const ros::TimerEvent& e);

    ros::NodeHandle n;
    ros::Time prev_time;

    boost::shared_ptr<BHD_280> bhd;
    boost::shared_ptr<Tactile> tact;

    // OWD parameters
    std::string calibration_filename;
    int canbus_number;
    std::string hand_type;
    bool forcetorque;
    int ft_pub_freq_;
    int tactile_pub_freq_;
    bool tactile;

    ros::Publisher pub_tactile;
    ros::Publisher pub_ft;
    ros::Publisher pub_filtered_ft;
    ros::Publisher pub_ft_state;
    ros::Publisher pub_accel;

    ros::Timer tactile_timer;

    owd_msgs::BHTactile tactile_msg;
    owd_msgs::ForceState ft_state;

    geometry_msgs::WrenchStamped ft_vals;
    geometry_msgs::Vector3 accel_vals;

    //sensor_msgs::PointCloud2 finger_1_contacts;
    pcl::PointCloud<pcl::PointXYZI>::Ptr finger_1_cloud;
    ros::Publisher pub_tactile_finger_1_pcl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr finger_2_cloud;
    ros::Publisher pub_tactile_finger_2_pcl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr finger_3_cloud;
    ros::Publisher pub_tactile_finger_3_pcl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr tactile_cloud;
    ros::Publisher pub_tactile_pcl;

};


#include <barrett_hand_hardware_interface/BarrettHandHardwareInterface.h>

bool isEqual(double & a, double & b, double threshold)
{
    return fabs(a-b)<threshold;
}

BarrettHandHardwareInterface::BarrettHandHardwareInterface(const int & canbus_number_, bool & forcetorque_, bool & tactile_,  std::string & calibration_filename_) : n("~"),
    canbus_number(canbus_number_),
    forcetorque(forcetorque_),
    tactile(tactile_),
    calibration_filename(calibration_filename_)
{
    pos.resize(joint_number);
    vel.resize(joint_number);
    eff.resize(joint_number);
    pos_cmd.resize(joint_number);
    vel_cmd.resize(joint_number);
    eff_cmd.resize(joint_number);
    pos_cmd_previous.resize(joint_number);
    vel_cmd_previous.resize(joint_number);
    eff_cmd_previous.resize(joint_number);

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_j1("finger_1_med_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_j1);

    hardware_interface::JointStateHandle state_handle_j2("finger_2_med_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_j2);

    hardware_interface::JointStateHandle state_handle_j3("finger_3_med_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_j3);

    hardware_interface::JointStateHandle state_handle_j4("finger_1_prox_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_j4);

    hardware_interface::JointStateHandle state_handle_j5("finger_2_prox_joint", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_j5);

    hardware_interface::JointStateHandle state_handle_j6("finger_1_dist_joint", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_j6);

    hardware_interface::JointStateHandle state_handle_j7("finger_2_dist_joint", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_j7);

    hardware_interface::JointStateHandle state_handle_j8("finger_3_dist_joint", &pos[7], &vel[7], &eff[7]);
    jnt_state_interface.registerHandle(state_handle_j8);


    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &pos_cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_j1);

    hardware_interface::JointHandle pos_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &pos_cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_j2);

    hardware_interface::JointHandle pos_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &pos_cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_j3);

    hardware_interface::JointHandle pos_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &pos_cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_j4);

    hardware_interface::JointHandle pos_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &pos_cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_j5);



    registerInterface(&jnt_pos_interface);

    // connect and register the velocity interface
    hardware_interface::JointHandle vel_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &vel_cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_j1);

    hardware_interface::JointHandle vel_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &vel_cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_j2);

    hardware_interface::JointHandle vel_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &vel_cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_j3);

    hardware_interface::JointHandle vel_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &vel_cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_j4);

    hardware_interface::JointHandle vel_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &vel_cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_j5);

    registerInterface(&jnt_vel_interface);

    // connect and register the effort interface
    hardware_interface::JointHandle eff_handle_j1(jnt_state_interface.getHandle("finger_1_med_joint"), &eff_cmd[0]);
    jnt_eff_interface.registerHandle(eff_handle_j1);

    hardware_interface::JointHandle eff_handle_j2(jnt_state_interface.getHandle("finger_2_med_joint"), &eff_cmd[1]);
    jnt_eff_interface.registerHandle(eff_handle_j2);

    hardware_interface::JointHandle eff_handle_j3(jnt_state_interface.getHandle("finger_3_med_joint"), &eff_cmd[2]);
    jnt_eff_interface.registerHandle(eff_handle_j3);

    hardware_interface::JointHandle eff_handle_j4(jnt_state_interface.getHandle("finger_1_prox_joint"), &eff_cmd[3]);
    jnt_eff_interface.registerHandle(eff_handle_j4);

    hardware_interface::JointHandle eff_handle_j5(jnt_state_interface.getHandle("finger_2_prox_joint"), &eff_cmd[4]);
    jnt_eff_interface.registerHandle(eff_handle_j5);


    registerInterface(&jnt_eff_interface);


    int BH_MODEL=280;
    wamdriver = boost::shared_ptr<OWD::WamDriver> (new OWD::WamDriver(canbus_number,BH_MODEL,forcetorque,tactile));
    std::cout << "calibration file name: "<<calibration_filename<< std::endl;
    try
    {
        if (! wamdriver->Init(calibration_filename.c_str()))
        {
            ROS_FATAL("WamDriver::Init() returned false; exiting.");
            exit(1);
        }
    }
    catch (int error)
    {
        ROS_FATAL("Error during WamDriver::Init(); exiting.");
        exit(1);
    }
    catch (const char *errmsg)
    {
        ROS_FATAL("Error during WamDriver::Init(): %s",errmsg);
        exit(1);
    }



    bhd=boost::shared_ptr<BHD_280>(new BHD_280(wamdriver->bus));
    tact=boost::shared_ptr<Tactile>(new Tactile(wamdriver->bus));


    //INITIALIZE
    readPosition();
    prev_time=ros::Time::now();

    pos_cmd=pos;
    pos_cmd_previous=pos_cmd;
    std::fill(vel.begin(), vel.end(), 0.0);
    std::fill(eff.begin(), eff.end(), 0.0);


    return;
}

BarrettHandHardwareInterface::~BarrettHandHardwareInterface()
{}

void BarrettHandHardwareInterface::readHW()
{
    ros::Time curr_time=ros::Time::now();
    ros::Duration period = curr_time-prev_time;
    readPositionAndComputeVelocity(period);
    readTorque();
    prev_time=curr_time;
}

void BarrettHandHardwareInterface::writeHW()
{
    writePosition();
    //writeVelocity();
}

void BarrettHandHardwareInterface::readPosition()
{
    wamdriver->bus->hand_get_positions(pos[0],
                            pos[1],
                            pos[2],
                            pos[3]);
    pos[4]=pos[3];

    pos[5]=(pos[0])/2.4+0.6;
    pos[6]=(pos[1])/2.4+0.6;
    pos[7]=(pos[2])/2.4+0.6;

    //wamdriver->bus->hand_get_outer_links(pos[5],pos[6],pos[7]);
}

void BarrettHandHardwareInterface::readPositionAndComputeVelocity(ros::Duration &  period)
{
    std::vector<double> prev_pos=pos;
    readPosition();
    //std::cout << period << std::endl;
    for(int i=0;i<joint_number;++i)
    {
        vel[i]=(pos[i]-prev_pos[i])/period.toSec();
    }

}
void BarrettHandHardwareInterface::readTorque()
{
    //std::cout << "read torque:" << std::endl;

    //static double ft_values[6];
    //static double ft_filtered_values[6];
    //int ft_get_status;
    //ft_get_status = wamdriver->bus->ft_get_data(ft_values,ft_filtered_values);
    //std::cout << "ft_get_status:" << ft_get_status<< std::endl;
    //std::cout << "ft_values:"<<ft_values[0]<< std::endl;

    wamdriver->bus->hand_get_strain(eff[0], eff[1], eff[2]);
}

void BarrettHandHardwareInterface::writePosition()
{
    bool new_command=false;
    for(int i=0; i<joint_number;++i)
    {
        if(!isEqual(pos_cmd_previous[i],pos_cmd[i],0.00001))
        {
                new_command=true;
                break;
        }
    }
    if(!new_command)
    {
        //pos_cmd_previous=pos_cmd;
        return;
    }

    std::vector<double> write_cmd;
    write_cmd.push_back(pos_cmd[0]);
    write_cmd.push_back(pos_cmd[1]);
    write_cmd.push_back(pos_cmd[2]);
    write_cmd.push_back(pos_cmd[3]);
    wamdriver->bus->hand_move(write_cmd);
    pos_cmd_previous=pos_cmd;
}

void BarrettHandHardwareInterface::writeVelocity()
{
    bool new_command=false;
    for(int i=0; i<joint_number;++i)
    {
        if(!isEqual(vel_cmd_previous[i],vel_cmd[i],0.00001))
        {
                new_command=true;
                break;
        }
    }
    if(!new_command)
    {
//        vel_cmd_previous=vel_cmd;
        return;
    }

    std::vector<double> write_cmd;
    write_cmd.push_back(vel_cmd[0]);
    write_cmd.push_back(vel_cmd[1]);
    write_cmd.push_back(vel_cmd[2]);
    write_cmd.push_back(vel_cmd[3]);
    wamdriver->bus->hand_velocity(write_cmd);
    vel_cmd_previous=vel_cmd;
}

void BarrettHandHardwareInterface::writeEffort()
{
    std::vector<double> write_cmd;
    write_cmd.push_back(eff_cmd[0]);
    write_cmd.push_back(eff_cmd[1]);
    write_cmd.push_back(eff_cmd[2]);
    write_cmd.push_back(eff_cmd[3]);
    wamdriver->bus->hand_torque(write_cmd);
}





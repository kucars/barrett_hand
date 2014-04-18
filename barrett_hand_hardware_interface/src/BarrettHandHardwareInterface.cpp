#include <barrett_hand_hardware_interface/BarrettHandHardwareInterface.h>

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
    /*#ifndef BH280_ONLY
     try {
       wamdriver->AdvertiseAndSubscribe(n);
     } catch (int error) {
       ROS_FATAL("Error during WamDriver::AdvertiseAndSubscribe(); exiting.");
       delete wamdriver;
       exit(1);
     }
   #endif // BH280_ONLY*/


    bhd=boost::shared_ptr<BHD_280>(new BHD_280(wamdriver->bus));


//    /boost::shared_ptr<FT> ft(new FT(wamdriver->bus));
    tact=boost::shared_ptr<Tactile>(new Tactile(wamdriver->bus));



    //#ifndef OWDSIM
    /*ros::Timer bhd_timer;
    if (bhd)
    {
        bhd_timer = n.createTimer(ros::Rate(hand_pub_freq).expectedCycleTime(), &BHD_280::Pump, bhd);
    }
    ros::Timer ft_timer;
    if (ft)
    {
        ft_timer = n.createTimer(ros::Rate(ft_pub_freq).expectedCycleTime(), &FT::Pump, ft);
    }
    ros::Timer tactile_timer;
    if (tact)
    {
        tactile_timer = n.createTimer(ros::Rate(tactile_pub_freq).expectedCycleTime(), &Tactile::Pump, tact);
    }*/
    //#endif // OWDSIM

    /*ros::MultiThreadedSpinner s(3);
    ROS_DEBUG("Spinning");
    ros::spin(s);
    ROS_DEBUG("Done spinning; exiting");
#ifndef OWDSIM
    while (wamdriver->owam->bus)
    {
        usleep(10000);
    }
#endif*/

    //INITIALIZE

    readHW();

    pos_cmd=pos;

    pos_cmd_previous=pos_cmd;

    return;
}

BarrettHandHardwareInterface::~BarrettHandHardwareInterface()
{}

void BarrettHandHardwareInterface::readHW()
{
    readPosition();
}

void BarrettHandHardwareInterface::writeHW()
{
    writePosition();

}

void BarrettHandHardwareInterface::readPosition()
{
    wamdriver->bus->hand_get_positions(pos[0],
                            pos[1],
                            pos[2],
                            pos[3]);
    pos[4]=pos[3];
    //std::cout << pos[3] << std::endl;
}

void BarrettHandHardwareInterface::writePosition()
{
    if(isEqual(pos_cmd_previous[0],pos_cmd[0],0.00001)&&
       isEqual(pos_cmd_previous[1],pos_cmd[1],0.00001)&&
       isEqual(pos_cmd_previous[2],pos_cmd[2],0.00001)&&
       isEqual(pos_cmd_previous[3],pos_cmd[3],0.00001))
    {
        pos_cmd_previous=pos_cmd;
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
    std::vector<double> write_cmd;
    write_cmd.push_back(vel_cmd[0]);
    write_cmd.push_back(vel_cmd[1]);
    write_cmd.push_back(vel_cmd[2]);
    write_cmd.push_back(vel_cmd[3]);
    wamdriver->bus->hand_velocity(write_cmd);
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





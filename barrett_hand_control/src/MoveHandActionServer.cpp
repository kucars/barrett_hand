#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <barrett_hand_msgs/MoveHandAction.h>

class MoveHandAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<barrett_hand_msgs::MoveHandAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  barrett_hand_msgs::MoveHandFeedback feedback_;
  barrett_hand_msgs::MoveHandResult result_;

public:

  MoveHandAction(std::string name) :
    as_(nh_, name, boost::bind(&MoveHandAction::executeCB, this, _1), false),
    action_name_(name)
  {

  	ros::Timer bhd_timer;
  	if (bhd) {
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
  	}

    as_.start();
  }

  ~MoveHandAction(void)
  {
  }

  void executeCB(const barrett_hand_msgs::MoveHandGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the MoveHand sequence
    //feedback_.sequence.clear();
    //feedback_.sequence.push_back(0);
    //feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating MoveHand sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->positions.size(); i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveHand");

  MoveHandAction MoveHand(ros::this_node::getName());
  ros::spin();

  return 0;
}

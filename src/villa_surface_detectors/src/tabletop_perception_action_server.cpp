#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <villa_surface_detectors/PerceiveTabletopSceneAction.h>

class PerceiveTabletopSceneAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<villa_surface_detectors::PerceiveTabletopSceneAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  villa_surface_detectors::PerceiveTabletopSceneFeedback feedback_;
  villa_surface_detectors::PerceiveTabletopSceneResult result_;

public:

  PerceiveTabletopSceneAction(std::string name) :
    as_(nh_, name, boost::bind(&PerceiveTabletopSceneAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PerceiveTabletopSceneAction(void)
  {
  }

  void executeCB(const villa_surface_detectors::PerceiveTabletopSceneGoalConstPtr &goal)
  {
    ROS_INFO("Hello Friends");
    bool success = true;
    as_.setSucceeded(result_);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "perceive_tabletop");

  PerceiveTabletopSceneAction perceive_tabletop("perceive_tabletop");
  ROS_INFO("server online");
  ros::spin();

  return 0;
}

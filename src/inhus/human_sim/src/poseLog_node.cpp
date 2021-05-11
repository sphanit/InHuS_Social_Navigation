#include "poseLog.h"

//////////////////// POSE LOG ////////////////////////////

// subscribes to Agent positions and sends them to the logManager to be saved

PoseLog::PoseLog()
{
	path_ = ros::package::getPath("human_sim");
	log_file_.open(path_ + "/logs/log_data/poseLog.txt");
	log_file_ << "LOG STARTS : " << ros::Time::now() << endl;

	sub_pose_H_ = nh_.subscribe("interface/in/human_pose", 100, &PoseLog::poseHCallback, this);
	sub_pose_R_ = nh_.subscribe("interface/in/robot_pose", 100, &PoseLog::poseRCallback, this);
}

PoseLog::~PoseLog()
{
	log_file_.close();
}

void PoseLog::poseHCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : H " << std::to_string(ros::Time::now().toSec()) << " " << msg->x << " " << msg->y << endl;
}

void PoseLog::poseRCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : R " << std::to_string(ros::Time::now().toSec()) << " " << msg->x << " " << msg->y << endl;
}

//////////////////////// MAIN ///////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "logManager");

	PoseLog log_manager;

	ros::spin();

	return 0;
}

/////////////////////////////////////////////////////////////
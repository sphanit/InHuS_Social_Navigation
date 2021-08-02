#include "logManagerCohan.h"
#include <string>
#include <cmath>
#include <cstdint>

void save_image(const ::std::string &name, int img_vals[][150])
{
   using ::std::string;
   using ::std::ios;
   using ::std::ofstream;
   typedef unsigned char pixval_t;
   auto int_to_pixval = [](int img_val) -> pixval_t {
      int tmpval = static_cast<int>(::std::floor(img_val));
      if (tmpval < 0) {
         return 0u;
      } else if (tmpval > 255) {
         return 255u;
      } else {
         return tmpval & 0xffu;
      }
   };
   auto as_pgm = [](const string &name) -> string {
      if (! ((name.length() >= 4)
             && (name.substr(name.length() - 4, 4) == ".pgm")))
      {
         return name + ".pgm";
      } else {
         return name;
      }
   };

   ofstream out(as_pgm(name), ios::binary | ios::out | ios::trunc);

   out << "P5\n150 150\n255\n";
   for (int x = 0; x < 150; ++x) {
      for (int y = 0; y < 150; ++y) {
         const pixval_t pixval = int_to_pixval(img_vals[x][y]);
         const char outpv = static_cast<const char>(pixval);
         out.write(&outpv, 1);
      }
   }
}


//////////////////// LOG MANAGER ////////////////////////////

LogManagerCohan::LogManagerCohan()
{
	path_ = ros::package::getPath("inhus");

	id_ = 0;

	sub_odom_r_ = nh_.subscribe("/robot_odom", 100, &LogManagerCohan::odomRCB, this);
	sub_odom_h_ = nh_.subscribe("/human_odom", 100, &LogManagerCohan::odomHCB, this);
	sub_robot_goal_ = nh_.subscribe("/robot_goal", 100, &LogManagerCohan::robotGoalCB, this);
	sub_robot_goal_status_ = nh_.subscribe("/robot_goal_status", 100, &LogManagerCohan::robotGoalStatusCB, this);
  sub_costmap_r_ = nh_.subscribe("/move_base/local_costmap/costmap", 100, &LogManagerCohan::costmapCB, this);

	goal_received_ = false;
	goal_done_ = true;
}

LogManagerCohan::~LogManagerCohan()
{
	log_file_r_.close();
	log_file_h_.close();
}

void LogManagerCohan::odomRCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_r_ = *msg;
}

void LogManagerCohan::odomHCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_h_ = *msg;
}

void LogManagerCohan::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	if(goal_received_ && !goal_done_){
		log_file_r_ << "pose: " << odom_r_.pose.pose.position.x << ", " << odom_r_.pose.pose.position.y << ", " << odom_r_.pose.pose.orientation.z << ", " << odom_r_.pose.pose.orientation.w << ", vel: " << odom_r_.twist.twist.linear.x << ", " << odom_r_.twist.twist.linear.y << ", " << odom_r_.twist.twist.angular.z << endl;
		log_file_h_ << "pose: " << odom_h_.pose.pose.position.x << ", " << odom_h_.pose.pose.position.y << ", " << odom_h_.pose.pose.orientation.z << ", " << odom_h_.pose.pose.orientation.w << ", vel: " << odom_h_.twist.twist.linear.x << ", " << odom_h_.twist.twist.linear.y << ", " << odom_h_.twist.twist.angular.z << endl;
		auto map_data = msg->data;
		int cmap[150][150];
		for(int i = 0; i < map_data.size(); ++i) {
			cmap[i / 150][i % 150] = (int) map_data[i];
		}
		string cmap_name = path_ + "/logs/cohan_logs/map_data/lcmap_" + std::to_string(id_)+ "_"+ std::to_string(map_id_);
		save_image(cmap_name,cmap);
		map_id_++;
	}
}


void LogManagerCohan::robotGoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!goal_done_)
	{
		log_file_r_.close();
		log_file_h_.close();
		id_++;
	}
  map_id_ = 0;

	log_file_r_.open(path_ + "/logs/cohan_logs/log_cohan_" + std::to_string(id_) + "_r.txt");
	log_file_r_ << "start: x= " << odom_r_.pose.pose.position.x << ", y= " << odom_r_.pose.pose.position.y << ", z= " << odom_r_.pose.pose.orientation.z << ", w= " << odom_r_.pose.pose.orientation.w << endl;
	log_file_r_ << "goal: x= " << msg->pose.position.x << ", y= " << msg->pose.position.y << ", z= " << msg->pose.position.z << ", w= " << msg->pose.orientation.w << endl;

	log_file_h_.open(path_ + "/logs/cohan_logs/log_cohan_" + std::to_string(id_) + "_h.txt");
	log_file_h_ << "start: x= " << odom_h_.pose.pose.position.x << ", y= " << odom_h_.pose.pose.position.y << ", z= " << odom_h_.pose.pose.orientation.z << ", w= " << odom_h_.pose.pose.orientation.w << endl;

	goal_received_ = true;
	goal_done_ = false;
}

void LogManagerCohan::robotGoalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	if(!msg->status_list.empty())
	{
		if(!goal_done_
		&& ((msg->status_list.back().status == 2) 	// PREEMPTED
		|| (msg->status_list.back().status == 3) 		// SUCCEEDED
		|| (msg->status_list.back().status == 4))) 	// ABORTED
		{
			log_file_r_.close();
			log_file_h_.close();
			goal_done_ = true;
			goal_received_ = false;
			id_++;
		}
	}
}

//////////////////////// MAIN ///////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LogManagerCohan");

	LogManagerCohan log_cohan_manager;

	ros::spin();

	return 0;
}

/////////////////////////////////////////////////////////////

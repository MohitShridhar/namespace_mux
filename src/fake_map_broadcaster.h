#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>

#define MUX_CONTROL_TOPIC "/rviz_mux/control"

class FakeMapBroadcaster
{
	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber control_sub;

		std::string map_name, fake_frame_id;
		std::vector<std::string> active_bots;
		int pub_freq;

		std::string currRobotNs;

		tf::TransformBroadcaster broadcaster;
		tf::Transform emptyTransform;

		void parseParams();
		void createEmptyTransform();
		void broadcastFakeMap();
		void setupController();
		void mux_control_callback(const std_msgs::String::ConstPtr& msg);

	public:
		FakeMapBroadcaster();
		~FakeMapBroadcaster();

		void manageBroadcast();
		void setCurrRobotNs(std::string newNamespace);
};
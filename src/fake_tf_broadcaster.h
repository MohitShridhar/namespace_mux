#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class FakeTfBroadcaster
{
	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber control_sub;

		std::vector<std::string> input_frame_ids, fake_ids;
		std::vector<int> tf_offset;
		int pub_freq;

		tf::TransformBroadcaster broadcaster;
		tf::TransformListener listener;
		tf::Transform constantTransform;

		void parseParams();
		void createTransform();
		void broadcastFakeTf();

	public:
		FakeTfBroadcaster();
		~FakeTfBroadcaster();

		void manageBroadcast();
};
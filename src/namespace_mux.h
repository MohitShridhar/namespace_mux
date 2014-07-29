#include "dynamic_topic_relay.h"

#include <topic_tools/shape_shifter.h>
#include <topic_tools/parse.h>

#include <std_msgs/String.h>

#define MUX_CONTROL_TOPIC "/rviz_mux/control"

class DynamicTopicRelay;

class NamespaceMux
{
	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber control_sub;
		std::string robot_namespace_ref, rviz_namespace;
		
		std::vector<std::string> active_bots, subscribed_topics, published_topics;
		std::vector<DynamicTopicRelay*> relayedSubs, relayedPubs;
		std::vector<ros::Subscriber> inputSubscriptions, inputPublications;

		std::string currRobotNs;

		void parseParams();
		void mux_control_callback(const std_msgs::String::ConstPtr& msg);
		void setupController();

	public:
		NamespaceMux();
		~NamespaceMux();

		void manageTopics();
		
		void setupMuxers();
		void setupSubscribers();
		void setupPublishers();

		void switchMuxes();
		void switchSubscribers();
		void switchPublishers();

};
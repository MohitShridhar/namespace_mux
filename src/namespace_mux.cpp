/* 
* @Author: MohitSridhar
*/

#include "namespace_mux.h"

NamespaceMux::NamespaceMux()
{
	parseParams();
	setupMuxers();
	setupController();
}

NamespaceMux::~NamespaceMux()
{
	rosNode->shutdown();
	delete rosNode;
}

void NamespaceMux::setupController()
{
	// Setup mux controller:
	control_sub = rosNode->subscribe<std_msgs::String>(MUX_CONTROL_TOPIC, 2, &NamespaceMux::mux_control_callback, this);
	
}

void NamespaceMux::parseParams()
{
	rosNode = new ros::NodeHandle("");	

	rosNode->getParam("/rviz_mux/robot_namespace_ref", robot_namespace_ref);
	ROS_INFO("PARAM: robot_namespace_ref - %s\n", robot_namespace_ref.c_str());

	rosNode->getParam("/rviz_mux/rviz_namespace", rviz_namespace);
	ROS_INFO("PARAM: rviz_namespace - %s\n", rviz_namespace.c_str());

	rosNode->getParam("/rviz_mux/active_bots", active_bots);
	rosNode->getParam("/rviz_mux/subscribed_topics", subscribed_topics);
	rosNode->getParam("/rviz_mux/published_topics", published_topics);

	if (active_bots.empty()) {
		ROS_ERROR("No bots are active. Dynamic Topic relay disabled\n");
		std::exit(EXIT_FAILURE);
	} 

	// Default namespace: first bot
	else {
		currRobotNs = active_bots.begin()->c_str();
		ROS_INFO("Current Robot Namespace: %s\n", currRobotNs.c_str());
	}
}

void NamespaceMux::setupMuxers()
{
	setupSubscribers();
	setupPublishers();
}

void NamespaceMux::setupSubscribers()
{
	for (std::vector<std::string>::iterator it = subscribed_topics.begin(); it != subscribed_topics.end(); ++it) {
		std::string mainTopic = (*it).c_str();
		std::string inputTopic = "/" + currRobotNs + mainTopic;
		std::string outputTopic = "/" + rviz_namespace + mainTopic;
		
		DynamicTopicRelay* relayedTopic = new DynamicTopicRelay(rosNode, mainTopic, inputTopic, outputTopic); 
		ros::Subscriber inputSubscription = rosNode->subscribe<topic_tools::ShapeShifter>(relayedTopic->getInputTopic(), 10, &DynamicTopicRelay::in_cb, relayedTopic);
		
		inputSubscriptions.push_back(inputSubscription);
		relayedSubs.push_back(relayedTopic);
	}
}

void NamespaceMux::setupPublishers()
{
	for (std::vector<std::string>::iterator it = published_topics.begin(); it != published_topics.end(); ++it) {
		std::string mainTopic = (*it).c_str();
		std::string inputTopic = "/" + rviz_namespace + mainTopic;
		std::string outputTopic = "/" + currRobotNs + mainTopic;

		DynamicTopicRelay* relayedTopic = new DynamicTopicRelay(rosNode, mainTopic, inputTopic, outputTopic);
		ros::Subscriber inputPublication = rosNode->subscribe<topic_tools::ShapeShifter>(relayedTopic->getInputTopic(), 10, &DynamicTopicRelay::in_cb, relayedTopic);

		inputPublications.push_back(inputPublication);
		relayedPubs.push_back(relayedTopic);
	}
}

void NamespaceMux::mux_control_callback(const std_msgs::String::ConstPtr& cmd)
{
	std::string newBotNs = cmd->data;

	// Ensure that robot namespace exists:
	if (std::find(active_bots.begin(), active_bots.end(), newBotNs) != active_bots.end()) {
		currRobotNs = newBotNs;
		switchMuxes();
		ROS_INFO("Current namespace: %s\n", currRobotNs.c_str());
	} else {
		ROS_ERROR("The namespace '%s' does exist. Check that it was properly initialized in your launch file\n", newBotNs.c_str());
	}
}

void NamespaceMux::manageTopics()
{
	ros::spin();
}

void NamespaceMux::switchMuxes()
{
	switchSubscribers();
	switchPublishers();
}

void NamespaceMux::switchPublishers()
{	
	for (std::vector<DynamicTopicRelay*>::iterator it = relayedPubs.begin(); it != relayedPubs.end(); ++it) {
		DynamicTopicRelay* relayedTopic = *it;
		relayedTopic->setOutputTopic("/" + currRobotNs + relayedTopic->getMainTopic());
	}	
}

void NamespaceMux::switchSubscribers()
{
	for (std::vector<ros::Subscriber>::iterator it = inputSubscriptions.begin(); it != inputSubscriptions.end(); ++it) {
		(*it).shutdown();
	}

	inputSubscriptions.clear();
	setupSubscribers();
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "rviz_namespace_mux");

	NamespaceMux nsMux;
	nsMux.manageTopics();

	return 0;
}

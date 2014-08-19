#include "dynamic_topic_relay.h"

DynamicTopicRelay::DynamicTopicRelay(ros::NodeHandle* rosNode, std::string mainTopic, std::string inputTopic, std::string outputTopic)
{	
	g_node = rosNode;
	g_advertised = false;

	g_main_topic = mainTopic;
	g_output_topic = outputTopic;
	g_input_topic = inputTopic;
}

DynamicTopicRelay::~DynamicTopicRelay()
{
	g_node->shutdown();
	delete g_node;	
}

void DynamicTopicRelay::in_cb(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
{
  if (!g_advertised)
  {
    g_pub = msg->advertise(*g_node, g_output_topic, 100);
    g_advertised = true;
    ROS_INFO("%s advertised as %s\n", g_input_topic.c_str(), g_output_topic.c_str());
  }

  g_pub.publish(msg);
}

std::string DynamicTopicRelay::getInputTopic()
{
	return g_input_topic;
}

std::string DynamicTopicRelay::getOutputTopic()
{
	return g_output_topic;
}

std::string DynamicTopicRelay::getMainTopic()
{
	return g_main_topic;
}

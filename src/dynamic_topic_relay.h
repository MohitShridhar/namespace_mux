#include <cstdio>
#include <topic_tools/shape_shifter.h>
#include <topic_tools/parse.h>

#include <ros/callback_queue.h>

class DynamicTopicRelay
{
	private:

		ros::NodeHandle *g_node; //= NULL;

		bool g_advertised; //= false;
		std::string g_output_topic, g_input_topic, g_main_topic;
		ros::Publisher g_pub;

	public:
		DynamicTopicRelay(ros::NodeHandle* rosNode, std::string mainTopic, std::string inputTopic, std::string outputTopic);
		~DynamicTopicRelay();
		void setOutputTopic(std::string newOutputTopic);
		
		std::string getInputTopic();
		std::string getOutputTopic();
		std::string getMainTopic();

	public:
		void in_cb(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
};
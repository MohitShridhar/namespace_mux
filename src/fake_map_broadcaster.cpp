/* 
* @Author: MohitSridhar
*/

#include "fake_map_broadcaster.h"

FakeMapBroadcaster::FakeMapBroadcaster()
{
	parseParams();
	setupController();
	createEmptyTransform();
}

FakeMapBroadcaster::~FakeMapBroadcaster()
{
	rosNode->shutdown();
	delete rosNode;
}

void FakeMapBroadcaster::setupController()
{
	control_sub = rosNode->subscribe<std_msgs::String>(MUX_CONTROL_TOPIC, 2, &FakeMapBroadcaster::mux_control_callback, this);
}

void FakeMapBroadcaster::createEmptyTransform()
{
	tf::Quaternion zeroQuat;
	zeroQuat.setRPY(0.0, 0.0, 0.0);

	emptyTransform = tf::Transform(zeroQuat, tf::Vector3(0.0, 0.0, 0.0));
}

void FakeMapBroadcaster::mux_control_callback(const std_msgs::String::ConstPtr& cmd)
{
	setCurrRobotNs(cmd->data);
}

void FakeMapBroadcaster::parseParams()
{
	rosNode = new ros::NodeHandle("");

	rosNode->getParam("/fake_map_broadcaster/map_name", map_name);
	rosNode->getParam("/fake_map_broadcaster/fake_frame_id", fake_frame_id);
	rosNode->getParam("/rviz_mux/active_bots", active_bots);
	rosNode->getParam("/fake_map_broadcaster/pub_freq", pub_freq);

	if (active_bots.empty()) {
		ROS_ERROR("No bots are active. Fake map broadcaster disabled\n");
		std::exit(EXIT_FAILURE);
	}

	else {
		currRobotNs = active_bots.begin()->c_str();
		ROS_INFO("Current Robot Namespace: %s\n", currRobotNs.c_str());
	}
}

void FakeMapBroadcaster::manageBroadcast()
{
	ros::Rate r(pub_freq);

	while(rosNode->ok()) {

		broadcastFakeMap();

		ros::spinOnce();
		r.sleep();

	}
}

void FakeMapBroadcaster::broadcastFakeMap()
{
	std::string refFrame = "/" + currRobotNs + "/" + map_name;
	std::string targetFrame = "/" + fake_frame_id + "/" + map_name;

	broadcaster.sendTransform(tf::StampedTransform(emptyTransform, ros::Time::now(), refFrame, targetFrame));
}

void FakeMapBroadcaster::setCurrRobotNs(std::string newNamespace)
{
	// Ensure that robot namespace exists:
	if (std::find(active_bots.begin(), active_bots.end(), newNamespace) != active_bots.end()) {
		currRobotNs = newNamespace;
		ROS_INFO("Current namespace: %s\n", currRobotNs.c_str());
	} else {
		ROS_ERROR("The namespace '%s' does exist. Check that it was properly initialized in your launch file\n", currRobotNs.c_str());
	}
}


int main(int argc, char** argv){
    
	ros::init(argc, argv, "fake_map_broadcaster");

	FakeMapBroadcaster fakeBc;
	fakeBc.manageBroadcast();	

    return 0;
}
/* 
* @Author: MohitSridhar
*/

#include "fake_tf_broadcaster.h"

FakeTfBroadcaster::FakeTfBroadcaster()
{
	parseParams();
	createTransform();
}

FakeTfBroadcaster::~FakeTfBroadcaster()
{
	rosNode->shutdown();
	delete rosNode;
}

void FakeTfBroadcaster::createTransform()
{	
	float x_off = tf_offset.at(0);
	float y_off = tf_offset.at(1);
	float z_off = tf_offset.at(2);

	float roll_off = tf_offset.at(3);
	float pitch_off = tf_offset.at(4);
	float yaw_off = tf_offset.at(5);

	tf::Quaternion quat;
	quat.setRPY(roll_off, pitch_off, yaw_off);

	constantTransform = tf::Transform(quat, tf::Vector3(x_off, y_off, z_off));
}

void FakeTfBroadcaster::parseParams()
{
	rosNode = new ros::NodeHandle("");

	rosNode->getParam("/fake_tf_broadcaster/pub_freq", pub_freq);
	rosNode->getParam("/fake_tf_broadcaster/input_curr_frame_ids", input_frame_ids);
	rosNode->getParam("/fake_tf_broadcaster/output_fake_frame_ids", fake_ids);
	rosNode->getParam("/fake_tf_broadcaster/output_tf_offset", tf_offset);
}

void FakeTfBroadcaster::manageBroadcast()
{
	ros::Rate r(pub_freq);

	while(rosNode->ok()) {

		broadcastFakeTf();

		ros::spinOnce();
		r.sleep();

	}
}

void FakeTfBroadcaster::broadcastFakeTf()
{

	std::string firstTargetFrame = fake_ids.at(0);

	for (std::vector<std::string>::iterator itBranch = input_frame_ids.begin(); itBranch != input_frame_ids.end(); ++itBranch) {
		std::string refFrame = *itBranch;
		broadcaster.sendTransform(tf::StampedTransform(constantTransform, ros::Time::now(), firstTargetFrame, refFrame));
	}

	std::string lastTargetFrame = firstTargetFrame;

	for (std::vector<std::string>::iterator itLink = fake_ids.begin()+1 ; itLink != fake_ids.end(); ++itLink) {
		std::string refFrame = *itLink;
		broadcaster.sendTransform(tf::StampedTransform(constantTransform, ros::Time::now(), refFrame, lastTargetFrame));
		lastTargetFrame = refFrame;
	}

}


int main(int argc, char** argv){
    
	ros::init(argc, argv, "fake_tf_broadcaster");

	FakeTfBroadcaster fakeBc;
	fakeBc.manageBroadcast();	

    return 0;
}
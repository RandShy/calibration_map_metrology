#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <marvelmind_nav/hedge_pos.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

const int NB_POINTS_TO_AVERAGE = 10;

std::string odomFrame;
std::string mapFrame;
std::string mobileHedgeFrame;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;

std::atomic_bool setFirstHedgePosition(false);
std::atomic_bool setSecondHedgePosition(false);
std::atomic_bool setThirdHedgePosition(false);
std::vector<marvelmind_nav::hedge_pos> hedgePointsRecorded;
marvelmind_nav::hedge_pos firstHedgeMsg;
marvelmind_nav::hedge_pos secondHedgeMsg;
marvelmind_nav::hedge_pos thirdHedgeMsg;

std::atomic_bool setFirstMapperPosition(false);
std::atomic_bool setSecondMapperPosition(false);
std::atomic_bool setThirdMapperPosition(false);
std::vector<geometry_msgs::Point> mapperPointsRecorded;
geometry_msgs::Point firstMobileHedgePositionInMapFrame;
geometry_msgs::Point secondMobileHedgePositionInMapFrame;
geometry_msgs::Point thirdMobileHedgePositionInMapFrame;
geometry_msgs::TransformStamped lastMapToOdomTf;

void hedgeCallback(const marvelmind_nav::hedge_pos& msg)
{
	if(setFirstHedgePosition.load())
	{
		hedgePointsRecorded.push_back(msg);

        if(hedgePointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
        {
            firstHedgeMsg.x_m = 0.0;
            firstHedgeMsg.y_m = 0.0;
            firstHedgeMsg.z_m = 0.0;
            for(int i = 0; i < hedgePointsRecorded.size(); i++)
            {
                firstHedgeMsg.x_m += hedgePointsRecorded[i].x_m;
                firstHedgeMsg.y_m += hedgePointsRecorded[i].y_m;
                firstHedgeMsg.z_m += hedgePointsRecorded[i].z_m;
            }
            firstHedgeMsg.x_m /= hedgePointsRecorded.size();
            firstHedgeMsg.y_m /= hedgePointsRecorded.size();
            firstHedgeMsg.z_m /= hedgePointsRecorded.size();

            hedgePointsRecorded.clear();
            setFirstHedgePosition.store(false);
        }
	}
	if(setSecondHedgePosition.load())
	{
		hedgePointsRecorded.push_back(msg);

        if(hedgePointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
        {
            secondHedgeMsg.x_m = 0.0;
            secondHedgeMsg.y_m = 0.0;
            secondHedgeMsg.z_m = 0.0;
            for(int i = 0; i < hedgePointsRecorded.size(); i++)
            {
                secondHedgeMsg.x_m += hedgePointsRecorded[i].x_m;
                secondHedgeMsg.y_m += hedgePointsRecorded[i].y_m;
                secondHedgeMsg.z_m += hedgePointsRecorded[i].z_m;
            }
            secondHedgeMsg.x_m /= hedgePointsRecorded.size();
            secondHedgeMsg.y_m /= hedgePointsRecorded.size();
            secondHedgeMsg.z_m /= hedgePointsRecorded.size();

            hedgePointsRecorded.clear();
            setSecondHedgePosition.store(false);
		}
	}
	if(setThirdHedgePosition.load())
	{
		hedgePointsRecorded.push_back(msg);

        if(hedgePointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
        {
            thirdHedgeMsg.x_m = 0.0;
            thirdHedgeMsg.y_m = 0.0;
            thirdHedgeMsg.z_m = 0.0;
            for(int i = 0; i < hedgePointsRecorded.size(); i++)
            {
                thirdHedgeMsg.x_m += hedgePointsRecorded[i].x_m;
                thirdHedgeMsg.y_m += hedgePointsRecorded[i].y_m;
                thirdHedgeMsg.z_m += hedgePointsRecorded[i].z_m;
            }
            thirdHedgeMsg.x_m /= hedgePointsRecorded.size();
            thirdHedgeMsg.y_m /= hedgePointsRecorded.size();
            thirdHedgeMsg.z_m /= hedgePointsRecorded.size();

            hedgePointsRecorded.clear();
            setThirdHedgePosition.store(false);
        }
	}
}

void mapperCallback(const nav_msgs::Odometry& msg)
{
	geometry_msgs::TransformStamped mobileHedgeToRobotTf = tfBuffer->lookupTransform(msg.child_frame_id, mobileHedgeFrame, msg.header.stamp, ros::Duration(0.1));
	geometry_msgs::Point mobileHedgePositionInRobotFrame;
	mobileHedgePositionInRobotFrame.x = mobileHedgeToRobotTf.transform.translation.x;
	mobileHedgePositionInRobotFrame.y = mobileHedgeToRobotTf.transform.translation.y;
	mobileHedgePositionInRobotFrame.z = mobileHedgeToRobotTf.transform.translation.z;
	geometry_msgs::Point mobileHedgePositionInMapFrame;
	geometry_msgs::TransformStamped robotToMapTf;
	robotToMapTf.transform.translation.x = msg.pose.pose.position.x;
	robotToMapTf.transform.translation.y = msg.pose.pose.position.y;
	robotToMapTf.transform.translation.z = msg.pose.pose.position.z;
	robotToMapTf.transform.rotation = msg.pose.pose.orientation;
	tf2::doTransform(mobileHedgePositionInRobotFrame, mobileHedgePositionInMapFrame, robotToMapTf);
	
	lastMapToOdomTf = tfBuffer->lookupTransform(odomFrame, mapFrame, msg.header.stamp, ros::Duration(0.1));

	if(setFirstMapperPosition.load())
	{
		mapperPointsRecorded.push_back(mobileHedgePositionInMapFrame);

		if(mapperPointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
		{
			firstMobileHedgePositionInMapFrame.x = 0.0;
			firstMobileHedgePositionInMapFrame.y = 0.0;
			firstMobileHedgePositionInMapFrame.z = 0.0;
			for(int i = 0; i < mapperPointsRecorded.size(); i++)
			{
				firstMobileHedgePositionInMapFrame.x += mapperPointsRecorded[i].x;
				firstMobileHedgePositionInMapFrame.y += mapperPointsRecorded[i].y;
				firstMobileHedgePositionInMapFrame.z += mapperPointsRecorded[i].z;
			}
			firstMobileHedgePositionInMapFrame.x /= mapperPointsRecorded.size();
			firstMobileHedgePositionInMapFrame.y /= mapperPointsRecorded.size();
			firstMobileHedgePositionInMapFrame.z /= mapperPointsRecorded.size();

			mapperPointsRecorded.clear();
			setFirstMapperPosition.store(false);
		}
	}
	if(setSecondMapperPosition.load())
    {
		mapperPointsRecorded.push_back(mobileHedgePositionInMapFrame);

        if(mapperPointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
        {
            secondMobileHedgePositionInMapFrame.x = 0.0;
            secondMobileHedgePositionInMapFrame.y = 0.0;
            secondMobileHedgePositionInMapFrame.z = 0.0;
            for(int i = 0; i < mapperPointsRecorded.size(); i++)
            {
                secondMobileHedgePositionInMapFrame.x += mapperPointsRecorded[i].x;
                secondMobileHedgePositionInMapFrame.y += mapperPointsRecorded[i].y;
                secondMobileHedgePositionInMapFrame.z += mapperPointsRecorded[i].z;
            }
            secondMobileHedgePositionInMapFrame.x /= mapperPointsRecorded.size();
            secondMobileHedgePositionInMapFrame.y /= mapperPointsRecorded.size();
            secondMobileHedgePositionInMapFrame.z /= mapperPointsRecorded.size();

            mapperPointsRecorded.clear();
            setSecondMapperPosition.store(false);
        }
	}
	if(setThirdMapperPosition.load())
	{
		mapperPointsRecorded.push_back(mobileHedgePositionInMapFrame);

        if(mapperPointsRecorded.size() >= NB_POINTS_TO_AVERAGE)
        {
            thirdMobileHedgePositionInMapFrame.x = 0.0;
            thirdMobileHedgePositionInMapFrame.y = 0.0;
            thirdMobileHedgePositionInMapFrame.z = 0.0;
            for(int i = 0; i < mapperPointsRecorded.size(); i++)
            {
                thirdMobileHedgePositionInMapFrame.x += mapperPointsRecorded[i].x;
                thirdMobileHedgePositionInMapFrame.y += mapperPointsRecorded[i].y;
                thirdMobileHedgePositionInMapFrame.z += mapperPointsRecorded[i].z;
            }
            thirdMobileHedgePositionInMapFrame.x /= mapperPointsRecorded.size();
            thirdMobileHedgePositionInMapFrame.y /= mapperPointsRecorded.size();
            thirdMobileHedgePositionInMapFrame.z /= mapperPointsRecorded.size();

            mapperPointsRecorded.clear();
            setThirdMapperPosition.store(false);
        }
	}
}

bool setFirstPositionCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Setting first position...");
	setFirstMapperPosition.store(true);
	setFirstHedgePosition.store(true);
	return true;
}

bool setSecondPositionCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Setting second position...");
	setSecondMapperPosition.store(true);
	setSecondHedgePosition.store(true);
	return true;
}

bool setThirdPositionCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	ROS_INFO("Setting third position...");
	setThirdMapperPosition.store(true);
	setThirdHedgePosition.store(true);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibration_map_metrology_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<std::string>("odom_frame", odomFrame, "odom");
	pnh.param<std::string>("map_frame", mapFrame, "map");
	pnh.param<std::string>("mobile_hedge_frame", mobileHedgeFrame, "hedge");

	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
	tf2_ros::TransformListener tfListener(*tfBuffer);

	ros::Subscriber hedgeSubscriber = nh.subscribe("/hedge_pos", 1000, hedgeCallback);
	ros::Subscriber mapperSubscriber = nh.subscribe("/icp_odom", 1000, mapperCallback);

	ros::ServiceServer setFirstPositionService = nh.advertiseService("set_first_position", setFirstPositionCallback);
	ros::ServiceServer setSecondPositionService = nh.advertiseService("set_second_position", setSecondPositionCallback);
	ros::ServiceServer setThirdPositionService = nh.advertiseService("set_third_position", setThirdPositionCallback);

	ros::spin();

	lastMapToOdomTf.transform.translation = geometry_msgs::Vector3();
	geometry_msgs::Point firstMobileHedgePositionInOdomFrame;
        tf2::doTransform(firstMobileHedgePositionInMapFrame, firstMobileHedgePositionInOdomFrame, lastMapToOdomTf);
	geometry_msgs::Point secondMobileHedgePositionInOdomFrame;
        tf2::doTransform(secondMobileHedgePositionInMapFrame, secondMobileHedgePositionInOdomFrame, lastMapToOdomTf);
	geometry_msgs::Point thirdMobileHedgePositionInOdomFrame;
        tf2::doTransform(thirdMobileHedgePositionInMapFrame, thirdMobileHedgePositionInOdomFrame, lastMapToOdomTf);

	Eigen::Vector3d firstPointMarvelmind(firstHedgeMsg.x_m, firstHedgeMsg.y_m, firstHedgeMsg.z_m);
    Eigen::Vector3d secondPointMarvelmind(secondHedgeMsg.x_m, secondHedgeMsg.y_m, secondHedgeMsg.z_m);
    Eigen::Vector3d thirdPointMarvelmind(thirdHedgeMsg.x_m, thirdHedgeMsg.y_m, thirdHedgeMsg.z_m);
    Eigen::Vector3d firstPointMapper(firstMobileHedgePositionInOdomFrame.x, firstMobileHedgePositionInOdomFrame.y, firstMobileHedgePositionInOdomFrame.z);
    Eigen::Vector3d secondPointMapper(secondMobileHedgePositionInOdomFrame.x, secondMobileHedgePositionInOdomFrame.y, secondMobileHedgePositionInOdomFrame.z);
    Eigen::Vector3d thirdPointMapper(thirdMobileHedgePositionInOdomFrame.x, thirdMobileHedgePositionInOdomFrame.y, thirdMobileHedgePositionInOdomFrame.z);

	Eigen::Vector3d xMarvelmind = (thirdPointMarvelmind - secondPointMarvelmind).normalized();
	Eigen::Vector3d zMarvelmind = xMarvelmind.cross(firstPointMarvelmind - secondPointMarvelmind).normalized();
	Eigen::Vector3d yMarvelmind = zMarvelmind.cross(xMarvelmind).normalized();
	Eigen::Matrix3d rotationMatrixMarvelmind;
	rotationMatrixMarvelmind.col(0) = xMarvelmind;
	rotationMatrixMarvelmind.col(1) = yMarvelmind;
	rotationMatrixMarvelmind.col(2) = zMarvelmind;

	Eigen::Vector3d xMapper = (thirdPointMapper - secondPointMapper).normalized();
	Eigen::Vector3d zMapper = xMapper.cross(firstPointMapper - secondPointMapper).normalized();
	Eigen::Vector3d yMapper = zMapper.cross(xMapper).normalized();
	Eigen::Matrix3d rotationMatrixMapper;
	rotationMatrixMapper.col(0) = xMapper;
	rotationMatrixMapper.col(1) = yMapper;
	rotationMatrixMapper.col(2) = zMapper;

	Eigen::Matrix3d rotation = rotationMatrixMarvelmind * rotationMatrixMapper.inverse();
	std::cout << "[[" << rotation(0, 0) << "," << rotation(0, 1) << "," << rotation(0, 2) << "],[" << rotation(1, 0) << "," << rotation(1, 1) << "," << rotation(1, 2) << "],[" << rotation(2, 0) << "," << rotation(2, 1) << "," << rotation(2, 2) << "]]" << std::endl;

	return 0;
}


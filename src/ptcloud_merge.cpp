#include "ros/ros.h"
#include "../include/api.h"//api interface
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
#define Pt32 geometry_msgs::Point32

bool LReady = false, RReady = false;
double pan_angle;
vector<Pt32> leftPts, rightPts, allPts;

void rotate(const vector<Pt32>& in, vector<Pt32>& result, double angle){
    result.resize(MAX_PIX_NUM);
    for (int i = 0; i < MAX_PIX_NUM; ++i){
        result[i].x = - cos(angle) * in[i].x + sin(angle) * in[i].z;
        result[i].y = sin(angle) * in[i].x + cos(angle) * in[i].z;
        result[i].z = in[i].y;
    }
}

void LCallback(const sensor_msgs::PointCloud::ConstPtr& left) {
    rotate(left->points, leftPts, -pan_angle);
    LReady = true;
}

void RCallback(const sensor_msgs::PointCloud::ConstPtr& right) {
    rotate(right->points, rightPts, pan_angle);
    RReady = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ptcloud_merge");
	ros::NodeHandle n("~");
    n.param<double>("pan_angle", pan_angle, 0);
    allPts.resize(2*MAX_PIX_NUM);

	ros::Subscriber ptcloudL_sub = n.subscribe<sensor_msgs::PointCloud>("/ptcloudL", 1, LCallback);	
	ros::Subscriber ptcloudR_sub = n.subscribe<sensor_msgs::PointCloud>("/ptcloudR", 1, RCallback);	

	ros::Publisher ptcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/ptcloud_merge", 1);

    sensor_msgs::PointCloud2 merged;

    //populate the PointCloud message with static info
    merged.header.frame_id = "hps";
	merged.width = 2 * RES_WIDTH;
	merged.height = RES_HEIGHT;
	merged.fields.resize(3); //xyz fields
	merged.fields[0].name = "x"; merged.fields[1].name = "y"; merged.fields[2].name = "z";
	// All offsets are *4, as all field data types are float32
	int offset = 0;
	for (size_t d = 0; d < 3; ++d, offset += 4)
	{
		merged.fields[d].offset = offset;
		merged.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
		merged.fields[d].count  = 1;
	}

	merged.point_step = offset;
	merged.row_step   = merged.point_step * 2 * RES_WIDTH;
	merged.data.resize(2 * MAX_PIX_NUM * offset);
	merged.is_bigendian = false;
	merged.is_dense     = false;

	while(ros::ok())
	{
		if (LReady && RReady){
			LReady = false; RReady = false;

            merged.header.stamp = ros::Time::now();

            allPts.insert( allPts.end(), rightPts.begin(), rightPts.end() );
            allPts.insert( allPts.end(), leftPts.begin(), leftPts.end() );

            for (int i = 0; i < 2 * MAX_PIX_NUM; ++i){
                memcpy (&merged.data[i * merged.point_step + merged.fields[0].offset], &allPts[i].x, sizeof (float));
                memcpy (&merged.data[i * merged.point_step + merged.fields[1].offset], &allPts[i].y, sizeof (float));
                memcpy (&merged.data[i * merged.point_step + merged.fields[2].offset], &allPts[i].z, sizeof (float));
            }

            ptcloud_pub.publish(merged);

            allPts.clear();
        }
		ros::spinOnce();
	}
}
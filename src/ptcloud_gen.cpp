#include "ros/ros.h"//ros
#include "sensor_msgs/PointCloud2.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

ros::Publisher ptcloudL_pub, ptcloudR_pub;//Global variable, because the observer callback function needs to be used

sensor_msgs::PointCloud2 measureData;

//The observer callback function
void *Pubulish_cb(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	uint8_t a = handle->DeviceAddr;

	//populate the PointCloud message with dynamic info
	ros::Time scan_time = ros::Time::now();
	measureData.header.stamp = scan_time;

	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			case FULL_DEPTH_PACKET: /*点云数据和深度数据在这里获取*/
				for(int i = 0; i < MAX_PIX_NUM; i++){
					memcpy (&measureData.data[i * measureData.point_step + measureData.fields[0].offset], &event->MeasureData.point_cloud_data[0].point_data[i].x, sizeof (float));
					memcpy (&measureData.data[i * measureData.point_step + measureData.fields[1].offset], &event->MeasureData.point_cloud_data[0].point_data[i].y, sizeof (float));
					memcpy (&measureData.data[i * measureData.point_step + measureData.fields[2].offset], &event->MeasureData.point_cloud_data[0].point_data[i].z, sizeof (float));
				}
				if (a==0) ptcloudL_pub.publish(measureData);
				else if (a==1) ptcloudR_pub.publish(measureData);
				else printf("Error!!");
				break;
			default:
				printf("system error!\n");
				break;
		}
	}
}

void lidar_close(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *observer){
	if(HPS3D_RemoveDevice(handle) != RET_OK){
		printf("HPS3D_RemoveDevice failed\n");
	}	else {	
		printf("HPS3D_RemoveDevice succeed\n");
	}
	HPS3D_DisConnect(handle);
	HPS3D_RemoveObserver(observer);
}
//printf log callback function
void my_printf(char *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{
	system("sudo chmod 777 /dev/ttyACM*");
	
	ros::init(argc, argv, "ptcloud_gen");//ros init
	ros::NodeHandle n;//Create a node

	int offset = 0;
	char fileName[10][20];
	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	HPS3D_HandleTypeDef handle[DEV_NUM];
	AsyncIObserver_t My_Observer[OBSERVER_NUM];

	//populate the PointCloud2 message with static info 
	measureData.header.frame_id = "hps";

	measureData.width = RES_WIDTH;
	measureData.height = RES_HEIGHT;

	measureData.fields.resize(3); //xyz fields
	measureData.fields[0].name = "x"; measureData.fields[1].name = "y"; measureData.fields[2].name = "z";
	// All offsets are *4, as all field data types are float32
	for (size_t d = 0; d < 3; ++d, offset += 4)
	{
		measureData.fields[d].offset = offset;
		measureData.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
		measureData.fields[d].count  = 1;
	}

	measureData.point_step = offset;
	measureData.row_step   = measureData.point_step * RES_WIDTH;
	measureData.data.resize(MAX_PIX_NUM * offset);
	measureData.is_bigendian = false;
	measureData.is_dense     = false;

	//Create a topic
	ptcloudL_pub = n.advertise<sensor_msgs::PointCloud2>("ptcloudL", 1);	
	ptcloudR_pub = n.advertise<sensor_msgs::PointCloud2>("ptcloudR", 1);	

	HPS3D_SetMeasurePacketType(DEPTH_DATA_PACKET);
	
	dev_cnt = HPS3D_AutoConnectAndInitConfigDevice(handle);
	if(dev_cnt == 0)
	{
		printf("Can't Connect!!\n");
		return 0;
	} else {
		printf("Connected %d device(s) including\n",dev_cnt);
	}
	
	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);

	//Smooth Edges
	HPS3D_SetEdgeDetectionEnable (true);
	HPS3D_SetEdgeDetectionValue (1000);

	//Point Data Setting
	HPS3D_SetPointCloudEn(true);
	HPS3D_SetOpticalEnable(handle, true);
	HPS3D_SetPacketType(handle, PACKET_FULL);

	for (int i = 0; i < dev_cnt; i++){
		printf("%s\n",handle[i].DeviceName);
	
		uint8_t a = handle[i].DeviceName[strlen(handle[i].DeviceName)-1] % 2;
		HPS3D_SetDevAddr(&handle[i], a);

		My_Observer[a].AsyncEvent = ISubject_Event_DataRecvd;
		My_Observer[a].NotifyEnable = true;
		My_Observer[a].ObserverID = a;
	}

	// Setting Distance Filter using Kalman Filter
	DistanceFilterConfTypeDef set_conf;
	HPS3D_SetDistanceFilterType(handle, DISTANCE_FILTER_SIMPLE_KALMAN );
	set_conf.kalman_K = 0.3;
	set_conf.kalman_threshold = 50;
	set_conf.num_check = 2;
	HPS3D_SetSimpleKalman(handle, set_conf);

	//Add observers
	HPS3D_AddObserver(&Pubulish_cb,handle,My_Observer);

	for (int i = 0; i < dev_cnt; i++){
		//Set running mode
		handle[i].RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&handle[i]);
	}

	printf("Start Streaming...\n");
	while(ros::ok());

	for (int i = 0; i < dev_cnt; i++){
		if(HPS3D_RemoveDevice(&handle[i]) != RET_OK){
			printf("HPS3D_RemoveDevice failed\n");
		}	else {	
			printf("HPS3D_RemoveDevice succeed\n");
		}
		HPS3D_DisConnect(&handle[i]);
		HPS3D_RemoveObserver(&My_Observer[i]);
	}
	return 0;
}


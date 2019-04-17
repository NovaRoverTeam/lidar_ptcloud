#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

ros::Publisher ptcloud_pub;//Global variable, because the observer callback function needs to be used

//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	sensor_msgs::PointCloud measureData;	
	measureData.points.resize(MAX_PIX_NUM);

	ros::Time scan_time = ros::Time::now();
	//populate the PointCloud message
	measureData.header.stamp = scan_time;
	measureData.header.frame_id = "hps";

	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			// case SIMPLE_ROI_PACKET:
			// 	printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_roi_data[0].distance_average,event->RetPacketType);
			// 	break;
			// case FULL_ROI_PACKET:
			// 	break;
			case FULL_DEPTH_PACKET: /*点云数据和深度数据在这里获取*/
				for(int i = 0; i < MAX_PIX_NUM; i++){
					geometry_msgs::Point32 pt;
					pt.x = event->MeasureData.point_cloud_data[0].point_data[i].x;
					pt.y = event->MeasureData.point_cloud_data[0].point_data[i].y;
					pt.z = event->MeasureData.point_cloud_data[0].point_data[i].z;
					measureData.points[i] = pt;
				}
				ptcloud_pub.publish(measureData);
				break;
			// case SIMPLE_DEPTH_PACKET:
			// 	printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_depth_data->distance_average,event->RetPacketType);
			// 	break;
			// case OBSTACLE_PACKET:
			// 	printf("Obstacle ID：%d\n",event->MeasureData.Obstacle_data->Id);
			// 	if(event->MeasureData.Obstacle_data->Id > 20)
			// 	{
			// 		handle->RunMode = RUN_IDLE;
			// 		HPS3D_SetRunMode(handle);
			// 	}
			// 	break;
			// case NULL_PACKET:
			// 	printf("null packet\n");
			// 	//The return packet type is empty
			// 	break;
			default:
				printf("system error!\n");
				break;
		}
	}
}

RET_StatusTypeDef lidar_init(uint8_t *filename, HPS3D_HandleTypeDef *handle, AsyncIObserver_t *observer){
	RET_StatusTypeDef ret = RET_OK;
	handle->DeviceName = filename;

	//Device Connection
	ret = HPS3D_Connect(handle);
	if(ret != RET_OK)
	{
		printf("Device open failed！ret = %d\n",ret);
	}
	
	//Point Data Setting
	HPS3D_SetOpticalEnable(handle, true);
	
	//Device init
	ret = HPS3D_ConfigInit(handle);
	if(RET_OK != ret)
	{
		printf("Initialization failed:%d\n", ret);
	} else {
		printf("Initialization succeed with DeviceAddr = %d\n", handle->DeviceAddr);
	}
	
	//Observer callback function and initialization
	observer->AsyncEvent = ISubject_Event_DataRecvd;
	observer->NotifyEnable = true;
	observer->ObserverID = 0;
	observer->RetPacketType = NULL_PACKET;

	handle->OutputPacketType = PACKET_FULL;
	HPS3D_SetPacketType(handle);

	//Add observer one
	HPS3D_AddObserver(&User_Func, handle, observer);		

	if(ret != RET_OK)
	{
		//Remove device and disconnect
		HPS3D_RemoveDevice(handle);
		printf("Initialization failed, Remove device\n");
	}

	//Set running mode
	handle->SyncMode = ASYNC;
	handle->RunMode = RUN_CONTINUOUS;
	HPS3D_SetRunMode(handle);

	return ret;
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
void my_printf(uint8_t *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{
	system("sudo chmod 777 /dev/ttyACM*");
	
	ros::init(argc, argv, "ptcloud_gen");//ros init
	ros::NodeHandle n;//Create a node

	uint32_t a = 0;
	uint8_t fileName[10][20];
	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	HPS3D_HandleTypeDef handle;
	AsyncIObserver_t My_Observer;

	//Create a topic
	ptcloud_pub = n.advertise<sensor_msgs::PointCloud>("ptcloud", 1000);	

	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);
	HPS3D_SetPointCloudEn(true);

	//Lists the optional devices
	dev_cnt = HPS3D_GetDeviceList((uint8_t *)"/dev/",(uint8_t *)"ttyACM",fileName);
	printf("%s\n", fileName[0]);
	
	lidar_init(fileName[0], &handle, &My_Observer);

	while(ros::ok());

	lidar_close(&handle, &My_Observer);
	return 0;
}


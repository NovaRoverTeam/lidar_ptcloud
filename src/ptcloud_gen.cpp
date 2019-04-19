#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

ros::Publisher ptcloudL_pub, ptcloudR_pub;//Global variable, because the observer callback function needs to be used

//The observer callback function
void *Pubulish_cb(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	uint8_t a = handle->DeviceAddr;

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
				if (a==0) ptcloudL_pub.publish(measureData);
				if (a==1) ptcloudR_pub.publish(measureData);
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

	char fileName[10][20];
	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	HPS3D_HandleTypeDef handle[DEV_NUM];
	AsyncIObserver_t My_Observer[OBSERVER_NUM];

	//Create a topic
	ptcloudL_pub = n.advertise<sensor_msgs::PointCloud>("ptcloudL", 1);	
	ptcloudR_pub = n.advertise<sensor_msgs::PointCloud>("ptcloudR", 1);	

	HPS3D_SetMeasurePacketType(DEPTH_DATA_PACKET);
	
	dev_cnt = HPS3D_AutoConnectAndInitConfigDevice(handle);
	if(dev_cnt == 0)
	{
		printf("Can't Connect!!\n");
		return 0;
	} else {
		printf("#Connected Devices: %d\n",dev_cnt);
	}
	
	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);

	//Point Data Setting
	HPS3D_SetPointCloudEn(true);

	for (int i = 0; i < dev_cnt; i++){
		printf("%s\n",handle[i].DeviceName);
		
		HPS3D_SetOpticalEnable(&handle[i], true);
		HPS3D_SetPacketType(&handle[i], PACKET_FULL);
		HPS3D_SetDevAddr(&handle[i], (uint8_t)i);

		printf("%d\n",handle[i].DeviceAddr);

		My_Observer[i].AsyncEvent = ISubject_Event_DataRecvd ; /*异步通知事件为数据接收*/
		My_Observer[i].NotifyEnable = true; /*使能通知事件*/
		My_Observer[i].ObserverID = (uint8_t)i; /*观察者ID*/

		printf("%d\n",My_Observer[i].ObserverID);
	}

	//Add observers
	HPS3D_AddObserver(&Pubulish_cb,handle,My_Observer);

	for (int i = 0; i < dev_cnt; i++){
		//Set running mode
		handle[i].RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&handle[i]);
	}

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


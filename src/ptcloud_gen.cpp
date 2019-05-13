#include <ros/ros.h>//ros
#include <sensor_msgs/PointCloud.h>
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

ros::Publisher ptcloudL_pub, ptcloudR_pub;//Global variable, because the observer callback function needs to be used

sensor_msgs::PointCloud measureData;

//The observer callback function
void *Pubulish_cb(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	uint8_t a = handle->DeviceAddr;

	//populate the PointCloud message with dynamic info
	measureData.header.stamp = ros::Time::now();

	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			case FULL_DEPTH_PACKET:
				for(int i = 0; i < MAX_PIX_NUM; i++){
					geometry_msgs::Point32 pt;
					pt.x = event->MeasureData.point_cloud_data[0].point_data[i].x/1000;
					pt.y = event->MeasureData.point_cloud_data[0].point_data[i].y/1000;
					pt.z = event->MeasureData.point_cloud_data[0].point_data[i].z/1000;
					measureData.points[i] = pt;
				}
				if (a==0) ptcloudL_pub.publish(measureData);
				else if (a==1) ptcloudR_pub.publish(measureData);
				break;
			default:
				printf("system error!\n");
				break;
		}
	}
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

	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	HPS3D_HandleTypeDef handle[DEV_NUM];
	AsyncIObserver_t My_Observer[OBSERVER_NUM];

	//Create a topic
	ptcloudL_pub = n.advertise<sensor_msgs::PointCloud>("/ptcloudL", 1);	
	ptcloudR_pub = n.advertise<sensor_msgs::PointCloud>("/ptcloudR", 1);	

	//populate the PointCloud message with static info
	measureData.header.frame_id = "hps";
	measureData.points.resize(MAX_PIX_NUM);

	HPS3D_SetMeasurePacketType(DEPTH_DATA_PACKET);
	
	dev_cnt = HPS3D_AutoConnectAndInitConfigDevice(handle);
	if(dev_cnt == 0)
	{
		printf("Can't Connect!!\n");
		return 0;
	} else {
		printf("Connected %d devices including:\n",dev_cnt);
	}
	
	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);

	//Point Data Setting
	HPS3D_SetPointCloudEn(true);
	HPS3D_SetOpticalEnable(handle, true);
	HPS3D_SetPacketType(handle, PACKET_FULL);

	for (int i = 0; i < dev_cnt; i++){
		printf("%s\n",(handle+i)->DeviceName);
	
		uint8_t a = (handle+i)->DeviceName[strlen((handle+i)->DeviceName)-1] % 2;
		printf("%d\n",a);
		HPS3D_SetDevAddr(handle+i, a);
		printf("%d\n",(handle+i)->DeviceAddr);
		My_Observer[a].AsyncEvent = ISubject_Event_DataRecvd ; /*异步通知事件为数据接收*/
		My_Observer[a].NotifyEnable = true; /*使能通知事件*/
		My_Observer[a].ObserverID = a; /*观察者ID*/

		//Add observers
		HPS3D_AddObserver(&Pubulish_cb,handle+a,My_Observer+a);

	}

	//Add observers
	//HPS3D_AddObserver(&Pubulish_cb,handle,My_Observer);

	// Setting Distance Filter using Kalman Filter
	DistanceFilterConfTypeDef set_conf;
	HPS3D_SetDistanceFilterType(handle, DISTANCE_FILTER_SIMPLE_KALMAN );
	set_conf.kalman_K = 0.6;
	set_conf.kalman_threshold = 1000;
	set_conf.num_check = 10;
	HPS3D_SetSimpleKalman(handle, set_conf);

	for (int i = 0; i < dev_cnt; i++){
		//Set running mode
		handle[i].RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&handle[i]);
	}

	printf("Start Streaming...");
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


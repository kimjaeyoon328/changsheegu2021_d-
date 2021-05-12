#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

float closeball_X;
float closeball_Y;
int closedir;
int approach_done;
int action;

int len;
int n;

#define RAD2DEG(x) ((x)*180./M_PI)



void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
		map_mutex.lock();

		int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];

    }
		map_mutex.unlock();

}
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

}

void forward_step(ros::Publisher left_wheel, ros::Publisher right_wheel) // 계단 오를 때 쓸 전진 속도 빠른 명
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=-2.8;
	right_wheel_msg.data=-2.8;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(1).sleep();
	ros::spinOnce();	
}

void go_forward(ros::Publisher left_wheel, ros::Publisher right_wheel) 
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=-6;
	right_wheel_msg.data=-6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.02).sleep();
	ros::spinOnce();	
}


// 피니언 내리기

void pinion_down(ros::Publisher pinion_move){
	std_msgs::Float64 joint_command;
	joint_command.data = -1;
	pinion_move.publish(joint_command);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
}

// 피니언 올리기
void pinion_up(ros::Publisher pinion_move){
	std_msgs::Float64 joint_command;
	joint_command.data = 1;
	pinion_move.publish(joint_command);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
}


//정지

void stop(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data = 0;
	right_wheel_msg.data = 0;
	left_wheel.publish(left_wheel_msg);  
	right_wheel.publish(right_wheel_msg); 
	ros::Duration(0.5).sleep();
	ros::spinOnce();
}


// 오른쪽 회전
void right_turn(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=-6;
	right_wheel_msg.data=6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.5).sleep();
	ros::spinOnce();	
	
}


// 왼쪽 회전
void left_turn(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=6;
	right_wheel_msg.data=-6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.5).sleep();
	ros::spinOnce();	
	
}

void s_right_turn(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=-6;
	right_wheel_msg.data=6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.01).sleep();
	ros::spinOnce();	
	
}


// 왼쪽 미세 회전
void s_left_turn(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=6;
	right_wheel_msg.data=-6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(0.01).sleep();
	ros::spinOnce();	
	
}

void cage_dump(ros::Publisher cage_move){
	std_msgs::Float64 joint_command;
	joint_command.data= 1.7;
	cage_move.publish(joint_command);   
	ros::Duration(3.5).sleep();
	ros::spinOnce();
}

void cage_restore(ros::Publisher cage_move){
	std_msgs::Float64 joint_command;
	joint_command.data= -0.02;
	cage_move.publish(joint_command);   
	ros::Duration(5).sleep();
	ros::spinOnce();
}


// 180도 도는 회전  (정확하지는 않음 약 180)
void back_turn(ros::Publisher left_wheel, ros::Publisher right_wheel)
{
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	left_wheel_msg.data=6;
	right_wheel_msg.data=-6;
	left_wheel.publish(left_wheel_msg);   
	right_wheel.publish(right_wheel_msg);
	ros::Duration(4).sleep();
	ros::spinOnce();	
	
}

void dump(ros::Publisher left_wheel, ros::Publisher right_wheel,ros::Publisher cage_move){
	back_turn(left_wheel, right_wheel);
	stop(left_wheel,right_wheel);
	cage_dump(cage_move);
	cage_restore(cage_move);
	stop(left_wheel,right_wheel);
}

// 회전이 가능한지 lidar data로 테스트 : 회전 가능 ->1 else 0
int turn_available(){
	
	int ans=1;
	for(int i = 0; i < lidar_size; i++)
    {
        if (0.015<lidar_distance[i] && lidar_distance[i] <0.5){ //0.5를 수정해서 criteria dis 변경 가능 m 단위
        	//std::cout << "0 maker " << lidar_distance[i] <<" | "<< i<< "\n" << std::endl;
        	ans=0;
        	return ans;
        	}

    }
    return ans;
    ros::Duration(0.5).sleep();
}

// 왼쪽으로 90도 정도 왼쪽으로 회전하면서 가장 가까운 공을 closeball_X,closeball_Y에 찾음 
// 가장 가까운 공이 일정 범위 안으로 들어오면 approach_done이 1이 됨
void detect_balldest(ros::Publisher left_wheel, ros::Publisher right_wheel){
	int rot_n=0;
	float distance_ball=100;
	float dummy_dis=0;
	float priority=10;
	approach_done=0;
	
	for(int i=0; i<4;i++){
		for(int j = 0; j < ball_number; j++)
    		{
    			dummy_dis=sqrt(ball_X[j]*ball_X[j]+ball_Y[j]*ball_Y[j]);
        		if(distance_ball > dummy_dis)
			{
				distance_ball=dummy_dis;
				rot_n=i;
			}
    		}
    		
		if(turn_available()){
			left_turn(left_wheel,right_wheel);
		}
	}
	for(int k=0; k<(4-rot_n); k++){
		right_turn(left_wheel,right_wheel);
	}
	distance_ball=100;
	for(int j = 0; j < ball_number; j++)
    		{
    			dummy_dis=sqrt(ball_X[j]*ball_X[j]+ball_Y[j]*ball_Y[j]);
        		if(distance_ball > dummy_dis)
			{
				closeball_X=ball_X[j];
				closeball_Y=ball_Y[j];
			}
    		}
	if(distance_ball<0.5){
    		approach_done=1;
    	}

}

// 정면에서 가장 가까운 공 찾음 위에 있는 함수랑 차이는 scan 하는 구역이 정면 뿐이라는 점
// 가장 가까운 공이 일정 범위 안으로 들어오면 approach_done이 1이 됨
void test_balldest(){
	int rot_n=0;
	float distance_ball=100;
	float dummy_dis=0;
	float priority=10;
	approach_done=0;
	distance_ball=100;
	for(int j = 0; j < ball_number; j++)
    		{
    			dummy_dis=sqrt(ball_X[j]*ball_X[j]+ball_Y[j]*ball_Y[j]);
        		if(distance_ball > dummy_dis)
			{
				closeball_X=ball_X[j];
				closeball_Y=ball_Y[j];
			}
    		}
    	if(distance_ball<0.5){
    		approach_done=1;
    	}

}

// 가장 가까운 공에 align 추후에 priority 도입예정
void align(ros::Publisher left_wheel, ros::Publisher right_wheel){
	
	while(closeball_X < -0.02 | closeball_X > 0.02){
		if(closeball_X<0){
			s_right_turn(left_wheel,right_wheel);
		}
		else{
			s_left_turn(left_wheel,right_wheel);
		}
		test_balldest();	
	}
}

// 가장 가까운 공에 접근하는 함수
void go_to_ball(ros::Publisher left_wheel, ros::Publisher right_wheel){
	while(approach_done!=1){
		align(left_wheel,right_wheel);
		go_forward(left_wheel,right_wheel);
	}
}

	

// 계단을 오르는 command 를 만들려고 하고 있습니다
void step_overcome(ros::Publisher pinion_move, ros::Publisher left_wheel, ros::Publisher right_wheel){
	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std_msgs::Float64 joint_command;
	pinion_down(pinion_move);
	forward_step(left_wheel,right_wheel);
	stop(left_wheel,right_wheel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integrate");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    
    ros::Publisher pinion_joint = n.advertise<std_msgs::Float64>("joint_v", 10);
    
    ros::Publisher cage_joint = n.advertise<std_msgs::Float64>("cage", 10);
    
    
	ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("joint_front_left_wheel", 10);
	ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("joint_front_right_wheel", 10);
	
	



    while(ros::ok){
	std_msgs::Float64 left_wheel_msg;
			std_msgs::Float64 right_wheel_msg;

			left_wheel_msg.data=0;   // set left_wheel velocity
			right_wheel_msg.data=0;  // set right_wheel velocity
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		/////////////////////////////////////////////////////////////////////////////////////////////////

	 //  for(int i = 0; i < lidar_size; i++
   // {
	 //    std::cout << "degree : "<< lidar_degree[i];
	 //    std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
	 //  }
		// for(int i = 0; i < ball_number; i++)
		// {
		// 	std::cout << "ball_X : "<< ball_X[i];
		// 	std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
   //
		// }

		  pub_left_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
		  pub_right_wheel.publish(right_wheel_msg);  // publish right_wheel velocity
		 
		 dump(pub_left_wheel,pub_right_wheel,cage_joint);
		 stop(pub_left_wheel,pub_right_wheel);
		 
	


	    ros::Duration(1).sleep();
	    ros::spinOnce();
	    
	    
	    
    }

    return 0;
}

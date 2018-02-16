// 2018 Copyright ABB, Power-One Italy S.p.A, all rights reserved 
// insert copyright 

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <boost/lexical_cast.hpp>
#include <string>

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>


// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread.hpp>


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;




std::string mess="init";
std::string mess1="init";
std::string mess2="!r";
serial::Serial* _my_serial;
serial::Serial* _my_serial1;
serial::Serial* _my_serial2;
serial::Serial* _my_serial3;
size_t n_char;
int var_stop=0;
int prec=0;
vector <string> MotorName;
vector <string> PortName;
vector <serial::Serial*> serial_list;

ros::Publisher pub_box_status;
ros::Publisher pub_program_status;
ros::Subscriber sub;



void print_info(std::string str){

  ROS_INFO("[%s] %s", ros::this_node::getName().c_str(), str.c_str());
}


void enumerate_ports()
{
	int cont=0;
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
		if(device.description=="Roboteq Motor Controller SBL1XXX") PortName.push_back(device.port.c_str());
	}
	for(int i=0; i<PortName.size(); i++){
		ROS_INFO("Port: %s", PortName[i].c_str());
	}
	
}

void enumerate_name(serial::Serial* tmp_s){
     bool find= false;
     while(!find){
          string tmp = tmp_s->readline();
          tmp.erase(tmp.find_last_not_of(" \n\r\t")+1);
	  print_info("serial read: " + tmp);
	  if(tmp.find("PLX")!=std::string::npos){
		 MotorName.push_back("PLX");
		 find=true;
		 break;
	  }
          else if(tmp.find("ALX")!=std::string::npos){
		 MotorName.push_back("ALX");
		 find=true;
		 break;
	  }
          else if(tmp.find("ARX")!=std::string::npos){
		 MotorName.push_back("ARX");
		 find=true;
		 break;
	  }
          else if(tmp.find("PRX")!=std::string::npos){
		 MotorName.push_back("PRX");
		 find=true;
		 break;
	  }
    }
}

void stopp(){

  while(ros::ok()){

	if(var_stop==1 && prec==0 ) {
		_my_serial->write("!MS 1");
		_my_serial1->write("!MS 1");
		ROS_INFO("stop thread launched");
		prec=1;
	}
		
  }
}

void serialRead(){

  while(ros::ok()){
  	//string tmp = _my_serial->read(1);
	string tmp = _my_serial->readline();
	
	tmp.erase(tmp.find_last_not_of(" \n\r\t")+1);
	print_info("serial read: " + tmp);

	usleep(1*1000);
  }
}


void redirect(const geometry_msgs::Twist::ConstPtr& msg){

	geometry_msgs::Twist vel = *msg;

	_my_serial=serial_list[0];
	_my_serial1=serial_list[1];

	_my_serial->flush();
	_my_serial1->flush();


	ROS_INFO("Received");

	if(vel.linear.x!=0 && vel.angular.z==0){
		mess = "!G 1 "+ boost::lexical_cast<string>((vel.linear.x*100))+"\r\n";
		mess1 = "!G 1 "+ boost::lexical_cast<string>((-vel.linear.x*100))+"\r\n";
		var_stop=0;
		prec=0;
		
	}
	else if(vel.linear.x==0 && vel.angular.z!=0){
	ROS_INFO("angular");
		mess = "!G 1 "+ boost::lexical_cast<string>((vel.angular.z*100)/2)+"\r\n";
		mess1 = "!G 1 "+ boost::lexical_cast<string>((vel.angular.z*100)/2)+"\r\n";
		var_stop=0;
		prec=0;
	}
	else if(vel.linear.x==0 && vel.linear.y==0 && vel.angular.z==0) {
		mess="!MS 1";
		mess1="!MS 1";
		var_stop=1;
	}
	
	ROS_INFO("Velocita emessa: %s ", mess.c_str());
	ROS_INFO("Velocita emessa port2: %s ", mess1.c_str());
        _my_serial->write("!r\r");
	_my_serial1->write("!r\r");
	size_t bytes_wrote = _my_serial->write(mess);
	size_t bytes_wrote1 = _my_serial1->write(mess1);
	


}

int main(int argc, char **argv) {

	ros::init(argc, argv, "usb_control");
	ros::NodeHandle n;

	std::string serial_port;

	if(argc < 2)
		serial_port = "/dev/ttyACM0";

	int serial_created = 0;

	serial_list.push_back(_my_serial);
	serial_list.push_back(_my_serial1);
	serial_list.push_back(_my_serial2);
	serial_list.push_back(_my_serial3);
	//name the serial
	enumerate_ports();

	std_msgs::String msg;

	ROS_INFO("C1");

        for(int i=0; i<PortName.size(); i++){

	 	try{ 
		
			serial_port=PortName[i];
 			serial_list[i] = new serial::Serial(serial_port, 115200, serial::Timeout::simpleTimeout(1000));
			ROS_INFO("OPENED PORT %d", i);
			serial_created++;
	        	serial_list[i]->flush();


		
	 	}
	 	catch(exception &e){

			ROS_ERROR("Unhandled Exception: %s", e.what());
			msg.data = std::string(e.what());
			sleep(1);	
	 	}



	}


//boost::thread* thr3 = new boost::thread(serialRead);

//enumerate_name(_my_serial);
//enumerate_name(_my_serial1);
/* for(int i=0; i<PortName.size(); i++){
	ROS_INFO("Port %s associated to Motor %s", PortName[i].c_str(), MotorName[i].c_str());
 }*/

	boost::thread* thr2 = new boost::thread(stopp);

 	sub=n.subscribe("cmd_vel", 1, redirect);


	ros::spin();

 
 return 0;
}

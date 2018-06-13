#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <alice_msgs/RoboCupGameControlReturnData.h>

#include "robocup_struct.h"

using namespace std;

RoboCupGameControlReturnData team_info;
bool info_recv = false;

void RobotInfoCallback(const alice_msgs::RoboCupGameControlReturnData &msg)
{
  info_recv = true;
  strcpy(team_info.header, "RGrt");
  team_info.version = 3;
  team_info.team    = msg.team;
  team_info.player  = msg.player;
  team_info.message = msg.message;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "referee_sender_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_game_state = nh.subscribe("/heroehs/alice/robot_info", 1, RobotInfoCallback);

  int broadcast = 1;
  // make socket variable
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1)
    exit(1);

  // information of the game_controller. you don't have to bind for sendto().
  sockaddr_in controller_addr;
  //bzero(&server_addr, sizeof(server_addr));    // set it 0

  controller_addr.sin_family = AF_INET;
  controller_addr.sin_port = htons(3939);    // set port number
  controller_addr.sin_addr.s_addr = htonl(INADDR_ANY);    // = inet_addr("0.0.0.0"); // broadcast ip addr

  while(ros::ok())
  {
    if(info_recv)
    {
      sendto(sock, (char*)&team_info, sizeof(team_info), 0, (sockaddr*)&controller_addr, sizeof(controller_addr));
      info_recv = false;
    }
    
    ros::spinOnce();
    usleep(10000);
  }
  close(sock);

  return 0;
}


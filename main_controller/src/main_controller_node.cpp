#include <iostream>
#include <stdlib.h>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <alice_msgs/MoveCommand.h>
#include <alice_msgs/RoboCupGameControlData.h>
#include <alice_msgs/RoboCupGameControlReturnData.h>
#include <alice_msgs/FoundObject.h>
#include <alice_msgs/FoundObjectArray.h>
#include <robotis_controller_msgs/StatusMsg.h>

#include "log_manager.h"

using namespace std;

void GameStateCallback(const alice_msgs::RoboCupGameControlData &msg);
//void OnProcessCallback(const std_msgs::Bool &msg);
void OnProcessCallback(const robotis_controller_msgs::StatusMsg &msg);
void DetectedObjectsCallback(const alice_msgs::FoundObjectArray &msg);

const float pi = 3.14159294;

class Alice
{
  Log *log_file;
public:
  ros::Publisher pub_return_data;
  ros::Publisher pub_move_command;
  ros::Publisher pub_head_command;
  ros::Publisher pub_emergency;
  ros::Subscriber sub_game_state;
  ros::Subscriber sub_on_process;
  ros::Subscriber sub_detected_objects;
  alice_msgs::FoundObjectArray object_info; 
  alice_msgs::RoboCupGameControlReturnData robot_info;
  alice_msgs::MoveCommand move_command_msg;
  std_msgs::UInt8 head_command_msg;

  bool robot_state_changed;
  bool game_state_changed;
  bool on_process;
  // -1:no signal, 0:init, 1:ready, 2:set, 3:play, 4:finished
  int game_state;
  // -1:no signal, 0:normal, 1~n:penalised
  int robot_state;
  int last_robot_state;
  int last_game_state;

  ros::Time cur_time;

  ros::Time signal_timer;
  ros::Duration cool_time;
  bool is_cooling;

  alice_msgs::FoundObject obj[5];
  ros::Time obj_lost[5];
  bool obj_recv[5];

  bool is_moving;
  bool is_kicking;

  Alice(){}
  Alice(ros::NodeHandle &nh):game_state(-1), robot_state(-1), on_process(false)
  {
    string path = "/home/alice2-nuke/logs/robocup_2018/main_controller";
    string file_name = "/"+GetYYMMDD()+"_main_controller_"+GetSeconds(ros::Time::now().sec)+".txt";
    log_file = new Log(path+file_name);
    sub_game_state       = nh.subscribe("/heroehs/game_state", 1, GameStateCallback);
    //    sub_on_process       = nh.subscribe("/heroehs/alice/on_process", 1, OnProcessCallback);
    sub_on_process       = nh.subscribe("/heroehs/status", 1, OnProcessCallback);
    sub_detected_objects = nh.subscribe("/heroehs/detected_objects", 1, DetectedObjectsCallback);

    pub_return_data  = nh.advertise<alice_msgs::RoboCupGameControlReturnData>("/heroehs/alice/robot_info", 1);
    pub_move_command = nh.advertise<alice_msgs::MoveCommand>("/heroehs/alice/move_command", 1);
    pub_head_command = nh.advertise<std_msgs::UInt8>("/heroehs/alice/head_command", 1);
    pub_emergency    = nh.advertise<std_msgs::Bool>("/heroehs/alice/emergency_stop", 1);

    int t, r;
    nh.param("team_number", t, 0);
    nh.param("robot_number", r, 0);
    robot_info.team = t;
    robot_info.player = r;
    robot_info.message = 0;

    cool_time = ros::Duration(1.5);
    is_moving = false;
  }

  // obj[0] : ball
  // obj[1] : center
  // obj[2] : robot
  // obj[3] : enemy goalpost
  // obj[4] : our goalpost
  void Update()
  {
    cur_time = ros::Time::now();
    Read();
    StateCheck();
    //cout << on_process << "," << is_cooling<<endl;
    if(game_state == 2 || robot_state != 0) // when game state is "set", or robot penalized
    {
      if(game_state_changed || robot_state_changed)  // play once
      {
        head_command_msg.data = 0;  // init tracking mode
        on_process = false;
        Stop();
        Ready();
      }
    }
    else if(game_state == 3 && robot_state == 0)  // when game state is "play", and robot playing
    {
      Attack();
      //Defense();

      if(!is_kicking)
      {
        if(!obj_recv[0])  // didn't receive ball data
          head_command_msg.data = 1;  // detect mode
        else
          head_command_msg.data = 2;  // ball tracking mode
      }
    }
    else
    {
      if(game_state_changed)  // play once
      {
        head_command_msg.data = 0;  // init mode
        on_process = false;
        Stop();
      }
    }
    pub_head_command.publish(head_command_msg);
    stringstream ss;
    ss << "[ROS Time " << cur_time.sec << "." << cur_time.nsec << "]" << endl
       << "head_command_msg.data        : " << head_command_msg.data << endl
       << "move_command_msg.mode        : " << move_command_msg.mode << endl
       << "move_command_msg.transform.x : " << move_command_msg.transform.x << endl
       << "move_command_msg.transform.y : " << move_command_msg.transform.y << endl
       << "move_command_msg.transform.z : " << move_command_msg.transform.z << endl;
    log_file->Write(ss.str());
  }

  void Ready()
  {
  }

  void Attack()
  {
    float angle = atan2(obj[0].pos.y, obj[0].pos.x);
    //cout << angle << endl;
    if(!on_process && !is_cooling)
    {
      if(obj_recv[0]) // received ball data
      {
        if(obj_lost[3]+ros::Duration(2) < cur_time)//obj_recv[3]) // received goalpost data
        {
          float d_a = atan2(obj[3].pos.x - obj[0].pos.x, obj[3].pos.y - obj[0].pos.y);
          float d_x = obj[0].pos.x - sin(d_a)*0.4;
          float d_y = obj[0].pos.y - cos(d_a)*0.4;
          angle = atan2(d_y, d_x);
          if(obj[0].pos.x < 0.4 && fabs(obj[0].pos.y) < 0.2)
          {
            Kick();
          }
          else if(obj[0].pos.x > 0.4 && fabs(angle) < (float)10/180*pi)
          {
            StraightWalk(obj[0].pos.x-0.4);
          }
          else if(fabs(angle) > (float)10/180*pi)
          {
            TurnWalk(angle);
          }
        }
        else // didn't received goalpost data
        {
          if(obj[0].pos.x < 0.4 && fabs(obj[0].pos.y) < 0.2)
          {
            Kick();
          }
          else if(obj[0].pos.x > 0.4 && fabs(angle) < (float)10/180*pi)
          {
            StraightWalk(obj[0].pos.x-0.4);
          }
          else if(fabs(angle) > (float)10/180*pi)
          {
            TurnWalk(angle);
          }
        }
      }
      else if(cur_time - obj_lost[0] < ros::Duration(2))
      {
        if(obj[0].pos.x < 0.4 && fabs(obj[0].pos.y) < 0.2)
        {
          Kick();
        }
      }

//      else  // didn't receive ball data
//      {
//        head_command_msg.data = 1;  // detect mode
//      }
    }
    else  // when on_process or is_cooling
    {
      if(obj_recv[0]) // receive ball data
      {
        if(move_command_msg.mode == 0)
        {
          if(move_command_msg.command == 0) // is straight walk
          {
            if(fabs(obj[0].pos.x) <= 0.4)
              Stop();
          }
          else if(move_command_msg.command == 1)  // is side walk
          {
            if(fabs(obj[0].pos.y) <= 0.4)
              Stop();
          }
          else if(move_command_msg.command == 2)  // is turning
          {
            if(move_command_msg.transform.z < 0 && angle >= 0)
              Stop();
            else if(move_command_msg.transform.z > 0 && angle <= 0)
              Stop();
          }
        }
      }
    }
  }

  void Defense()
  {
  }

  void Stop()
  {
    if(is_moving)
    {
      move_command_msg.mode = 2;
      pub_move_command.publish(move_command_msg);
      SetSignalTimer();
      is_moving = false;
    }
  }

  void StateCheck()
  {
    // check cool time
    if(signal_timer <= ros::Time::now())
      is_cooling = false;
    else
      is_cooling = true;

    // check robot state changed
    if(robot_state != last_robot_state)
    {
      robot_state_changed = true;
      last_robot_state = robot_state;
    }
    else
    {
      robot_state_changed = false;
    }

    // check game state changed
    if(game_state != last_game_state)
    {
      game_state_changed = true;
      last_game_state = game_state;
    }
    else
    {
      game_state_changed = false;
    }
  }

  void Kick()
  {
    is_kicking = true;
    head_command_msg.data = 0;
    move_command_msg.mode = 1;
    if(obj[0].pos.y > 0)
      move_command_msg.command = 0; // left kick
    else
      move_command_msg.command = 1; // right kick
    pub_move_command.publish(move_command_msg);
    SetSignalTimer();
  }

  void StraightWalk(float d)
  {
    head_command_msg.data = 2;
    move_command_msg.mode        = 0;
    move_command_msg.command     = 0;
    move_command_msg.transform.x = d;
    pub_move_command.publish(move_command_msg);
    SetSignalTimer();
    is_moving = true;
  }

  void SideWalk(float d)
  {
    head_command_msg.data = 2;
    move_command_msg.mode        = 0;
    move_command_msg.command     = 1;
    move_command_msg.transform.y = d;
    pub_move_command.publish(move_command_msg);
    SetSignalTimer();
    is_moving = true;
  }

  void TurnWalk(float d)
  {
    head_command_msg.data = 2;
    move_command_msg.mode        = 0;
    move_command_msg.command     = 2;
    move_command_msg.transform.z = d;
    pub_move_command.publish(move_command_msg);
    SetSignalTimer();
    is_moving = true;
  }

  void PubReturnData()
  {
    pub_return_data.publish(robot_info);
  }

  void Read()
  {
    obj_recv[0] = false;  // ball
    obj_recv[1] = false;  // center
    obj_recv[2] = false;  // robot
    obj_recv[3] = false;  // enemy goalpost
    obj_recv[4] = false;  // our goalpost
    for(int i=0 ; i<object_info.length ; i++)
    {
      if(strcmp(object_info.data[i].name.c_str(), "ball") == 0)
      {
        if(obj_recv[0])
        {
          if(obj[0].pos.x < object_info.data[i].pos.x)
            obj[0] = object_info.data[i];
        }
        else
        {
          obj[0] = object_info.data[i];
          obj_recv[0] = true;
        }
      }
      else if(strcmp(object_info.data[i].name.c_str(), "center") == 0)
      {
        if(obj_recv[1])
        {
          if(obj[1].pos.x < object_info.data[i].pos.x)
            obj[1] = object_info.data[i];
        }
        else
        {
          obj[1] = object_info.data[i];
          obj_recv[1] = true;
        }
      }
      else if(strcmp(object_info.data[i].name.c_str(), "robot") == 0)
      {
        if(obj_recv[2])
        {
          if(obj[2].pos.x < object_info.data[i].pos.x)
            obj[2] = object_info.data[i];
        }
        else
        {
          obj[2] = object_info.data[i];
          obj_recv[2] = true;
        }
      }
      else if(strcmp(object_info.data[i].name.c_str(), "goal") == 0)
      {
        if(obj_recv[3])
        {
          if(obj[3].pos.x < object_info.data[i].pos.x)
            obj[3] = object_info.data[i];
        }
        else
        {
          obj[3] = object_info.data[i];
          obj_recv[3] = true;
        }
      }
    }
    // update obj_lost time
    ros::Time temp = ros::Time::now();
    for(int i=0 ; i<5 ; i++)
    {
      if(obj_recv[i])
        obj_lost[i] = temp;
    }
  }

  void SetSignalTimer()
  {
    signal_timer = ros::Time::now() + cool_time;
  }

  void SetSignalTimer(float t)
  {
    signal_timer = ros::Time::now() + ros::Duration(t);
  }
};

Alice *alice;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "game_communicator_node");
  ros::NodeHandle nh;

  alice = new Alice(nh);

  while(ros::ok())
  {
    alice->Update();
    ros::spinOnce();
    usleep(200000);
  }
  delete alice;
}

void GameStateCallback(const alice_msgs::RoboCupGameControlData &msg)
{
  int8_t team_index = 0;

  if(msg.teams[team_index].team_number != alice->robot_info.team)
    team_index = 1;

  alice->game_state  = msg.state;
  alice->robot_state = msg.teams[team_index].players[alice->robot_info.player-1].penalty;

  alice->PubReturnData();
}

//void OnProcessCallback(const std_msgs::Bool &msg)
void OnProcessCallback(const robotis_controller_msgs::StatusMsg &msg)
{
  //alice->on_process = msg.data;
  if(strcmp(msg.status_msg.c_str(), "Walking_Started") == 0)
    alice->on_process = true;
  else if(strcmp(msg.status_msg.c_str(), "Walking_Finished") == 0)
  {
    alice->is_kicking = false;
    alice->on_process = false;
    alice->SetSignalTimer(0.6);
  }
}

void DetectedObjectsCallback(const alice_msgs::FoundObjectArray &msg)
{
  alice->object_info = msg;
}



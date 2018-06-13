#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>

#include <alice_msgs/RoboCupGameControlData.h>
#include <alice_msgs/RobotInfo.h>
#include <alice_msgs/TeamInfo.h>

#include "log_manager.h"
#include "robocup_struct.h"

void WriteControlData(Log *log, RoboCupGameControlData &control_data);
void WriteTeamData(Log *log, TeamInfo &team, int num);
void WriteRobotData(Log *log, RobotInfo &player);

void PrintControlData(RoboCupGameControlData &control_data);
void PrintTeamData(TeamInfo &team, int num);
void PrintRobotData(RobotInfo &player);

void PubControlData(ros::Publisher &pub_game_state, RoboCupGameControlData &control_data);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "referee_receiver_node");
  ros::NodeHandle nh;

  ros::Publisher pub_game_state = nh.advertise<alice_msgs::RoboCupGameControlData>("/heroehs/game_state", 1);

  ros::Time t = ros::Time::now();

  string path = "/home/alice1-nuke/logs/robocup_2018/referee";
  string date = GetYYMMDD();
  string seconds = GetSeconds(t.sec);
  //nh.getParam("path", path);
  path = path+"/"+date+"_referee_receiver_"+seconds+".txt";
  Log *log = new Log(path);

  int broadcast = 1;
  // make socket variable
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1)
    exit(1);

  // make variable for communicate
  sockaddr_in controller_addr;
  //bzero(&peer_addr, sizeof(peer_addr)); // set it 0
  controller_addr.sin_family = AF_INET;
  controller_addr.sin_port = htons(3838);
  controller_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // set broadcast ip(0.0.0.0)
  bind(sock, (sockaddr*)&controller_addr, sizeof(controller_addr));

  RoboCupGameControlData control_data; // buffer

  // make variable for communicate
  sockaddr_in client_addr; // sender's address
  socklen_t client_addr_len;

  int recv_len;

  while(ros::ok())
  {
    client_addr_len = sizeof(client_addr);
    recv_len = recvfrom(sock, (char*)&control_data, sizeof(control_data), 0, (sockaddr*)&client_addr, &client_addr_len);
    WriteControlData(log, control_data);
    //PrintControlData(control_data);
    PubControlData(pub_game_state, control_data);

    ros::spinOnce();
    usleep(10);
  }
  close(sock);

  return 0;
}

void PubControlData(ros::Publisher &pub_game_state, RoboCupGameControlData &control_data)
{
  alice_msgs::RoboCupGameControlData msg;

  msg.header            = control_data.header;
  msg.version           = control_data.version;
  msg.packet_number     = control_data.packetNumber;
  msg.players_per_team  = control_data.playersPerTeam;
  msg.competition_phase = control_data.competitionPhase;
  msg.competition_type  = control_data.competitionType;
  msg.game_phase        = control_data.gamePhase;
  msg.state             = control_data.state;
  msg.set_play          = control_data.setPlay;
  msg.first_half        = control_data.firstHalf;
  msg.kicking_team      = control_data.kickingTeam;
  msg.drop_in_team      = control_data.dropInTeam;
  msg.drop_in_time      = control_data.dropInTime;
  msg.secs_remaining    = control_data.secsRemaining;
  msg.secondary_time    = control_data.secondaryTime;

  for(int i=0 ; i<2 ; i++)
  {
    msg.teams.push_back(alice_msgs::TeamInfo());
    msg.teams[i].team_number  = control_data.teams[i].teamNumber;
    msg.teams[i].team_colour  = control_data.teams[i].teamColour;
    msg.teams[i].score        = control_data.teams[i].score;
    msg.teams[i].penalty_shot = control_data.teams[i].penaltyShot;
    msg.teams[i].single_shots = control_data.teams[i].singleShots;

    for(int j=0 ; j<control_data.playersPerTeam ; j++)
    {
      msg.teams[i].players.push_back(alice_msgs::RobotInfo());
      msg.teams[i].players[j].penalty               = control_data.teams[i].players[j].penalty;
      msg.teams[i].players[j].secs_till_unpenalised = control_data.teams[i].players[j].secsTillUnpenalised;
    }
  }

  pub_game_state.publish(msg);
}

void WriteControlData(Log *log, RoboCupGameControlData &control_data)
{
  ros::Time t = ros::Time::now();
  stringstream ss;
  ss << t.sec % (24*60*60) / (60*60) << ":"
     << t.sec % (24*60*60) / 60 % 60 << ":"
     << t.sec % (24*60*60) % 60 << "."
     << t.nsec << endl
     << control_data.header << "/"
     //<< (uint16_t)control_data.version << "/"
     << (int)control_data.packetNumber << "/"
     << (int)control_data.playersPerTeam << "/"
     << (int)control_data.competitionPhase << "/"
     << (int)control_data.competitionType << "/"
     << (int)control_data.gamePhase << "/"
     << (int)control_data.state << "/"
     << (int)control_data.setPlay << "/"
     << (int)control_data.firstHalf << "/"
     << (int)control_data.kickingTeam << "/"
     << (int)control_data.dropInTeam << "/"
     << (int)control_data.dropInTime << "/"
     << (int)control_data.secsRemaining << "/"
     << (int)control_data.secondaryTime << "/";
  log->Write(ss.str());

  for(int i=0 ; i<2 ; i++)
  {
    WriteTeamData(log, control_data.teams[i], (int)control_data.playersPerTeam);
  }
  log->Write("\n");
}

void WriteTeamData(Log *log, TeamInfo &team, int num)
{
  stringstream ss;
  ss << (int)team.teamNumber << "/"
     << (int)team.teamColour << "/"
     << (int)team.score << "/"
     << (int)team.penaltyShot << "/"
     << (int)team.singleShots << "/";
  log->Write(ss.str());

  for(int i=0 ; i<num ; i++)
  {
    WriteRobotData(log, team.players[i]);
  }
}

void WriteRobotData(Log *log, RobotInfo &player)
{
  stringstream ss;
  ss << (int)player.penalty << "/"
     << (int)player.secsTillUnpenalised << "/";
  log->Write(ss.str());
}

void PrintControlData(RoboCupGameControlData &control_data)
{
  //    cout << "           header : " << (string)control_data.header << endl;
  //    cout << "          version : " << (int)control_data.version << endl;             // version of the data structure
  cout << "    packet number : " << (int)control_data.packetNumber << endl;         // number incremented with each packet sent (with wraparound)
  //    cout << "  player per team : " << (int)control_data.playersPerTeam << endl;       // the number of players on a team
  //    cout << "competition phase : " << (int)control_data.competitionPhase << endl;     // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  //    cout << " competition type : " << (int)control_data.competitionType << endl;      // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_MIXEDTEAM, COMPETITION_TYPE_GENERAL_PENALTY_KICK)
  cout << "       game phase : " << (int)control_data.gamePhase << endl;            // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  cout << "            state : " << (int)control_data.state << endl;                // state of the game (STATE_READY, STATE_PLAYING, etc)
  //    cout << "         set play : " << (int)control_data.setPlay << endl;              // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_FREE_KICK, etc)
  cout << "       first harf : " << (int)control_data.firstHalf << endl;            // 1 = game in first half, 0 otherwise
  //    cout << "     kicking team : " << (int)control_data.kickingTeam << endl;          // the team number of the next team to kick off, free kick, DROPBALL etc.
  //    cout << "     drop in team : " << (int)control_data.dropInTeam << endl;           // number of team that caused last drop in
  //    cout << "     drop in time : " << (int)control_data.dropInTime << endl;          // number of seconds passed since the last drop in. -1 (0xffff) before first dropin
  cout << "   secs remaining : " << (int)control_data.secsRemaining << endl;       // estimate of number of seconds remaining in the half
  //    cout << "   secondary time : " << (int)control_data.secondaryTime << endl;       // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  cout << endl;
  cout << "---------------------------------------------------------" << endl;
  cout << endl;

  for(int i=0 ; i<2 ; i++)
  {
    cout << "[team " << i+1 << "]" << endl;
    PrintTeamData(control_data.teams[i], (int)control_data.playersPerTeam);
  }
  //cout << endl;
}

void PrintTeamData(TeamInfo &team, int num)
{
  cout << "      team number : " << (int)team.teamNumber << endl;
  //    cout << "      team colour : " << (int)team.teamColour << endl;           // colour of the team
  cout << "            score : " << (int)team.score << endl;                // team's score
  cout << "     penalty shot : " << (int)team.penaltyShot << endl;          // penalty shot counter
  cout << "      single shot : " << (int)team.singleShots << endl;         // bits represent penalty shot success
  cout << endl;

  for(int i=0 ; i<num ; i++)
  {
    cout << "[player " << i+1 << "]" << endl;
    PrintRobotData(team.players[i]);
  }
  cout << "---------------------------------------------------------" << endl;
  cout << endl;
}

void PrintRobotData(RobotInfo &player)
{
  cout << "            penalty : " << (int)player.penalty << endl;              // penalty state of the player
  cout << "secsTillUnpenalised : " << (int)player.secsTillUnpenalised << endl;  // estimate of time till unpenalised
  cout << endl;
}






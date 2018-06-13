#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string.h>
#include <time.h>

using namespace std;

string GetYYMMDD();
string GetSeconds(int32_t stamp);

class Log
{
  private:
    ofstream file;
    string file_name;

  public:
    Log();
    Log(string name);

    void Init(string name);
    void Write(string str);

    ~Log();
};

#endif

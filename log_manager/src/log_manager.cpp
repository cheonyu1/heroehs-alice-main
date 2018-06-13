#include "log_manager.h"

Log::Log(){}
Log::Log(string name)
{
  Init(name);
}

void Log::Init(string name)
{
  file_name = name;
  file.open(file_name.c_str());
}

void Log::Write(string str)
{
  file << str.c_str() << endl;
}

Log::~Log()
{
  file.close();
}

string GetYYMMDD()
{
  time_t cur_time = time(NULL);
  struct tm *t = localtime(&cur_time);

  char result[50];
  sprintf(result, "%02d%02d%02d", 
      t->tm_year-100, t->tm_mon+1, t->tm_mday);
  return result;
}

string GetSeconds(int32_t stamp)
{
  char result[50];
  sprintf(result, "%05d", stamp%86400);
  return result;
}

/*
   int main(int argc, char **argv)
   {
   }
 */

#pragma once

#include <string>
#include <vector>

double now();
void tempo(int ms);

struct TicToc
{
  TicToc(std::string name_):name(name_){}

  std::string name;
  double t=0;
  double total_time=0;
  double n=0;
  
  TicToc& tic();
  TicToc& toc();
  TicToc& disp();
  TicToc& toc_and_disp();
  double mean_time() const { return n==0 ? 0 : total_time/n; }
  double dtime();
};

void red();
void green();
void bold();
void color(bool b);
void reset_color();


struct Option
{
  std::vector<std::string> cmd;
  Option(int n, char** c)
  {
    for(int i = 1 ; i < n ; ++i)
    {
      cmd.push_back(std::string(c[i]));
    }
  }

  bool operator()(std::string opt) const
  {
    for(int i = 0 ; i < cmd.size() ; ++i)
    {
      if (cmd[i]==opt)
      {
        return true;
      }
    }
    return false;
  }
};
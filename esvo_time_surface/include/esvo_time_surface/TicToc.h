#ifndef esvo_tictoc_H_
#define esvo_tictoc_H_
//防止重复包含
#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace esvo_time_surface
//使用命名空间
{
class TicToc
{
  public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  //chrono库：https://www.cnblogs.com/zlshmily/p/10058427.html
  //tic初始化start 用的system_clock
  //做差结果保存到duration<double> 
};
}

#endif // esvo_tictoc_H_

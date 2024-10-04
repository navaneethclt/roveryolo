#include <chrono>
#include <thread>
#include <opencv2/dnn.hpp>
#include "pch.h"
#include "ArduSerial.h"
#include <iostream>


class loopread
{
public:
    loopread();


    int getread();
    int getstatus();
    double getreadf();
    double getreadm();


    void start();
    double findMedian(std::vector<int>& nums);


private:
    void mainloop();

    std::mutex mutex_lock_;
    int lidar_ = 0, lidarf_=0,status_=0;
    double lidarm_=0;
    std::vector<int> values_ = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
}; 


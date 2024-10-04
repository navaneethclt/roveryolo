#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>







class cameraThread
{
public:
    cameraThread(int device_id = 0);
   
    void start();
    cv::Mat* getLatestFrame();

private:
    void mainloop();
      
    cv::VideoCapture cap_;
    cv::Mat image_;
    std::mutex mutex_lock_;
};
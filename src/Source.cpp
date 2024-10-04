
#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif


#include "Iir.h" //filter design

#include <iostream>
#include <opencv2/opencv.hpp>
#include "camerathread.h"
#include "cmdline.h"
#include "utilsm.h"
#include "detector.h"

#include "pch.h"
#include "ArduSerial.h"


#include <conio.h>
#include <stdio.h>
#include <wtypes.h>
#include <winbase.h>
#include <math.h>
#include <time.h>
#include <mmsystem.h>


#include<vector> 
#include <unordered_map>

#include "loopread.h"

constexpr auto SAMPLE_RATE = 28.0;
#define PERFORMANCE_COUNTS  10000000

const int order = 6;
Iir::Butterworth::LowPass<order> f0;
std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}
void sleepfor(LARGE_INTEGER from, long performance_clicks)
{
    LARGE_INTEGER pc0;
    long pc_diff;

    QueryPerformanceCounter(&pc0);
    pc_diff = (long)pc0.QuadPart - (long)from.QuadPart;

    if (pc_diff > performance_clicks)
        printf(".");

    while (pc_diff < performance_clicks) {
        QueryPerformanceCounter(&pc0);
        pc_diff = (long)pc0.QuadPart - (long)from.QuadPart;
    }

    return;
}

int findMode(const std::vector<int>& arr) {
    std::unordered_map<int, int> freqMap;
    int maxFreq = 0;
    int mode = 0;

    for (int num : arr) {
        freqMap[num]++;
        if (freqMap[num] > maxFreq) {
            maxFreq = freqMap[num];
            mode = num;
        }
    }

    return mode;
}
double findMedian(std::vector<int>& nums) {
    std::sort(nums.begin(), nums.end());

    int n = nums.size();
    if (n % 2 == 0) {
        int middle1 = n / 2 - 1;
        int middle2 = n / 2;
        return (nums[middle1] + nums[middle2]) / 2.0;
    }
    else {
        int middle = n / 2;
        return nums[middle];
    }
}
void setup() //setting up the arduino connection
{



    Serial4.begin(9600);





    std::cout << "Connected" << std::endl;


}

void loop3(int steps) // input number of steps to travel
{

    Serial4.println(std::to_string(steps));

}
double loop4read() // input number of steps to travel
{
    std::string line;
    int lidar;
    std::vector<int> vec;
        while (Serial4.available())
    {
        char data = Serial4.read();

        // Check for newline character
        if (data == '\n')
        {
            // Process the received line
          // std::cout << "Received line: " << line << std::endl;

            // Reset the line buffer
            try {
                lidar = std::stoi(line);
                vec.insert(vec.begin(), lidar);
           
            }
            catch (std::exception& err)
            {
                std::cout << "Conversion failure: " << err.what() << std::endl; // Note: what() tells the exact error
                std::cout << lidar << std::endl;
            }
            line.clear();

        }
        else
        {
            // Append character to the line
           // std::cout << "Received line: " << line << std::endl;

            line += data;
        }
    }
    return findMedian(vec);
}

int main(int argc, char* argv[])
{
    double cutoff_freq = 14.0;
    double sampling_rate = 30.0;
    f0.setup(sampling_rate, cutoff_freq);

    double filtptcx[50000];
    char key = 0;
    long sample_period;
    LARGE_INTEGER hr0;
    double elapsed_time, freq;
    clock_t start2, stop2;
    sample_period = PERFORMANCE_COUNTS / SAMPLE_RATE;
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

    start2 = clock();
    QueryPerformanceCounter(&hr0);

  //  loopread loop4;
   // loop4.start();

    const float confThreshold = 0.6f;
    const float iouThreshold = 0.4f;

    cmdline::parser cmd;
    cmd.add<std::string>("model_path", 'm', "Path to onnx model.", true, "yolov5.onnx");
    cmd.add<std::string>("image", 'i', "Image source to be detected.", true, "bus.jpg");
    cmd.add<std::string>("class_names", 'c', "Path to class names file.", true, "coco.names");
    cmd.add("gpu", '\0', "Inference on cuda device.");

    cmd.parse_check(argc, argv);

    bool isGPU = cmd.exist("gpu");
    const std::string classNamesPath = cmd.get<std::string>("class_names");
    const std::vector<std::string> classNames = utilsm::loadNames(classNamesPath);
    const std::string imagePath = cmd.get<std::string>("image");
    const std::string modelPath = cmd.get<std::string>("model_path");

    if (classNames.empty())
    {
        std::cerr << "Error: Empty class names file." << std::endl;
        return -1;
    }

    YOLODetector detector{ nullptr };
    cv::Mat image;
    std::vector<Detection> result;

    setup();
    FILE* fmaxtheta = fopen("yolov7_ver78.csv", "wb");//opening the file
    auto start = std::chrono::high_resolution_clock::now();
    long long prev_ms = 0;
    float fps = 0;

    try
    {
        detector = YOLODetector(modelPath, isGPU, cv::Size(640, 640));
        std::cout << "Model was initialized." << std::endl;
        cameraThread camera_th(0);
        camera_th.start();
        std::cout << "starting loop" << std::endl;
        auto frame = camera_th.getLatestFrame();
        int count = 999;
        double Theta = 0, Thetaf = 0;
        int go;
        double lidar = 0;
        double lidarm=0, lidarf=0;
        int lidarint;
        double ptcf;
        while (true)
        {
            double timer = (double)cv::getTickCount();
            count++;

            frame = camera_th.getLatestFrame();
            image = *frame;


            result = detector.detect(image, confThreshold, iouThreshold);
            int flag=0;
            double cinput = 0.0;

            cinput = utilsm::visualizeDetection(image, result, classNames, &flag,count);
            //std::cout << cinput << "\n";
            if (flag == 0) {
                std::cout << "detect error" << "\n";

                imwrite("D:/lisec/rover/Project4/ConsoleApplication1/img27/frame" + std::to_string(count) + ".png", image);

                /* std::string ty = type2str(image.type());
                 printf("Matrix: %s %dx%d \n", ty.c_str(), image.cols, image.rows);

                 auto image2 = cv::imread("D:/lisec/rover/Project4/ConsoleApplication1/img26/frame" + std::to_string(count) + ".png");

                 ty = type2str(image2.type());
                 printf("Matrix: %s %dx%d \n", ty.c_str(), image2.cols, image2.rows);


                 auto imgsub = image - image2;
                  cv::imshow("result", imgsub);
            cv::waitKey(1);
            imwrite("D:/lisec/rover/Project4/ConsoleApplication1/img26/frame" + std::to_string(count) + ".png", imgsub);*/

            }

            // cinput = 0;
              // auto cinput = -((ptc_x - 306) * 15) / 640.0; //36.78
            //lidar = loop4read();
           
            
            /*lidar = loop4.getread();
            lidarm = loop4.getreadm();
            lidarf = loop4.getreadf();*/

           // std::cout << lidar <<" "<<loop4.getstatus()<< std::endl;


            if (count % 1 == 0)
            {
                if (abs(cinput) > 1.0) { cinput = cinput / abs(cinput); }
                Theta = Thetaf + cinput;
                Thetaf = f0.filter(Theta);
                go = int(Thetaf* 3600 / 360);
                loop3(go);
            }
            std::cout << lidarm <<"\t"<<Theta<<std::endl;

            imwrite("D:/lisec/rover/Project4/ConsoleApplication1/img78/frame" + std::to_string(count) + ".jpg", image);
            auto elapsed = std::chrono::high_resolution_clock::now() - start;
            long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            long long time_diff = microseconds - prev_ms;
            prev_ms = microseconds;

            fprintf(fmaxtheta, "%d", count);//1
            fprintf(fmaxtheta, ",");

            fprintf(fmaxtheta, "%e", Theta);//2
            fprintf(fmaxtheta, ",");

            fprintf(fmaxtheta, "%e", Thetaf);//3
            fprintf(fmaxtheta, ",");


            fprintf(fmaxtheta, "%e", fps);//4
            fprintf(fmaxtheta, ",");

            fprintf(fmaxtheta, "%e", lidar);//5
            fprintf(fmaxtheta, ",");

            fprintf(fmaxtheta, "%e", lidarf);//6
            fprintf(fmaxtheta, ",");


            fprintf(fmaxtheta, "%e", lidarm);//7
            fprintf(fmaxtheta, ",");

            fprintf(fmaxtheta, "%e", cinput);//8
            fprintf(fmaxtheta, ",");


            fprintf(fmaxtheta, "%llu", microseconds);//9
            fprintf(fmaxtheta, "\n");



            // float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
             //putText(image, "FPS : " + std::to_string(int(fps)), cv::Point(100, 50),
              //   cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(200, 250, 200), 2);

     /*cv::imshow("result", image);
         cv::waitKey(1);*/
            fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

            sleepfor(hr0, sample_period);
            QueryPerformanceCounter(&hr0); // counts from right NOW

        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "here  " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

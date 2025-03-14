#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "Iir.h"           // Filter design
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
#include <vector> 
#include <unordered_map>
#include "loopread.h"

// Sampling rate constant
constexpr auto SAMPLE_RATE = 28.0;
#define PERFORMANCE_COUNTS  10000000

// Butterworth low-pass filter setup
const int order = 6;
Iir::Butterworth::LowPass<order> f0;

// Function to determine the data type of an OpenCV matrix
std::string type2str(int type) {
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    std::string r;

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

    r += "C" + std::to_string(chans);
    return r;
}

// Function to create a sleep delay using performance counters
void sleepfor(LARGE_INTEGER from, long performance_clicks) {
    LARGE_INTEGER pc0;
    long pc_diff;
    QueryPerformanceCounter(&pc0);
    pc_diff = static_cast<long>(pc0.QuadPart) - static_cast<long>(from.QuadPart);

    if (pc_diff > performance_clicks) {
        std::cout << ".";
    }

    while (pc_diff < performance_clicks) {
        QueryPerformanceCounter(&pc0);
        pc_diff = static_cast<long>(pc0.QuadPart) - static_cast<long>(from.QuadPart);
    }
}

// Function to find the mode of an integer array
int findMode(const std::vector<int>& arr) {
    std::unordered_map<int, int> freqMap;
    int maxFreq = 0, mode = 0;

    for (int num : arr) {
        freqMap[num]++;
        if (freqMap[num] > maxFreq) {
            maxFreq = freqMap[num];
            mode = num;
        }
    }
    return mode;
}

// Function to find the median of an integer array
double findMedian(std::vector<int>& nums) {
    std::sort(nums.begin(), nums.end());
    int n = nums.size();
    
    if (n % 2 == 0) {
        int middle1 = n / 2 - 1;
        int middle2 = n / 2;
        return (nums[middle1] + nums[middle2]) / 2.0;
    } else {
        return nums[n / 2];
    }
}

// Serial communication setup function
void setup() {
    Serial4.begin(9600);
    std::cout << "Connected" << std::endl;
}

// Function to send step count via serial communication
void loop3(int steps) {
    Serial4.println(std::to_string(steps));
}

// Function to read lidar data via serial communication
double loop4read() {
    std::string line;
    int lidar;
    std::vector<int> vec;

    while (Serial4.available()) {
        char data = Serial4.read();

        if (data == '\n') {
            try {
                lidar = std::stoi(line);
                vec.insert(vec.begin(), lidar);
            } catch (const std::exception& err) {
                std::cout << "Conversion failure: " << err.what() << std::endl;
            }
            line.clear();
        } else {
            line += data;
        }
    }
    return findMedian(vec);
}

// Main function
int main(int argc, char* argv[]) {
    double cutoff_freq = 14.0, sampling_rate = 30.0;
    f0.setup(sampling_rate, cutoff_freq);

    double filtptcx[50000];
    char key = 0;
    long sample_period = PERFORMANCE_COUNTS / SAMPLE_RATE;
    LARGE_INTEGER hr0;
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

    clock_t start2 = clock();
    QueryPerformanceCounter(&hr0);

    // Command line parser for model and image input
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

    if (classNames.empty()) {
        std::cerr << "Error: Empty class names file." << std::endl;
        return -1;
    }

    // YOLO detector setup
    YOLODetector detector{nullptr};
    cv::Mat image;
    std::vector<Detection> result;

    setup();
    FILE* fmaxtheta = fopen("yolov7_ver78.csv", "wb");

    try {
        detector = YOLODetector(modelPath, isGPU, cv::Size(640, 640));
        std::cout << "Model was initialized." << std::endl;
        cameraThread camera_th(0);
        camera_th.start();
        std::cout << "starting loop" << std::endl;

        long long prev_ms = 0;
        float fps = 0;
        int count = 999;
        double Theta = 0, Thetaf = 0;
        int go;

        while (true) {
            double timer = static_cast<double>(cv::getTickCount());
            count++;

            auto frame = camera_th.getLatestFrame();
            image = *frame;
            result = detector.detect(image, 0.6f, 0.4f);

            int flag = 0;
            double cinput = utilsm::visualizeDetection(image, result, classNames, &flag, count);

            if (count % 1 == 0) {
                Theta = Thetaf + cinput;
                Thetaf = f0.filter(Theta);
                go = int(Thetaf * 3600 / 360);
                loop3(go);
            }

            fprintf(fmaxtheta, "%d,%e,%e,%e,%llu\n", count, Theta, Thetaf, fps, prev_ms);
            sleepfor(hr0, sample_period);
            QueryPerformanceCounter(&hr0);
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

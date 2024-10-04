



#include"loopread.h"
#include "Iir.h" //filter design
#include <iostream>


#include <stdlib.h>
#include <stdio.h>



Iir::Butterworth::LowPass<5> f4;



loopread::loopread()
{

    Serial4.begin(115200);
    std::cout << "Connected" << std::endl;
    f4.setup(300, 30);
    
   




}


double loopread::findMedian(std::vector<int>& nums) {
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



void loopread::start()
{
    std::thread thread(&loopread::mainloop, this);
    thread.detach();
}
int loopread::getread()
{
    std::lock_guard<std::mutex> lock(mutex_lock_);

    return this->lidar_;
}
int loopread::getstatus()
{
    std::lock_guard<std::mutex> lock(mutex_lock_);

    int tempstatus = status_;
    status_ = 0;
    return tempstatus;
}
double loopread::getreadm()
{
    std::lock_guard<std::mutex> lock(mutex_lock_);

    return this->lidarm_;
}
double loopread::getreadf()
{
    std::lock_guard<std::mutex> lock(mutex_lock_);

    return this->lidarf_;
}

void loopread::mainloop()
{
    std::string line;
    std::vector<int> vec;
    while (true) {



        if (Serial4.available())
        {
            status_ = 1;

            char data = Serial4.read();
            // Check for newline character
            if (data == '\n')
            {
                // Process the received line
              // std::cout << "Received line: " << line << std::endl;

                // Reset the line buffer
                this->mutex_lock_.lock();

                try {
                  
                    this->lidar_ = std::stoi(line);
                   this->lidarf_ = f4.filter(this->lidar_);
                   values_.erase(values_.begin());
                   values_.push_back(lidar_);
                   std::vector<int> vect2(values_);
                   lidarm_= this->findMedian(vect2);

                  /* copy(values_.begin(),
                       values_.end(),
                       std::ostream_iterator<int>(std::cout, " "));
                   std::cout << std::endl; */
                   // std::cout << this->lidar_ << std::endl;
                    // vec.insert(vec.begin(), lidar);

                }
                catch (std::exception& err)
                {
                    //do nothing
                }
                line.clear();
                this->mutex_lock_.unlock();


            }
            else
            {
                // Append character to the line
               // std::cout << "Received line: " << line << std::endl;

                line += data;

            }
        }
    }
}

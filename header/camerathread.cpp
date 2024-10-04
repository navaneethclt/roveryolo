#include"camerathread.h"





cameraThread::cameraThread(int device_id)
    {

        cap_.open(device_id);
        if (!this->cap_.isOpened()) {
            std::cerr << "Unable to open camera\n";
            exit(EXIT_FAILURE);
        }

        this->cap_.read(this->image_);
        if (this->image_.empty()) {
            std::cerr << "empty frame\n";
            return;
        }
        else {
            std::cout << "got frame" << std::endl;
        }
    }

    void cameraThread::start()
    {
        std::thread thread(&cameraThread::mainloop, this);
        thread.detach();
    }
    cv::Mat* cameraThread::getLatestFrame()
    {
        return &this->image_;
    }


    void cameraThread::mainloop()
    {
        while (cap_.isOpened()) {


            this->mutex_lock_.lock();
            cap_.grab();
            cap_ >> image_;

            this->mutex_lock_.unlock();

        }
    }
    
#include "utilsm.h"


size_t utilsm::vectorProduct(const std::vector<int64_t>& vector)
{
    if (vector.empty())
        return 0;

    size_t product = 1;
    for (const auto& element : vector)
        product *= element;

    return product;
}

std::wstring utilsm::charToWstring(const char* str)
{
    typedef std::codecvt_utf8<wchar_t> convert_type;
    std::wstring_convert<convert_type, wchar_t> converter;

    return converter.from_bytes(str);
}

std::vector<std::string> utilsm::loadNames(const std::string& path)
{
    // load class names
    std::vector<std::string> classNames;
    std::ifstream infile(path);
    if (infile.good())
    {
        std::string line;
        while (getline (infile, line))
        {
            if (line.back() == '\r')
                line.pop_back();
            classNames.emplace_back(line);
        }
        infile.close();
    }
    else
    {
        std::cerr << "ERROR: Failed to access class name path: " << path << std::endl;
    }

    return classNames;
}


double utilsm::visualizeDetection(cv::Mat& image, std::vector<Detection>& detections,
                               const std::vector<std::string>& classNames,int* flag=0, int fnum=0)
{
    double cinput = 0.0;
    cv::Mat gray, image_c;
    double ptc_x, ptc_y;
    int classsum = 0;
   //auto annotationPath = "D:/lisec/rover/Project4/ConsoleApplication1/img66/frame" + std::to_string(fnum) + ".txt";
   // std::ofstream annotationFile(annotationPath);

    for (const Detection& detection : detections)
    {

       // cv::rectangle(image, detection.box, cv::Scalar(229, 160, 21), 2);

        int x = detection.box.x;
        int y = detection.box.y;
        int w = detection.box.width;
        int h = detection.box.height;


        int conf = (int)std::round(detection.conf * 100);
        int classId = detection.classId;
        classsum = classsum + classId;
                  //  annotationFile << classId << " " << (x + w / 2) / 640.0 << " " << (y + h / 2) / 480.0 << " " << w / 640.0 << " " << h / 480.0 << std::endl;

        if(classId ==1){
            cv::Point pt(x+w/2,y+h/2);


            //image_c = image(detection.box);
            //cv::cvtColor(image_c, gray, cv::COLOR_BGRA2GRAY);    // Blur the image to reduce noise 
            //cv::Mat img_blur;
            //medianBlur(gray, img_blur, 5);
            //// Create a vector for detected circles
            //std::vector<cv::Vec3f>  circles;
            //// Apply Hough Transform
            //HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2, 100, 50, 10, 10, 25);
            //if (circles.size() > 0) {

            //    for (size_t i = 0; i < circles.size(); i++) {
            //        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            //        int radius = cvRound(circles[i][2]);
            //        cv::Point pt(x, y);
            //        //std::cout << pt + center << "\n";

            //        //circle(image, pt + center, radius, cv::Scalar(255, 255, 255), 2, 8, 0);
            //        auto ptc = pt + center;
            //         ptc_x = ptc.x;
            //         ptc_y = ptc.y;

            //    }
            //}
          // cv::circle(image, pt, 0, cv::Scalar(0, 0, 0), 10, 8, 0);
            ptc_x = pt.x;

            cinput = -((ptc_x - 293)* 15 )/ 640.0; //36.78//15//293//299
           // cinput = ptc_x;
          //  std::cout << x + (w / 2) <<',' <<cinput<< std::endl;



        }

       /*std::string label = classNames[classId] + " 0." + std::to_string(conf);

        int baseline = 0;
        cv::Size size = cv::getTextSize(label, cv::FONT_ITALIC, 0.8, 2, &baseline);
        cv::rectangle(image,
                      cv::Point(x, y - 25), cv::Point(x + size.width, y),
                      cv::Scalar(classId*229, (1- classId)*229, 21), -1);

        cv::putText(image, label,
                    cv::Point(x, y - 3), cv::FONT_ITALIC,
                    0.8, cv::Scalar(255, 255, 255), 2);*/
    }
    *flag = classsum;
    return cinput;
}

void utilsm::letterbox(const cv::Mat& image, cv::Mat& outImage,
                      const cv::Size& newShape = cv::Size(640, 640),
                      const cv::Scalar& color = cv::Scalar(114, 114, 114),
                      bool auto_ = true,
                      bool scaleFill = false,
                      bool scaleUp = true,
                      int stride = 32)
{
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);

    float ratio[2] {r, r};
    int newUnpad[2] {(int)std::round((float)shape.width * r),
                     (int)std::round((float)shape.height * r)};

    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    if (auto_)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        newUnpad[0] = newShape.width;
        newUnpad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
    {
        cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void utilsm::scaleCoords(const cv::Size& imageShape, cv::Rect& coords, const cv::Size& imageOriginalShape)
{
    float gain = std::min((float)imageShape.height / (float)imageOriginalShape.height,
                          (float)imageShape.width / (float)imageOriginalShape.width);

    int pad[2] = {(int) (( (float)imageShape.width - (float)imageOriginalShape.width * gain) / 2.0f),
                  (int) (( (float)imageShape.height - (float)imageOriginalShape.height * gain) / 2.0f)};

    coords.x = (int) std::round(((float)(coords.x - pad[0]) / gain));
    coords.y = (int) std::round(((float)(coords.y - pad[1]) / gain));

    coords.width = (int) std::round(((float)coords.width / gain));
    coords.height = (int) std::round(((float)coords.height / gain));

    // // clip coords, should be modified for width and height
    // coords.x = utils::clip(coords.x, 0, imageOriginalShape.width);
    // coords.y = utils::clip(coords.y, 0, imageOriginalShape.height);
    // coords.width = utils::clip(coords.width, 0, imageOriginalShape.width);
    // coords.height = utils::clip(coords.height, 0, imageOriginalShape.height);
}

template <typename T>
T utilsm::clip(const T& n, const T& lower, const T& upper)
{
    return std::max(lower, std::min(n, upper));
}

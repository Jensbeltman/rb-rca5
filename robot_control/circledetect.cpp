#include "circledetect.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include "buffer.h"
#include <iostream>
#include <math.h>

circleDetect::circleDetect()
{
    //circBuffer.setSize(25);
    //circBuffer.setLowerB(10);
}

void circleDetect::detect(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
    cv::cvtColor(im, im, CV_RGB2BGR);

    cv::cvtColor(im, im, CV_BGR2HLS);
    cv::Mat hlsChannels[3];
    cv::split(im, hlsChannels);

    cv::Mat binSat;
    cv::threshold(hlsChannels[2], binSat, 125, 255, cv::THRESH_BINARY);
    cv::GaussianBlur(binSat, binSat, cv::Size(9, 9), 2, 2);

    /*
    cv::Mat edges, dx, dy;
    int kernelSize = 3;
    int cannyThreshold = 50;
    cv::Sobel(binSat, dx, CV_16S, 1, 0, kernelSize, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(binSat, dy, CV_16S, 0, 1, kernelSize, 1, 0, cv::BORDER_REPLICATE);
    cv::Canny(dx, dy, edges, std::max(1, cannyThreshold / 2), cannyThreshold, false);
    */


    std::vector<cv::Vec3f> circs;
    cv::HoughCircles(binSat, circs, cv::HOUGH_GRADIENT, 0.8, im.rows/8, 50, 25, 0, 0);

    for (size_t i = 0; i < circs.size(); i++) {
        cv::Point center(cvRound(circs[i][0]), cvRound(circs[i][1]));
        int radius = cvRound(circs[i][2]);
        //Center
        circle(binSat, center, 3, cv::Scalar(150), -1, 8, 0);
        //Outline
        circle(binSat, center, radius, cv::Scalar(150), 3, 8, 0);
    }

    // Draw center point on image
    int center_x = im.cols/2;
    int center_y = im.rows/2;
    circle(binSat, cv::Point(center_x, center_y), 2, cv::Scalar(150), -1, 8, 0);


    cv::imshow("", binSat);
}

int circleDetect::search(cv::Mat & in_img)
{
    cv::Mat grey_img = in_img.clone();
    cv::cvtColor(grey_img, grey_img, CV_RGB2GRAY);

    cv::GaussianBlur(grey_img, grey_img, cv::Size(9, 9), 2, 2);


    //cv::Mat hlsImg = in_img.clone();
    //cv::cvtColor(in_img, hlsImg, CV_RGB2HLS);

    //cv::Mat hlsCh[3];
    //cv::split(hlsImg, hlsCh);

    //cv::Mat binImg;

    //cv::threshold(hlsCh[2], binImg, 120, 255, cv::THRESH_BINARY);

    //cv::GaussianBlur(binImg, binImg, cv::Size(9, 9), 2, 2);



    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grey_img, circles, cv::HOUGH_GRADIENT, 1, in_img.rows/8, 50, 25, 0, 0);

    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //Center
        circle(in_img, center, 3, cv::Scalar(0, 0, 150), -1, 8, 0);
        //Outline
        circle(in_img, center, radius, cv::Scalar(0, 0, 150), 3, 8, 0);
    }


    // Draw center point on image
    int center_x = in_img.cols/2;
    int center_y = in_img.rows/2;
    circle(in_img, cv::Point(center_x, center_y), 2, cv::Scalar(255, 255, 255), -1, 8, 0);


    circles.size() ? circBuffer.addToBuf(true) : circBuffer.addToBuf(false);
    //std::cout << circBuffer.size() << " - " << circBuffer.isEmpty() << std::endl;

    float length = 0;
    if (!circBuffer.isEmpty() && circles.size()) {
        // Calculate distance to center
        cv::Vec3f largestC = {0, 0, 0};
        for (int i = 0; i < circles.size(); i++) {
            largestC = (circles[i][2] > largestC[2]) ? circles[i] : largestC;
        }
        cv::line(in_img, cv::Point(center_x, center_y), cv::Point(largestC[0], largestC[1]), cv::Scalar(0, 200, 0), 1, 8, 0);
        length = (largestC[0]-center_x > 0) ? std::sqrt(std::pow(largestC[0]-center_x, 2) + std::pow(largestC[1]-center_y, 2)) : -1*std::sqrt(std::pow(largestC[0]-center_x, 2) + std::pow(largestC[1]-center_y, 2));
    }
    //std::cout << length << std::endl;
    return length;
}

int circleDetect::getAmountBlue(cv::Mat img)
{
    //Make a histogram to determine the amount of blue in the picture.
    cv::Mat hlsImg = img.clone();
    cv::cvtColor(img, hlsImg, CV_BGR2HLS);


    //Splitting the hls to see the individual channels.
    cv::Mat hlsCh[3];
    cv::split(img, hlsCh);

    cv::Mat binImg;

    cv::threshold(hlsCh[2], binImg, 120, 255, cv::THRESH_BINARY);

    cv::GaussianBlur(binImg, binImg, cv::Size(9, 9), 2, 2);

    //cv::imshow("", hlsCh[0]);
    cv::Mat histhalloj;
    float ranges[] = {0, 256};
    int histsize[] = {20};
    int channels[] = {0};
    const float *p_ranges[] = {ranges};
    cv::calcHist(&hlsImg, 1, channels, cv::Mat(), histhalloj, 1, histsize, p_ranges, true);
    cv::imshow("", hlsCh[0]);
    std::cout << "Hej " << histhalloj << std::endl;

    int histogram[256] = {0};
    for (int i = 0; i < hlsImg.rows; i++) {
        for (int j = 0; j < hlsImg.cols; j++) {
            histogram[hlsImg.at<cv::Vec3b>(i, j)[1]] += 1;//2 shows the presence of colour very well. 1 looks more cool.
            //histogram[binImg.at<cv::Vec3b>(i, j)[0]] += 1;
        }
    }
    cv::Mat hist = cv::Mat::zeros(1000, 500, 0);
    for (int i = 0; i < 256; i++) {
        cv::line(hist, cv::Point(i, 0), cv::Point(i, histogram[i]), cv::Scalar(255), 1, 8, 0);
    }
    cv::Mat flip;
    cv::flip(hist, flip, 0);

    int avg_blue = 0;
    for (int i = 40; i < 51; i++) {
        avg_blue += histogram[i];
    }
    //std::cout << avg_blue/10 << std::endl;
    //cv::imshow("", flip);
    return avg_blue/10;
}

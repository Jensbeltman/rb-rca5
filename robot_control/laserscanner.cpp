#include "laserscanner.h"

#include <math.h>
#include <iostream>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>

laserscanner::laserscanner()
{

}

void laserscanner::performScan(ConstLaserScanStampedPtr &msg)
{
    nranges = msg->scan().ranges_size();
    angle_step = msg->scan().angle_step();
    smallest_dist = msg->scan().ranges(0);
    smallest_dir = msg->scan().angle_min();
    new_angle = smallest_dir;

    for (int i = 0; i < nranges; i++) {
        smallest_dist = (smallest_dist < msg->scan().ranges(i)) ? smallest_dist : msg->scan().ranges(i);
        new_angle = new_angle + angle_step;
        smallest_dir = (smallest_dist < msg->scan().ranges(i)) ? smallest_dir : new_angle;
    }
    scan.direction = smallest_dir;
    scan.distance = smallest_dist;
}

closestScan laserscanner::getClosestScan()
{
    return scan;
}

static boost::mutex mutex;
void testDraw(ConstLaserScanStampedPtr &msg, std::vector<std::pair<float, float>> startpoints, std::vector<std::pair<float, float>> endpoints) {
    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;

    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);
    for (int i = 0; i < nranges; i++) {
      float angle = angle_min + i * angle_increment;
      float range = std::min(float(msg->scan().ranges(i)), range_max);
      //    double intensity = msg->scan().intensities(i);
      cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                          200.5f - range_min * px_per_m * std::sin(angle));
      cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                        200.5f - range * px_per_m * std::sin(angle));
      cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
               cv::LINE_AA, 4);

      //    std::cout << angle << " " << range << " " << intensity << std::endl;
    }
    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

    for (int i = 0; i < startpoints.size(); i++) {
        cv::line(im, cv::Point(200+startpoints[i].first*px_per_m, 200-startpoints[i].second*px_per_m), cv::Point(200+endpoints[i].first*px_per_m, 200-endpoints[i].second*px_per_m), cv::Scalar(0, 0, 200), 2, cv::LINE_8);
    }

    mutex.lock();
    cv::imshow("Lidar", im);
    mutex.unlock();
}

void laserscanner::findLines(ConstLaserScanStampedPtr &msg)
{
    std::vector<std::pair<float, float>> startPoints;
    std::vector<std::pair<float, float>> endPoints;
    std::vector<std::pair<float, float>> oToStart;
    std::vector<std::pair<float, float>> oToEnd;


    std::vector<std::pair<float, float>> cartCoordinates;
    int amountScans = msg->scan().ranges_size();
    float angle = msg->scan().angle_min();
    float angleInc = msg->scan().angle_step();
    for (int i = 0; i < amountScans; i++) {
        if (! isinf(msg->scan().ranges(i))) {
            float range = msg->scan().ranges(i);
            cartCoordinates.push_back(std::make_pair(range * std::cos(angle), range * std::sin(angle)));
        }
        angle += angleInc;
    }


    int npoints = cartCoordinates.size();
    std::cout << "npoints " << npoints << std::endl;

    //Delete the previous lines.
    ransacLines.erase(ransacLines.begin(), ransacLines.end());


    //Following needs to be done N times. (Number of points?)
    for (int find = 0; find < npoints; find++) {

        int point1;
        int point2;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, npoints - 1);
        while(true) {
            point1 = dis(gen);
            point2 = dis(gen);
            if (point1 != point2)
                break;
        }

        //std::cout << point1 << ", " << point2 << ", " << amountScans-1 << std::endl;

        float x1 = cartCoordinates[point1].first;
        float y1 = cartCoordinates[point1].second;
        float x2 = cartCoordinates[point2].first;
        float y2 = cartCoordinates[point2].second;

        float threshold = 0.005f;
        std::vector<std::pair<float, float>> keep;
        std::vector<int> keepIndex;
        float distance = 0;
        int current_coordinates = cartCoordinates.size();


        for (int i = 0; i < current_coordinates; i++) {
            if (i != point1 && i != point2) {
                distance = std::abs(((y2 - y1)*cartCoordinates[i].first) - ((x2 - x1)*cartCoordinates[i].second) + (x2*y1) - (y2*x1)) / std::sqrt(std::pow(y2-y1, 2)+std::pow(x2-x1, 2));
                //std::cout << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ", " << distance << std::endl;
            }

            if (distance < threshold || i == point1 || i == point2) {
                //Keep
                keep.push_back(std::make_pair(cartCoordinates[i].first, cartCoordinates[i].second));
                keepIndex.push_back(i);
            }
        }

        int lineThresh = 20;
        //std::cout << keep.size() << std::endl;

        if (keep.size() >= lineThresh) {
            //Do least squares of keep and push line parameters to ransacLines.
            float x_bar = 0;
            float y_bar = 0;
            float xy_bar = 0;
            float xsq_bar = 0;
            float param_a = 0;
            float param_b = 0;

            for (int i = 0; i < keep.size(); i++) {
                x_bar += keep[i].first;
                y_bar += keep[i].second;
                xy_bar += keep[i].first*keep[i].second;
                xsq_bar += std::pow(keep[i].first, 2);
                if (i == 0)
                    startPoints.push_back(std::make_pair(keep[i].first, keep[i].second));
                if (i == keep.size() - 1)
                    endPoints.push_back(std::make_pair(keep[i].first, keep[i].second));
                cartCoordinates.erase(cartCoordinates.begin() + keepIndex[i]);
            }
            x_bar /= keep.size();
            y_bar /= keep.size();
            xy_bar /= keep.size();
            xsq_bar /= keep.size();

            param_a = (xy_bar - x_bar*y_bar) / (xsq_bar - std::pow(x_bar, 2));
            param_b = (xsq_bar * y_bar - x_bar * xy_bar) / (xsq_bar - std::pow(x_bar, 2));
            ransacLines.push_back(std::make_pair(param_a, param_b));
            //std::cout << "y = " << param_a << "x + " << param_b << std::endl;
        }

        if (cartCoordinates.size() < lineThresh)
            break;
    }

    if (ransacLines.empty())
        ransacLines.push_back(std::make_pair(0, 0));
    for (int i = 0; i < ransacLines.size(); i++) {
        std::cout << "y = " << ransacLines[i].first << "x + " << ransacLines[i].second << std::endl;
    }
    for (int i = 0; i < startPoints.size(); i++) {
        std::cout << "Start, " << startPoints[i].first << ", " << startPoints[i].second << std::endl;
        std::cout << "End, " << endPoints[i].first << ", " << endPoints[i].second << std::endl;
    }

    for (int i = 0; i < startPoints.size(); i++) {

    }

    testDraw(msg, startPoints, endPoints);
    //Make a new container for the lines here that deletes and overwrites in two lines to avoid an empty vector while finding new lines.

}

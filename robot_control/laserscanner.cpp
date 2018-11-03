#include "laserscanner.h"

#include <math.h>
#include <iostream>
#include <vector>
#include <random>

#include <opencv2/opencv.hpp>

#define EMPTY '\0'

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
void testDraw(ConstLaserScanStampedPtr &msg,
              std::vector<std::pair<float, float>> startpoints, std::vector<std::pair<float, float>> endpoints, std::vector<std::pair<float, float>> closestP) {
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
    //cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
    //            cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
    //            cv::Scalar(255, 0, 0));
    //cv::putText(im, std::to_string(startpoints.size()),
    //            cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1.0,
    //            cv::Scalar(255, 0, 0));

    for (int i = 0; i < (int)startpoints.size(); i++) {
        cv::line(im, cv::Point(200+startpoints[i].first*px_per_m, 200-startpoints[i].second*px_per_m),
                 cv::Point(200+endpoints[i].first*px_per_m, 200-endpoints[i].second*px_per_m), cv::Scalar(rand()%200, rand()%200, rand()%200), 2, cv::LINE_8);
    }
    //for (int i = 0; i < (int)closestP.size(); i++) {
    //    //Draw lines to shortest distances.
    //    cv::line(im, cv::Point(200+0*px_per_m, 200-0*px_per_m),
    //             cv::Point(200+closestP[i].first*px_per_m, 200-closestP[i].second*px_per_m), cv::Scalar(rand()%200, rand()%200, rand()%200), 2, cv::LINE_8);
    //}
    //cv::line(im, cv::Point(200+0*px_per_m, 200-0*px_per_m),
    //         cv::Point(200+1*px_per_m, 200-0*px_per_m), cv::Scalar(rand()%200, rand()%200, rand()%200), 2, cv::LINE_8);

    mutex.lock();
    cv::imshow("Lidar", im);
    mutex.unlock();
}

void laserscanner::findLines(ConstLaserScanStampedPtr &msg)
{
    std::vector<std::pair<float, float>> startPoints;
    std::vector<std::pair<float, float>> endPoints;
    //std::vector<std::pair<float, float>> oToStart;
    //std::vector<std::pair<float, float>> oToEnd;
    std::vector<std::pair<float, float>> closestPoints;


    std::vector<std::pair<float, float>> cartCoordinates;
    //std::vector<int> scanNumber;
    int amountScans = msg->scan().ranges_size();
    float angle = msg->scan().angle_min();
    float angleInc = msg->scan().angle_step();
    for (int i = 0; i < amountScans; i++) {
        if (! isinf(msg->scan().ranges(i))) {
            float range = msg->scan().ranges(i);
            cartCoordinates.push_back(std::make_pair(range * std::cos(angle), range * std::sin(angle)));
            //scanNumber.push_back(i);
        }// else
        //    cartCoordinates.push_back(std::make_pair(EMPTY, EMPTY));
        angle += angleInc;
    }
    //while (cartCoordinates.front().first == EMPTY)
    //    cartCoordinates.erase(cartCoordinates.begin());
    //while (cartCoordinates.back().first == EMPTY)
    //    cartCoordinates.erase(cartCoordinates.end());

    int npoints = cartCoordinates.size();
    std::cout << "npoints " << npoints << std::endl;

    //Delete the previous lines.
    ransacLines.erase(ransacLines.begin(), ransacLines.end());


    //Following needs to be done N times. (Number of points?)
    for (int find = 0; find < 200; find++) {

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


        int lineThresh = 8;

        for (int i = 0; i < current_coordinates; i++) {
            if (i != point1 && i != point2) {// && cartCoordinates[i].first != EMPTY) {
                distance = std::abs(((y2 - y1)*cartCoordinates[i].first) - ((x2 - x1)*cartCoordinates[i].second) + (x2*y1) - (y2*x1)) / std::sqrt(std::pow(y2-y1, 2)+std::pow(x2-x1, 2));
                //std::cout << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ", " << distance << std::endl;
            }// else
             //   distance = 1;

            if (distance < threshold || i == point1 || i == point2) {
                //Keep
                keep.push_back(std::make_pair(cartCoordinates[i].first, cartCoordinates[i].second));
                keepIndex.push_back(i);
            }
        }


        if ((int)keep.size() >= lineThresh) {

            //Have polar coordinates instead of doing atan2...
            for (int i = 1; i < (int)keep.size(); i++) {
                if (abs(atan2(keep[i].second, keep[i].first)-atan2(keep[i-1].second, keep[i-1].first)) >= 5*angleInc) {
                    keep.erase(keep.begin()+i, keep.end());
                    keepIndex.erase(keepIndex.begin()+i, keepIndex.end());
                }
            }
            std::cout << cartCoordinates.size() << std::endl;
            for (int i = keepIndex.size() - 1; i >= 0; i--) {
                cartCoordinates.erase(cartCoordinates.begin() + keepIndex[i]);
            }
            std::cout << cartCoordinates.size() << std::endl;
            if ((int)keep.size() < lineThresh) {
                std::cout << "hej " << keep.size() << std::endl;
                continue;
            }

            //Do least squares of keep and push line parameters to ransacLines.
            float x_bar = 0;
            float y_bar = 0;
            float xy_bar = 0;
            float xsq_bar = 0;
            float param_a = 0;
            float param_b = 0;

            for (int i = 0; i < (int)keep.size(); i++) {
                x_bar += keep[i].first;
                y_bar += keep[i].second;
                xy_bar += keep[i].first*keep[i].second;
                xsq_bar += std::pow(keep[i].first, 2);
                if (i == 0)
                    startPoints.push_back(std::make_pair(keep[i].first, keep[i].second));
                if (i == (int)keep.size() - 1)
                    endPoints.push_back(std::make_pair(keep[i].first, keep[i].second));
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

        if ((int)cartCoordinates.size() < lineThresh) {
            //std::cout << find << std::endl;
            break;
        }
    }

    if (ransacLines.empty())
        ransacLines.push_back(std::make_pair(0, 0));
    for (int i = 0; i < (int)ransacLines.size(); i++) {
        //std::cout << "y = " << ransacLines[i].first << "x + " << ransacLines[i].second << std::endl;
    }

    for (int i = 0; i < (int)startPoints.size(); i++) {
        //std::cout << "Start, " << startPoints[i].first << ", " << startPoints[i].second << std::endl;
        //std::cout << "End, " << endPoints[i].first << ", " << endPoints[i].second << std::endl;
    }

    std::vector<std::pair<float, float>> regStart;
    std::vector<std::pair<float, float>> regEnd;
    float setX1 = 5;
    float setX2 = 10;

    for (int i = 0; i < (int)startPoints.size(); i++) {
        //Line 1
        float xa1 = startPoints[i].first;
        float ya1 = startPoints[i].second;
        float x2 = 0;
        float y2 = 0;
        float xb1 = endPoints[i].first;
        float yb1 = endPoints[i].second;
        //Line 2
        float x3 = setX1;
        float y3 = ransacLines[i].first*setX1+ransacLines[i].second;
        float x4 = setX2;
        float y4 = ransacLines[i].first*setX2+ransacLines[i].second;

        //Point for startpoint.
        float sdx1 = xa1 * y2 - ya1 * x2;
        float sdx2 = x3 * y4 - y3 * x4;
        float sdx3 = xa1 - x2;
        float sdx4 = x3 - x4;
        float sdx5 = ya1 - y2;
        float sdx6 = y3 - y4;
        float intersectX1 = (sdx1 * sdx4 - sdx3 * sdx2) / (sdx3 * sdx6 - sdx5 * sdx4);

        float sdy1 = sdx1;
        float sdy2 = sdx2;
        float sdy3 = sdx5;
        float sdy4 = sdx6;
        float sdy5 = sdx3;
        float sdy6 = sdx4;
        float intersectY1 = (sdy1 * sdy4 - sdy3 * sdy2) / (sdy5 * sdy4 - sdy3 * sdy6);

        //Point for endpoint.
        sdx1 = xb1 * y2 - yb1 * x2;
        sdx2 = x3 * y4 - y3 * x4;
        sdx3 = xb1 - x2;
        sdx4 = x3 - x4;
        sdx5 = yb1 - y2;
        sdx6 = y3 - y4;
        float intersectX2 = (sdx1 * sdx4 - sdx3 * sdx2) / (sdx3 * sdx6 - sdx5 * sdx4);

        sdy1 = sdx1;
        sdy2 = sdx2;
        sdy3 = sdx5;
        sdy4 = sdx6;
        sdy5 = sdx3;
        sdy6 = sdx4;
        float intersectY2 = (sdy1 * sdy4 - sdy3 * sdy2) / (sdy5 * sdy4 - sdy3 * sdy6);

        regStart.push_back(std::make_pair(intersectX1, intersectY1));
        regEnd.push_back(std::make_pair(intersectX2, intersectY2));
    }

    #define PI 3.14159265
    float xCoord;
    float yCoord;
    //Find the points on the lines closest to the robot.
    //for (int i = 0; i < (int)ransacLines.size(); i++) {
    //
    //        float perpSlope = (1/ransacLines[i].first)*(-1);
    //        xCoord = ransacLines[i].second / (perpSlope-ransacLines[i].first);
    //        yCoord = ransacLines[i].first * xCoord + ransacLines[i].second;
    //
    //        float closestPAngle = atan2(yCoord, xCoord);
    //        float startAngle = atan2(startPoints[i].second, startPoints[i].first);
    //        float endAngle = atan2(endPoints[i].second, endPoints[i].first);
    //        //std::cout << "start: " << startAngle << " closest: " << closestPAngle << " end: " << endAngle << std::endl;
    //        if (startAngle < closestPAngle && closestPAngle < endAngle)
    //            closestPoints.push_back(std::make_pair(xCoord, yCoord));
    //}

    testDraw(msg, regStart, regEnd, closestPoints);
    //Make a new container for the lines here that deletes and overwrites in two lines to avoid an empty vector while finding new lines.

}

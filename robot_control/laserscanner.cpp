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

    //int sec = msg->time().sec();
    //int nsec = msg->time().nsec();

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

    for (int i = 0; i < (int)closestP.size(); i++) {
        //Draw lines to shortest distances.
        cv::line(im, cv::Point(200+0*px_per_m, 200-0*px_per_m),
                 cv::Point(200+closestP[i].first*px_per_m, 200-closestP[i].second*px_per_m), cv::Scalar(0, 0, 150), 2, cv::LINE_8);
    }
    //cv::line(im, cv::Point(200+0*px_per_m, 200-0*px_per_m),
    //         cv::Point(200+1*px_per_m, 200-0*px_per_m), cv::Scalar(rand()%200, rand()%200, rand()%200), 2, cv::LINE_8);

    mutex.lock();
    cv::imshow("Lidar", im);
    mutex.unlock();
}

void laserscanner::findLines(ConstLaserScanStampedPtr &msg)
{
    startPoints.clear();
    endPoints.clear();
    closestPoints.clear();
    cartCoordinates.clear();
    regressionStart.clear();
    regressionEnd.clear();
    ransacLines.clear();

    float threshold = 0.005f;
    int lineThresh = 8;
    float distance = 0;
    int point1;
    int point2;

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
    //std::cout << "npoints " << npoints << std::endl;

    //Following needs to be done N times. (Number of points?)
    for (int find = 0; find < 200; find++) {

        keep.clear();
        keepIndex.clear();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, npoints - 1);
        while(true) {
            point1 = dis(gen);
            point2 = dis(gen);
            if (point1 != point2)
                break;
        }

        float x1 = cartCoordinates[point1].first;
        float y1 = cartCoordinates[point1].second;
        float x2 = cartCoordinates[point2].first;
        float y2 = cartCoordinates[point2].second;

        int current_coordinates = cartCoordinates.size();

        for (int i = 0; i < current_coordinates; i++) {
            if (i != point1 && i != point2) {
                distance = std::abs(((y2 - y1)*cartCoordinates[i].first) - ((x2 - x1)*cartCoordinates[i].second) + (x2*y1) - (y2*x1)) / std::sqrt(std::pow(y2-y1, 2)+std::pow(x2-x1, 2));
            }

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
            for (int i = keepIndex.size() - 1; i >= 0; i--) {
                cartCoordinates.erase(cartCoordinates.begin() + keepIndex[i]);
            }
            if ((int)keep.size() < lineThresh) {
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
        }

        if ((int)cartCoordinates.size() < lineThresh)
            break;
    }

    if (ransacLines.empty())
        ransacLines.push_back(std::make_pair(0, 0));

    for (int i = 0; i < (int)startPoints.size(); i++) {

        //Intersection between regression line and line to startpoint.
        intersectionMatrix startIntersection{
            startPoints[i].first, startPoints[i].second,
            0, 0,
            5, ransacLines[i].first*5+ransacLines[i].second,
            10, ransacLines[i].first*10+ransacLines[i].second
        };

        //Intersection between regression line and line to endpoint.
        intersectionMatrix endIntersection{
            endPoints[i].first, endPoints[i].second,
            0, 0,
            5, ransacLines[i].first*5+ransacLines[i].second,
            10, ransacLines[i].first*10+ransacLines[i].second
        };

        regressionStart.push_back(std::make_pair(startIntersection.pointX, startIntersection.pointY));
        regressionEnd.push_back(std::make_pair(endIntersection.pointX, endIntersection.pointY));
    }

    //Find the points on the lines closest to the robot.
    for (int i = 0; i < (int)ransacLines.size(); i++) {

        float perpSlope = (1/ransacLines[i].first)*(-1);
        closestX = ransacLines[i].second / (perpSlope-ransacLines[i].first);
        closestY = ransacLines[i].first * closestX + ransacLines[i].second;

        float closestPAngle = atan2(closestY, closestX);
        float startAngle = atan2(startPoints[i].second, startPoints[i].first);
        float endAngle = atan2(endPoints[i].second, endPoints[i].first);
        if (startAngle < closestPAngle && closestPAngle < endAngle)
            closestPoints.push_back(std::make_pair(closestX, closestY));
        else if ((0 < endAngle && endAngle < PI/2) || (-PI/2 < startAngle && startAngle < 0)) {
            bool startOrEnd = pow(startPoints[i].first, 2)+pow(startPoints[i].second, 2) < pow(endPoints[i].first, 2)+pow(endPoints[i].second, 2) ? true : false;
            closestX = startOrEnd ? startPoints[i].first : endPoints[i].first;
            closestY = startOrEnd ? startPoints[i].second : endPoints[i].second;
            closestPoints.push_back(std::make_pair(closestX, closestY));
        }
    }

    //Determine the length and angle of the shortest line for the fuzzy controller.
    if (closestPoints.size() > 1) {
        for (int i = 1; i < (int)closestPoints.size(); i++) {
            bool shortestLine = pow(closestPoints[i-1].first, 2)+pow(closestPoints[i-1].second, 2) < pow(closestPoints[i].first, 2)+pow(closestPoints[i].second, 2) ? true : false;
            shortestLineLength = shortestLine ? pow(closestPoints[i-1].first, 2)+pow(closestPoints[i-1].second, 2) : pow(closestPoints[i].first, 2)+pow(closestPoints[i].second, 2);
            shortestLineAngle = shortestLine ? atan2(closestPoints[i-1].second, closestPoints[i-1].first) : atan2(closestPoints[i].second, closestPoints[i].first);
        }
    } else if (closestPoints.size() == 1){
        shortestLineLength = pow(closestPoints[0].first, 2)+pow(closestPoints[0].second, 2);
        shortestLineAngle = atan2(closestPoints[0].second, closestPoints[0].first);
    } else {
        shortestLineLength = pow(msg->scan().range_max(), 2);
        shortestLineAngle = 0;
    }
    shortestLineLength = sqrt(shortestLineLength);
    std::cout << "Length: " << shortestLineLength << " Angle: " << shortestLineAngle << std::endl;

    testDraw(msg, regressionStart, regressionEnd, closestPoints);
}

closestLine laserscanner::getClosestLine()
{
    closestLine line;
    line.distance = shortestLineLength;
    line.angle = shortestLineAngle;
    return line;
}

#ifndef RANSACSCANNER_H
#define RANSACSCANNER_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <vector>


#define PI 3.14159265

struct closestScan {
    float distance;
    float direction;
};

struct closestLine {
    float distance;
    float angle;
    float distance2;
};

struct intersectionMatrix {
    intersectionMatrix(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) :
        x1(x1), y1(y1), x2(x2), y2(y2), x3(x3), y3(y3), x4(x4), y4(y4) {}
private:

    float x1, y1, x2, y2, x3, y3, x4, y4;

    float dxy1 = x1-x2;
    float dxy2 = y3-y4;
    float dxy3 = y1-y2;
    float dxy4 = x3-x4;

    float dx1 = x1*y2-y1*x2;
    float dx2 = dxy4;
    float dx3 = dxy1;
    float dx4 = x3*y4-y3*x4;

    float dy1 = dx1;
    float dy2 = dxy2;
    float dy3 = dxy3;
    float dy4 = dx4;
public:
    float pointX = (dx1*dx2-dx3*dx4) / (dxy1*dxy2-dxy3*dxy4);
    float pointY = (dy1*dy2-dy3*dy4) / (dxy1*dxy2-dxy3*dxy4);
};



class laserscanner
{
public:
    laserscanner();
    void performScan(ConstLaserScanStampedPtr &msg);
    closestScan getClosestScan();

    void findLines(ConstLaserScanStampedPtr &msg);
    closestLine getClosestLine();

private:
    closestScan scan;

    float nranges;
    float angle_step;
    float angle_min;
    float smallest_dist;
    float smallest_dir;
    float new_angle;

    //RANSAC
    std::vector<std::pair<float, float>> startPoints;
    std::vector<std::pair<float, float>> endPoints;
    std::vector<std::pair<float, float>> cartCoordinates;
    std::vector<std::pair<float, float>> closestPoints;

    std::vector<std::pair<float, float>> keep;
    std::vector<int> keepIndex;

    std::vector<std::pair<float, float>> ransacLines;

    std::vector<std::pair<float, float>> regressionStart;
    std::vector<std::pair<float, float>> regressionEnd;

    float closestX;
    float closestY;

    float shortestLineLength;
    float shortestLineAngle;

    float shortestLineLength2;
};

#endif // LASERSCANNER_H

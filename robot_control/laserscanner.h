#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <vector>

struct closestScan {
    float distance;
    float direction;
};


class laserscanner
{
public:
    laserscanner();
    void performScan(ConstLaserScanStampedPtr &msg);
    closestScan getClosestScan();

    void findLines(ConstLaserScanStampedPtr &msg);

private:
    closestScan scan;

    float nranges;
    float angle_step;
    float angle_min;
    float smallest_dist;
    float smallest_dir;
    float new_angle;

    //RANSAC
    std::vector<std::pair<float, float>> ransacLines;
};

#endif // LASERSCANNER_H

#ifndef MOVETOPOINT_H
#define MOVETOPOINT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <vector>

struct distAndAngle {
    float distance;
    float angle;
};

class MoveToPoint
{
public:
    MoveToPoint();

    void setGoal(float x, float y);
    distAndAngle leftToGoal(float positionX, float positionY);
    distAndAngle leftToGoal();
    void setPosition(ConstPosesStampedPtr &msg);
    void displayGoal(ConstPosesStampedPtr &msg);

private:
    std::pair<float, float> goal;
    std::pair<float, float> position;
    float orientation;
};

#endif // MOVETOPOINT_H

#include "mclocalizer.h"

MCLocalizer::MCLocalizer()
{

}

void MCLocalizer::localPoseCallback(ConstPosesStampedPtr &_msg)
{
    float c_pl; // * Current position of wheels
    float c_pr; // *

    // ** Extract positions from the msg
    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::right_wheel") {
        double qw = _msg->pose(i).orientation().w();
        double qx = _msg->pose(i).orientation().x();
        double qy = _msg->pose(i).orientation().y();
        double qz = _msg->pose(i).orientation().z();
        double nqw = qw * 0.5 - qx * 0.5 - qy * 0.5 - qz * 0.5;
        double nqy = qw * 0.5 - qx * 0.5 + qy * 0.5 + qz * 0.5;

        c_pr = atan2(nqw, nqy);
      }
      if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::left_wheel") {
        double qw = _msg->pose(i).orientation().w();
        double qx = _msg->pose(i).orientation().x();
        double qy = _msg->pose(i).orientation().y();
        double qz = _msg->pose(i).orientation().z();
        double nqw = qw * 0.5 - qx * 0.5 - qy * 0.5 - qz * 0.5;
        double nqy = qw * 0.5 - qx * 0.5 + qy * 0.5 + qz * 0.5;

        c_pl = atan2(nqw, nqy);
      }
    }
    // Extract time from the msg
    float c_t = _msg->time().sec() + 0.000000001 * _msg->time().nsec();

    if(t < 0){
        pr = c_pr;
        pl = c_pl;
        t  = c_t;
        return;
    }

    float d_pr = c_pr - pr; // *
    float d_pl = c_pl - pl; // | deltas
    float dt   = c_t  - t;  // *

    pr = c_pr; // *
    pl = c_pl; // | overwrite
    t  = c_t;  // *

    if (d_pl > 1.8 * M_PI)  d_pl -= 2 * M_PI; // *
    if (d_pr > 1.8 * M_PI)  d_pr -= 2 * M_PI; // | delta correction
    if (d_pr < -1.8 * M_PI) d_pr += 2 * M_PI; // | when pos jumps 2 pi
    if (d_pl < -1.8 * M_PI) d_pl += 2 * M_PI; // *

    float av_r = 2 * d_pr / dt; // * Angular velocity of wheels
    float av_l = 2 * d_pl / dt; // *

    const double l = 0.34 + 0.05;

    double vl = av_l * 0.12; // * Velocity of wheels
    double vr = av_r * 0.12; // *

    double R = (l / 2) * (vl + vr) / (vr - vl);
    double omega = (vr - vl) / l;

    float dphi = omega * dt;
    float dx = -sin(dphi)*R;
    float dy =  cos(dphi)*R - R;

    upos.x += dx*cos(uphi) + dy*sin(uphi);
    upos.y += dx*sin(uphi) + dy*cos(uphi);
    uphi   += dphi;
}

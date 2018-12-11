
#include "localizor.h"


Localizor::Localizor(int w, int h, float phi) {
  upos.x = 0;
  upos.y = 0;
  uphi   = phi;

  center.x = w / 2;
  center.y = h / 2;
}
void Localizor::globalPoseCallback(ConstPosesStampedPtr &_msg)
{
    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx") {
        apos.x = center.x /2 + _msg->pose(i).position().x() * s;
        apos.y = center.y /2 - _msg->pose(i).position().y() * s;
        aphi   = -M_PI + 2 * atan2(_msg->pose(i).orientation().w(),
                                 _msg->pose(i).orientation().z());
      }
    }
}

void Localizor::printPose() {
  cout << fixed << "X: " << setw(5) << setprecision(2) << upos.x << " Y: " << setw(5) << setprecision(2) << upos.y
       << " p: " << setw(5) << setprecision(2) << uphi << " e: " << setw(5) << setprecision(2) << upos.x - apos.x << "  "
       << setw(5) << setprecision(2) << upos.y - apos.y << endl;
}

double Localizor::getX() { return center.x + upos.x; }

double Localizor::getY() { return center.y - upos.y; }

void Localizor::localPoseCallback(ConstPosesStampedPtr &_msg) {

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

    if (d_pl > M_PI)  d_pl -= 2 * M_PI; // *
    if (d_pr > M_PI)  d_pr -= 2 * M_PI; // | delta correction
    if (d_pr < -M_PI) d_pr += 2 * M_PI; // | when pos jumps 2 pi
    if (d_pl < -M_PI) d_pl += 2 * M_PI; // *

    const double l = 0.34 + 0.05;
    const double r = 0.12;

    double vl = (2 * d_pl / dt) * r; // * Velocity of wheels
    double vr = (2 * d_pr / dt) * r; // *

    double nx = 0;
    double ny = 0;
    double dphi = 0;

    if (vr - vl != 0){
        double R = (l / 2) * (vl + vr) / (vr - vl);
        double omega = (vr - vl) / l;

        dphi = omega * dt;


        double ICC_x = upos.x - R * sin(uphi);
        double ICC_y = upos.y - R * cos(uphi);

        nx = (cos(dphi) * (upos.x - ICC_x) - sin(dphi) * (upos.y - ICC_y)) + ICC_x;
        ny = (sin(dphi) * (upos.x - ICC_x) + cos(dphi) * (upos.y - ICC_y)) + ICC_y;
    }else{
        nx = upos.x + (vr * dt)*cos(uphi);
        ny = upos.y - (vr * dt)*sin(uphi);
    }

    upos.x = nx;
    upos.y = ny;
    uphi  += dphi;

    //mx = upos.x * G2P + bitmap.cols / 2;
    //my = -upos.y * G2P + bitmap.rows / 2;

}

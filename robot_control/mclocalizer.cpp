#include "mclocalizer.h"

MCLocalizer::MCLocalizer(Mat m, uchar s)
{
    scale = s;

    int rows = m.rows;
    int cols = m.cols;

    map = Mat(rows * scale, cols * scale, CV_8UC1); // redo this with cv resize

    for (int r = 0; r < rows; r++) {
      uchar *value = m.ptr(r);
      for (int c = 0; c < cols; c++) {
        uchar g = (*value++ < 128) ? 0 : 255;
        for (int j = 0; j < scale; j++) {
          for (int i = 0; i < scale; i++) {
            map.at<uchar>(r * scale + j, c * scale + i) = g;
          }
        }
      }
    }   // end redo

    for(int i = 0; i < N_CONF; i++){
        bel[i].x = (cols / 2.) * scale;
        bel[i].y = (rows / 2.) * scale;
        bel[i].dir = 0;
    }

}

MCLocalizer::~MCLocalizer()
{
    delete _ptex;
    delete _btex;
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

    const double l = 0.34 + 0.05;
    const double r = 0.12;

    double vl = (2 * d_pl / dt) * 0.12; // * Velocity of wheels
    double vr = (2 * d_pr / dt) * 0.12; // *

    float dphi = 0;
    float dx   = vr * dt;
    float dy   = 0;

    if(vr - vl != 0){
        double R = (l / 2) * (vl + vr) / (vr - vl); // Center of rotation

        dphi = ((vr - vl) / l) * dt;
        dx = -sin(dphi)*R;
        dy =  cos(dphi)*R - R;
    }

    upos.x += dx*cos(uphi) + dy*sin(uphi);
    upos.y += dx*sin(uphi) + dy*cos(uphi);
    uphi   += dphi;

}

void MCLocalizer::lidarScanCallback(ConstLaserScanStampedPtr &msg)
{
    laserScanner.parseScan(msg);
    localize(laserScanner.getScan());
}

void MCLocalizer::globalPoseCallback(ConstPosesStampedPtr &_msg)
{
    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx") {
        apos.x = map.cols /2 + _msg->pose(i).position().x() * s;
        apos.y = map.rows /2 - _msg->pose(i).position().y() * s;
        aphi   = -M_PI + 2 * atan2(_msg->pose(i).orientation().w(),
                                 _msg->pose(i).orientation().z());
      }
    }
}

void MCLocalizer::show()
{
    Mat m = Mat(map.rows, map.cols, CV_8UC3);
    cvtColor(map, m, COLOR_GRAY2BGR);
    //_btex->lock();
    for(int i = 0; i < N_CONF; i++){
      Point2f p(bel[i].x,bel[i].y);
      arrowedLine(
          m, p,
          p + Point2f(8 * cos(bel[i].dir), 8 * sin(bel[i].dir)),
          Scalar(120, 120, 255), 1, LINE_AA, 0, .5);
    }
    //_btex->unlock();
    /*
    arrowedLine(
        m, rConf[0].pos,
        rConf[0].pos + Point2f(8 * cos(rConf[0].dir), 8 * sin(rConf[0].dir)),
        Scalar(0, 255, 0), 1, LINE_AA, 0, .5);*/
    arrowedLine(m, apos, apos + Point2f(6 * cos(aphi), 6 * sin(aphi)), Scalar(255, 0, 0), 1, LINE_AA, 0, .5);

     /*imshow("Real Lidar", laserScanner.visualizeScan(laserScanner.getScan()));
     imshow("Map Lidar",
    laserScanner.visualizeScan(laserScanner.generateScan(map, Point2f(bel[0].x,bel[0].y), bel[0].dir)));
    */
    imshow("Map", m);
}

void MCLocalizer::tempBelief()
{
    double x = 0;
    double y = 0;
    double phi = 0;

        x   = upos.x; upos.x = 0;
        y   = upos.y; upos.y = 0;
        phi = uphi;   uphi   = 0;

    for(int i = 0; i < N_CONF; i++){
        double tphi = bel[i].dir ;
        tbel[i].x   = bel[i].x + (x*cos(tphi) + y*sin(tphi))*s * ndist(gen);
        tbel[i].y   = bel[i].y + (x*sin(tphi) + y*cos(tphi))*s * ndist(gen);
        tbel[i].dir = tphi + phi* ndist(gen);
    }
}

bool conf_sorter(conf const &lhs, conf const &rhs) {
  return lhs.weight < rhs.weight;
}

void MCLocalizer::calcWeights(LaserScan *ls)
{
    for(int i = 0; i < N_CONF; i++){
        Point2f p(tbel[i].x,tbel[i].y);
        LaserScan *gls = laserScanner.generateScan(map, p, tbel[i].dir);
        float score = 0;
        for (int j = 0; j < ls->ntps; j++) {
          score += std::abs(ls->pts[j] - gls->pts[j]);
        }
        tbel[i].weight = score / ls->ntps;
        delete gls;
    }


    std::cout << std::endl;
}

void MCLocalizer::localize(LaserScan *ls)
{
    tempBelief();
    calcWeights(ls);

    std::sort(tbel.begin(), tbel.end(), &conf_sorter);
    for(int i = 0; i < N_CONF; i++){
        int sel = std::abs(ndist_conf(gen));
        if (sel >= N_CONF) sel = N_CONF - 1;
        bel[i] = tbel[sel];
    }
}

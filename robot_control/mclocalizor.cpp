#include "mclocalizor.h"

MCLocalizor::MCLocalizor(Mat m, uchar s)
{
    scale = s;

    rows = m.rows;
    cols = m.cols;

    map = Mat(rows * scale, cols * scale, CV_8UC1); // redo this with cv resize

    Mat mi (rows * scale, cols * scale, CV_8UC3);


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
    cvtColor(map, mi, COLOR_GRAY2BGR);
    resize(mi,prettymap,Size(),2,2,INTER_NEAREST);

    for(int i = 0; i < N_CONF; i++){
        bel[i].x = (cols / 2.) * scale;
        bel[i].y = (rows / 2.) * scale;
        bel[i].dir = 0;
    }

    positions.open("pos0.txt");
}

MCLocalizor::~MCLocalizor()
{
    delete _ptex;
    delete _btex;
    positions.close();
}

void MCLocalizor::localPoseCallback(ConstPosesStampedPtr &_msg)
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

    if (d_pl > M_PI)  d_pl -= 2 * M_PI; // *
    if (d_pr > M_PI)  d_pr -= 2 * M_PI; // | delta correction
    if (d_pr < -M_PI) d_pr += 2 * M_PI; // | when pos jumps 2 pi
    if (d_pl < -M_PI) d_pl += 2 * M_PI; // *

    const double l = 0.34 + 0.05;
    const double r = 0.11;

    double vl = (2 * d_pl / dt) * r; // * Velocity of wheels
    double vr = (2 * d_pr / dt) * r; // *

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

void MCLocalizor::lidarScanCallback(ConstLaserScanStampedPtr &msg)
{
    laserScanner.parseScan(msg);
    localize(laserScanner.getScan());
}

void MCLocalizor::globalPoseCallback(ConstPosesStampedPtr &_msg)
{
    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx") {
        apos.x = map.cols /2 + _msg->pose(i).position().x() * s;
        apos.y = map.rows /2 - _msg->pose(i).position().y() * s;
        aphi   = -M_PI + 2 * atan2(_msg->pose(i).orientation().w(),
                                 _msg->pose(i).orientation().z());
      }
    }
    if(breset) reset();
}

void MCLocalizor::show()
{
    Mat m = prettymap.clone();

    //cvtColor(prettymap, m, COLOR_GRAY2BGR);
    for(int i = 0; i < N_CONF; i++){
      Point2f p(bel[i].x,bel[i].y);
      p*=2;
      arrowedLine(
          m, p,
          p + Point2f(14 * cos(bel[i].dir), 12 * sin(bel[i].dir)),
          Scalar(120, 120, 255), 1, LINE_AA, 0, .5);
    }
    arrowedLine(m, apos*2, apos*2 + Point2f(10 * cos(aphi), 10 * sin(aphi)), Scalar(255, 0, 0), 1, LINE_AA, 0, .5);

    //circle(prettymap, Point2f(tbel[0].x, tbel[0].y)*2, 2, Vec3b(0,0,255), -1);
//    prettymap.at<Vec3b>(apos*2) = Vec3b(255,0,0);
//    prettymap.at<Vec3b>(Point2f(tbel[0].x, tbel[0].y)*2) = Vec3b(0,0,255);


    //float phi = tbel[0].dir;
    //positions << apos.x << "," << -apos.y << "," << tbel[0].x - 0.56*cos(phi)<< "," << -(tbel[0].y - 0.56*sin(phi)) << std::endl;

    //circle(m,Point(90,30),2,Scalar(0,255,0),-1);
    //circle(m,Point(90,30)*4,2,Scalar(0,255,0),-1);

    //imshow("real lidar", laserScanner.visualizeScan(laserScanner.getScan()));
    //imshow("fake lidar", laserScanner.visualizeScan(laserScanner.generateScan(map, Point2f(tbel[0].x,tbel[0].y), tbel[0].dir)));
    imshow("mc map", m);
}

void MCLocalizor::tempBelief()
{
    double x = 0;

    double y = 0;
    double phi = 0;

    x   = upos.x; upos.x = 0;
    y   = upos.y; upos.y = 0;
    phi = uphi;   uphi   = 0;

    for(int i = 0; i < N_CONF - 1; i++){
        double tphi = fmod(bel[i].dir, 2*M_PI) ;
        tbel[i].x   = bel[i].x + (x*cos(tphi) + y*sin(tphi))*s * ndist(gen);
        tbel[i].y   = bel[i].y + (x*sin(tphi) + y*cos(tphi))*s * ndist(gen);
        tbel[i].dir = tphi + phi * ndist(gen);
    }
    double tphi = fmod(bel[N_CONF-1].dir, 2*M_PI) ;
    tbel[N_CONF-1].x   = bel[N_CONF-1].x + (x*cos(tphi) + y*sin(tphi))*s;
    tbel[N_CONF-1].y   = bel[N_CONF-1].y + (x*sin(tphi) + y*cos(tphi))*s;
    tbel[N_CONF-1].dir = tphi + phi;

}

bool conf_sorter(conf const &lhs, conf const &rhs) {
  return lhs.weight < rhs.weight;
}

void MCLocalizor::calcWeights(LaserScan *ls)
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
}

void MCLocalizor::localize(LaserScan *ls)
{
    tempBelief();
    calcWeights(ls);

    std::sort(tbel.begin(), tbel.end(), &conf_sorter);

    for(int i = 0; i < N_CONF -1; i++){
        int sel = std::abs(ndist_conf(gen));
        if (sel >= N_CONF) sel = N_CONF - 1;
        bel[i] = tbel[sel];
    }

    bel[N_CONF-1] = tbel[N_CONF-1];

    float x = 0;
    float y = 0;
    float phi = 0;

    for(int i = 0 ; i < 10; i++){
        x += tbel[i].x;
        y += tbel[i].y;
        phi += tbel[i].dir;
    }
    x /= 10;
    y /= 10;
    phi /= 10;

    prettymap.at<Vec3b>(apos*2) = Vec3b(255,0,0);
    prettymap.at<Vec3b>(Point2f(x- 0.56*cos(phi)+0.56, y - 0.56*sin(phi)+0.56)*2) = Vec3b(0,0,255);
    if(bel[N_CONF-1].x < 480 && bel[N_CONF-1].x >= 0 && bel[N_CONF-1].y < 320 && bel[N_CONF-1].y >=0)
    prettymap.at<Vec3b>(Point2f(bel[N_CONF-1].x, bel[N_CONF-1].y)*2) = Vec3b(0,255,0);

//    positions << apos.x << "," << -apos.y << "," << x- 0.56*cos(phi)<< "," << -(y - 0.56*sin(phi))
//              << "," << bel[N_CONF-1].x << "," << bel[N_CONF-1].y << std::endl;
    positions << apos.x << "," << -apos.y << "," << x- 0.56*cos(phi)+0.56<< "," << -(y - 0.56*sin(phi))-0.56
              << "," << bel[N_CONF-1].x + 0.56<< "," << -bel[N_CONF-1].y-0.56 << std::endl;
}

conf MCLocalizor::getConfiguration() {
    return tbel[0];
}

void MCLocalizor::reset()
{
    for(int i = 0; i < N_CONF; i++){
        bel[i].x = apos.x;
        bel[i].y = apos.y;
        bel[i].dir = aphi;
    }
    upos.x = 0;
    upos.y = 0;
    uphi = 0;
    breset = false;
}

void MCLocalizor::qreset()
{
    breset = true;
    positions.close();
    positions.open("pos" + std::to_string(nfile++) + ".txt");
}

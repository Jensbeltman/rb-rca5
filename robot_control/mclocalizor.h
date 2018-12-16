#ifndef MCLOCALIZER_H
#define MCLOCALIZER_H

#include <mutex>
#include <array>

#include <opencv2/opencv.hpp>
#include "laserscanner.h"
#include <fstream>

#define N_CONF 40

using namespace cv;

struct conf {
  float x;
  float y;
  float dir;
  float weight;
};


class MCLocalizor
{
public:
    MCLocalizor(){}
    MCLocalizor(Mat m, uchar s);
    ~MCLocalizor();

    // Gazebo subscribers
    void localPoseCallback(ConstPosesStampedPtr &_msg);
    void lidarScanCallback(ConstLaserScanStampedPtr &msg);

    void globalPoseCallback(ConstPosesStampedPtr &_msg); // for debug only

    void show();

    conf getConfiguration();
    void reset();

    void qreset();


private:
    float s = ((72.0 / 25.4) * 2);
    int rows;
    int cols;

    bool breset = false;

    std::mutex * _ptex = new std::mutex();
    std::mutex * _btex = new std::mutex();
    int scale;

    Mat map;
    Mat prettymap;

    Point2f upos;
    float   uphi;

    Point2f apos; // * actual pos
    float   aphi; // * and dir

    LaserScanner laserScanner;

    std::default_random_engine gen;
    std::normal_distribution<float> ndist = std::normal_distribution<float>(1.0, 0.3);
    std::normal_distribution<float>   ndist_conf = std::normal_distribution<float> (0,N_CONF/3);

    float t = -1;
    float pl = 0;
    float pr = 0;

    conf  bel[N_CONF];
    std::array<conf, N_CONF> tbel;

    void tempBelief();
    void calcWeights(LaserScan *ls);
    void localize(LaserScan *ls);

    std::ofstream positions;
    int nfile = 0;

    bool skipPrint = false;
};

#endif // MCLOCALIZER_H

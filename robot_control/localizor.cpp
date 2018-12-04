#include "localizor.h"

Localizor::Localizor() {}

Localizor::Localizor(Mat bmap, Mat heatmap) {
  bitmap = bmap;
  cnrheatmap = heatmap;
  montecarloMap = bmap;
  x = 0;
  y = 0;
  phi = 0;
  mx = bmap.cols / 2;
  my = bmap.rows / 2;
  tx = 0;
  ty = 0;
  montecarlo = Montecarlo(montecarloMap);
}
void Localizor::poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
	if (_msg->pose(i).name() == "pioneer2dx") {
	  rx = _msg->pose(i).position().x();
	  ry = _msg->pose(i).position().y();
	  rphi = -M_PI + 2 * atan2(_msg->pose(i).orientation().w(),
							   _msg->pose(i).orientation().z());
	}
  }
}

void Localizor::printPose() {
  cout << fixed << "X: " << setw(5) << setprecision(2) << x << " Y: " << setw(5) << setprecision(2) << y
       << " p: " << setw(5) << setprecision(2) << phi << " e: " << setw(5) << setprecision(2) << rx - x << "  "
       << setw(5) << setprecision(2) << ry - y << endl;
}

double Localizor::getX() { return mx; }

double Localizor::getY() { return my; }

bool Localizor::cnrInrange(int threshold) {
  // checks if the green color is not too white or white
  if ((cnrheatmap.at<Vec3b>(my, mx)[0] < threshold)) return true;

  return false;
}
int test_ittr = 0;
float test_x = 0;
float test_y = 0;
float test_phi = 0;

void Localizor::localPoseCallback(ConstPosesStampedPtr &_msg) {
  // cout << "test" << endl;

  for (int i = 0; i < _msg->pose_size(); i++) {
	if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::right_wheel") {
	  qw = _msg->pose(i).orientation().w();
	  qx = _msg->pose(i).orientation().x();
	  qy = _msg->pose(i).orientation().y();
	  qz = _msg->pose(i).orientation().z();
	  double nqw = qw * 0.5 - qx * 0.5 - qy * 0.5 - qz * 0.5;
	  double nqy = qw * 0.5 - qx * 0.5 + qy * 0.5 + qz * 0.5;

      //if (fabs(rw - qw) > 0.02) first = true;
      //rw = qw;

	  p_r = M_PI + atan2(nqw, nqy);
	  // cout << fixed << setw(4) << p_r << endl;
	}
	if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::left_wheel") {
	  qw = _msg->pose(i).orientation().w();
	  qx = _msg->pose(i).orientation().x();
	  qy = _msg->pose(i).orientation().y();
	  qz = _msg->pose(i).orientation().z();
	  double nqw = qw * 0.5 - qx * 0.5 - qy * 0.5 - qz * 0.5;
	  double nqy = qw * 0.5 - qx * 0.5 + qy * 0.5 + qz * 0.5;

      //if (fabs(lw - qw) > 0.02) first = true;
      //lw = qw;

	  p_l = M_PI + atan2(nqw, nqy);
	}
  }

  t = _msg->time().sec() + 0.000000001 * _msg->time().nsec();

  if (first) {
      prev_p_l = p_l;
      prev_p_r = p_r;
      prev_t = t;
      first = false;
      return;
  }

	dp_l = p_l - prev_p_l;
	dp_r = p_r - prev_p_r;
	dt = t - prev_t;

	if (dp_l > 1.8 * M_PI) {
	  dp_l -= 2 * M_PI;
	}
	if (dp_r > 1.8 * M_PI) {
	  dp_r -= 2 * M_PI;
	}

	if (dp_l < -1.8 * M_PI) {
	  dp_l += 2 * M_PI;
	}
	if (dp_r < -1.8 * M_PI) {
	  dp_r += 2 * M_PI;
	}

	v_l = dp_l / dt;
	v_r = dp_r / dt;

	prev_p_l = p_l;
	prev_p_r = p_r;
	prev_t = t;

    const double l = 0.34 + 0.05;
	double R, omega;

    double dpx = 0;
    double dpy = 0;
	double dphi = 0;
	double ICC_x, ICC_y;

    double vl = v_l * (0.12 * 2);
    double vr = v_r * (0.12 * 2);

	R = (l / 2) * (vl + vr) / (vr - vl);
	omega = (vr - vl) / l;

	dphi = omega * dt + phi;

    ICC_x = x - R * sin(phi); //0
    ICC_y = y - R * cos(phi); //-R

    dpx = (cos(omega * dt) * (x - ICC_x) - sin(omega * dt) * (y - ICC_y)) + ICC_x;
    dpy = (sin(omega * dt) * (x - ICC_x) + cos(omega * dt) * (y - ICC_y)) + ICC_y;

	x = dpx;
	y = dpy;
	phi = dphi;

	mx = x * G2P + bitmap.cols / 2;
	my = -y * G2P + bitmap.rows / 2;

    float ndphi = dt * omega;

    float ndx = -sin(ndphi)*R;
    float ndy =  cos(ndphi)*R - R;

    test_x += ndx*cos(test_phi) + ndy*sin(test_phi);
    test_y += ndx*sin(test_phi) + ndy*cos(test_phi);
    test_phi += ndphi;

    if(++test_ittr == 50){

        Mat test(200,200,CV_8UC3);

        test.setTo(255);

        Point arrow_end1(100 + 200*test_x,100 + 200*test_y);   
        Point arrow_end2(100 + 50*cos(test_phi), 100 + 50*sin(test_phi));

        arrowedLine(test,Point(100,100),arrow_end1,Scalar(0,0,255),1,8,0,0.1);
        arrowedLine(test,Point(100,100),arrow_end2,Scalar(255,0,0),1,8,0,0.1);

        imshow("test 1", test);

        test_x = 0;
        test_y = 0;
        test_phi = 0;
        test_ittr = 0;

    }
	montecarlo.setConf(Point2f(4 * mx, 4 * my), (float)phi);



}

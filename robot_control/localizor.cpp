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
  cout << fixed << "X: " << setw(5) << x << ", Y: " << setw(5) << y
	   << ", phi: " << setw(5) << phi << "error: " << setw(5) << rx - x << ", "
	   << setw(5) << ry - y << endl;
}

double Localizor::getX() { return mx; }

double Localizor::getY() { return my; }

bool Localizor::cnrInrange(int threshold) {
  // checks if the green color is not too white or white
  if ((cnrheatmap.at<Vec3b>(my, mx)[0] < threshold)) return true;

  return false;
}
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

	  if (fabs(rw - qw) > 0.02) first = true;
	  rw = qw;

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

	  if (fabs(lw - qw) > 0.02) first = true;
	  lw = qw;

	  p_l = M_PI + atan2(nqw, nqy);
	}
  }
  if (cnrInrange(100)) {
	nmcA = 80;
  } else {
	nmcA = max(0, nmcA - 1);
  }
  if (nmcA > 0) {
	if (!mcActive) {
	  mcActive = true;
	  montecarlo.reDistribute(Point2f(mx * 4, my * 4), phi);
	} else {
	  if (nMonte++ >= 0) {
		tx += P2G * montecarlo.getBestPos().x;
		ty += P2G * montecarlo.getBestPos().y;
		tphi += montecarlo.getBestDir();
	  }
	  if (nMonte >= 20) {
		nMonte = -40;
		x = tx / 20.;
		y = ty / 20.;
		phi = tphi / 20.;
		tx = 0;
		ty = 0;
		tphi = 0;
		mx = x * G2P + bitmap.cols / 2;
		my = -y * G2P + bitmap.rows / 2;
		// montecarlo.reDistribute(Point2f(mx*4,my*4),phi); // enten eller :
		montecarlo.setConf(Point2f(4 * mx, 4 * my), (float)phi, true);
	  }
    }
  } else {
	mcActive = false;
	nMonte = -40;
  }

  if (!first) {
	t = _msg->time().sec() + 0.000000001 * _msg->time().nsec();

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

	//	cout << fixed << setprecision(6) << "pl: " << setw(4) <<
	// prev_pitch_l
	//		   << " | pr: " << setw(4) << prev_pitch_r << " ||   vl:
	//" <<
	// setw(4)
	//<< 	 vel_l
	//		   << " ||   vr: " << setw(4) << vel_r << endl;

	prev_p_l = p_l;
	prev_p_r = p_r;
	prev_t = t;

	const double l = 0.34;  // + 0.05;
	double R, omega;

    double dpx = 0;
    double dpy = 0;
	double dphi = 0;
	double ICC_x, ICC_y;

	double vl = v_l * (0.11 * 2);
	double vr = v_r * (0.11 * 2);

	R = (l / 2) * (vl + vr) / (vr - vl);
	omega = (vr - vl) / l;

	dphi = omega * dt + phi;

	ICC_x = x - R * sin(phi);
	ICC_y = y - R * cos(phi);

	dpx =
		(cos(omega * dt) * (x - ICC_x) - sin(omega * dt) * (y - ICC_y)) + ICC_x;
	dpy =
		(sin(omega * dt) * (x - ICC_x) + cos(omega * dt) * (y - ICC_y)) + ICC_y;

	x = dpx;
	y = dpy;
	phi = dphi;

	mx = x * G2P + bitmap.cols / 2;
	my = -y * G2P + bitmap.rows / 2;

	montecarlo.setConf(Point2f(4 * mx, 4 * my), (float)phi);

	//	cout << fixed << "X: " << setw(5) << x << ",
	// Ymontecarlo.pos.x=dpx;: "
	//<< setw(5) << y
	//		 << ", phi: " << setw(5) << dphi << setw(5) << lw <<
	// setw(5) <<
	// rw
	//		 << endl;
  } else {
	prev_p_l = p_l;
	prev_p_r = p_r;
	prev_t = t;
	first = false;
  }
}

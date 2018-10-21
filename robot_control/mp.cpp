#include "mp.h"

mp::mp() {}

mp::mp(Mat src) {
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);

  cvtColor(src, bitmap, COLOR_BGR2GRAY);
  bitmap.copyTo(bitmap_t);
  resize(src,display,src.size()*4,0,0,INTER_NEAREST);
  cornerMask = Mat::zeros(display.rows, display.cols, CV_8UC3);
  areaMask = cornerMask.clone();
  namedWindow("map", WINDOW_AUTOSIZE);
  resizeWindow("map", display.cols, display.rows);

  //findAreas();
}

int mp::findCorners(Mat map) {

  cnr_t.clear();
  corner C;
  int kVal = 0;
  int mapVal = 0;
  for (int r = 0; r < map.rows - 1; r++) {
    for (int c = 0; c < map.cols - 1; c++) {
      kVal = 0;

      for (int x = 0; x < cornerkernel.cols; x++) {
        for (int y = 0; y < cornerkernel.rows; y++) {
          kVal +=
              cornerkernel.at<uchar>(y, x) * (map.at<uchar>(r + y, c + x) / 255);
        }
      }

      switch (kVal) {
      case 5:
        //		arrowedLine(display, Point(c + 1, r + 1), Point(c + aL,
        // r
        //+ aL), 					Scalar(0, 255, 0));
        C.P = Point(c + 1, r + 1);
        C.D = Point(1, 1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;
      case 7:
        //		arrowedLine(display, Point(c, r + 1), Point(c - aL, r +
        // aL), 					Scalar(0, 255, 0));
        C.P = Point(c, r + 1);
        C.D = Point(-1, 1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;
      case 3:
        //		arrowedLine(display, Point(c + 1, r), Point(c + aL, r -
        // aL), 					Scalar(0, 255, 0));
        C.P = Point(c + 1, r);
        C.D = Point(1, -1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;
      case 1:
        //		arrowedLine(display, Point(c, r), Point(c - aL, r - aL),
        //					Scalar(0, 255, 0));
        C.P = Point(c, r);
        C.D = Point(-1, -1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;

      default:
        break;
      }
    }
  }
  return cnr_t.size();
}

void mp::drawCorners() {
  for (int i = 0; i < cnr.size(); i++) {
    if (cnr[i].type == 'i') {
      display.at<Vec3b>(cnr[i].P) = Vec3b(0, 255, 0);
    } else {
      display.at<Vec3b>(cnr[i].P) = Vec3b(255, 0, 0);
    }
  }
}

void mp::connectCorners(){
    findCorners(bitmap);
    int rows = bitmap.rows;
    int cols = bitmap.cols;
    int maxc = 0;

    vector<vispos> vps;

    //vector<corner> cnrt();
    //vector<corner*> cnr_ptr();
    /*for(int i = 0; i < cnr_t.size(); i++){
        cnr_ptr[i] = &cnr_t[i];
    }*/

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if(bitmap.at<uchar>(r,c) != 0){
                int count = 0;

                vispos cvp;
                cvp.pos = Point(c,r);

                for(int i = 0; i < cnr_t.size(); i++){
                    LineIterator li(bitmap, Point(c,r), cnr_t[i].P, 4);
                    if(li.count < 40) {
                        bool connected = true;

                        for(int j = 0; j < li.count; j++,++li){

                            if(**li == 0){
                                connected = false;
                                break;
                            }
                        }
                        if(connected){
                            cvp.corners.push_back(i);
                            count += 16;
                            cnr_t[i].seenBy++;
                        }
                    }
                }
                vps.push_back(cvp);

                if(count > 255) count = 255;
                maxc = max(maxc, count);
                rectangle(display, Point(4*c,4*r),Point(4*c+3,4*r+3),Scalar(255-count,255-count,255),FILLED);
            }
        }
    }
    cout << endl;
    cout << "display";
    imshow("map", display);
    waitKey(0);

    vector<corner> cnrrem(cnr_t);


    // copy vector to sort
    vector<corner> cnrt(cnr_t);
    sort(cnrt.begin(),cnrt.end(),[](corner a, corner b){
        return a.seenBy > b.seenBy;
    });
    // find shit and scale and normalize score
    int   shift = cnrt[cnrt.size()-1].seenBy;
    float scale = 1/ (cnrt[0].seenBy - shift);

    for(int i = 0; i < cnrt.size(); i++){
        cout <<  setw(5) << cnrt[i].seenBy << "  |"<<  setw(5) << setprecision(2) << fixed << 2 - cnrt[i].seenBy/(float)cnrt[0].seenBy  << "  |  " << cnrt[i].P  <<endl ;
        cnr_t[i].score = 0.001 - (cnr_t[i].seenBy-shift)*scale;
    }

    vector<Point> spots;

    while(true){

        resize(bitmap,display,bitmap.size()*4,0,0,INTER_NEAREST);

        for(int i = 0; i < vps.size(); i++){
            float score = 0;
            int p = 0;
            for(int j = 0; j < vps[i].corners.size(); j++){
                float ts = cnr_t[vps[i].corners[j]].score;
                if(ts > 0){
                    score += ts;
                    p++;
                }
            }
            vps[i].score = score * pow(0.9,p);
        }

        sort(vps.begin(),vps.end(),[](vispos a, vispos b){
            return a.score > b.score;
        });
        if(vps[0].score == 0){break;}
        scale = 255 / vps[0].score;
        for(int i = 0; i < vps.size(); i++){
            //cout <<  setw(5) << vps[i].score << "  |" << vps[i].pos  <<endl ;
            int x = vps[i].pos.x;
            int y = vps[i].pos.y;
            int color = vps[i].score * scale;
            rectangle(display, Point(4*x,4*y),Point(4*x+3,4*y+3),Scalar(255,255-color,255-color),FILLED);
        }
        spots.push_back(vps[0].pos*4);
        for(int i = 0; i < spots.size(); i++){
            circle(display,spots[i],2,Scalar(0,0,255),FILLED);
        }

        for(int i = 0; i < vps[0].corners.size(); i++){
            cnr_t[vps[0].corners[i]].score = 0;
        }
        cout << endl;
        cout << "display";

        imshow("map", display);
        waitKey(0);

    }

}

bool sort_point(Point a, Point b){
    if(a.x == b.x)
        return a.y < b.y;
    return a.x < b.x;
}


void mp::brushfire(){

    // brush fire begin
    vector<Point> cps;
    vector<Point> t_cps;
    vector<Point> peaks;

    int rows = display.rows;
    int cols = display.cols;

    for (int r = 3; r < rows-2; r++) {
        for (int c = 3; c < cols-2; c++) {
            if(display.at<Vec3b>(r,c)[2] == 0){
                cps.push_back(Point(c,r));
            }
        }
    }
    int color = 254;
    while(cps.size() > 0) {
        for(int i = 0; i < cps.size(); i++){
            bool inserted = false;
            for(int dx = -1; dx < 2; dx++){
                for(int dy = -1; dy < 2; dy++){
                    if(dx != 0 || dy !=0){
                        Point p(cps[i].x+dx,cps[i].y+dy);
                        auto pixel = display.at<Vec3b>(p)[2];
                        if(pixel == 255){
                            display.at<Vec3b>(p) = Vec3b(0,127+(255-color)/2,color/2);
                            t_cps.push_back(p);
                            inserted = true;
                        }if(pixel == color/2 || pixel == 0) inserted = true;
                    }
                }
            }
            if(!inserted){
                peaks.push_back(cps[i]);
                //display.at<Vec3b>(cps[i]) = Vec3b(254,1,1);
            }

        }
        color-=5;
        cps = t_cps;
        t_cps.clear();
        //cout << endl;
        //cout << "display brushfire";
        imshow("map", display);
        waitKey(20);
    }

    // brush fire done

    sort( peaks.begin(), peaks.end() , &sort_point );
    peaks.erase( unique( peaks.begin(), peaks.end() ), peaks.end() );

    for(int i = 0; i < peaks.size(); i++){
      //  display.at<Vec3b>(peaks[i]) = Vec3b(255,255,255);
    }

    // identify line segments
    vector<vector<Point>> lines;

    while(peaks.size() > 1){

        Point sp(peaks[0]);
        Point cp(sp);
        peaks.erase(peaks.begin());

        for(auto ittr = peaks.begin(); ittr != peaks.end(); ittr++){
            Point tp = *ittr;
            if( abs(tp.x-cp.x) <= 2 && abs(tp.y-cp.y) <= 2){
                cp = tp;
                peaks.erase(ittr);
                ittr = peaks.begin();
            }
        }

        if(cp != sp){
            lines.push_back(vector<Point>());
            lines[lines.size() - 1].push_back(sp);
            lines[lines.size() - 1].push_back(cp/*-Point(1,1)*/);
        };
    }

    int test = 0;
    auto ittr = lines.begin();
    while( ittr != lines.end()){
        test++;
        ittr++;
        Point a = (*ittr).at(0);
        Point b = (*ittr).at(1);
        double n = norm(a-b);
        if(n < 4){
            ittr = lines.erase(ittr);
        }


        if(ittr == lines.end()) cout << "\n\n DONE! \n\n";
    }

    // draw line segments
    for(int i = 0; i < lines.size(); i++){
        //line(display,lines[i][0],lines[i][1],Scalar(rand() % 235 + 20, rand() % 235 + 20, rand() % 235 + 20),2);
        line(display,lines[i][0],lines[i][1],Scalar(255,255,255),2);
        imshow("map", display);
        waitKey(20);
    }

    // some new
    for(int i = 0; i < lines.size(); i++){
        float dgmin0 = 70;
        float dgmin1 = 70;
        Point pgi0(0,0);
        Point pgj0(0,0);
        Point pgi1(0,0);
        Point pgj1(0,0);
        for(int j = 0; j < lines.size(); j++){
            if(i == j) continue;
            int d0 = norm(lines[i][0]-lines[j][0]);
            int d1 = norm(lines[i][0]-lines[j][1]);
            int d2 = norm(lines[i][1]-lines[j][0]);
            int d3 = norm(lines[i][1]-lines[j][1]);
            double dmin = min({d0,d1,d2,d3});

            if(dmin < dgmin0 || dmin < dgmin1){
                Point pi;
                Point pj;
                int n = -1;

                if(d0 == dmin && dmin < dgmin0){
                    pi = lines[i][0];
                    pj = lines[j][0];
                    n  = 0;
                    dgmin0 = dmin;
                } else if(d1 == dmin && dmin < dgmin0){
                    pi = lines[i][0];
                    pj = lines[j][1];
                    n  = 0;
                    dgmin0 = dmin;
                } else if(d2 == dmin && dmin < dgmin1){
                    pi = lines[i][1];
                    pj = lines[j][0];
                    n  = 1;
                    dgmin1 = dmin;
                } else if(d3 == dmin && dmin < dgmin1){
                    pi = lines[i][1];
                    pj = lines[j][1];
                    n  = 1;
                    dgmin1 = dmin;
                }
                if(n != -1){
                    LineIterator li(display, pi, pj, 4);
                    bool connected = true;

                    for(int q = 0; q < li.count; q++,++li){
                        if(((const Vec<uchar,3>) *li)[2] == 0){
                            connected = false;
                            break;
                        }
                    }

                    if(connected ){
                        if(n == 0){
                            pgi0 = pi;
                            pgj0 = pj;
                        } else {
                            pgi1 = pi;
                            pgj1 = pj;
                        }
                    }
                }
            }
        }
        if(norm(pgi0-pgj0) < norm(pgi1-pgj0) && pgi0.x != 0) {
            line(display,pgi0,pgj0,Scalar(255,255,255),2);
        }
        if(norm(pgi1-pgj1) < norm(pgi0-pgj1) && pgi1.x != 0) line(display,pgi1,pgj1,Scalar(255,255,255),2);
        imshow("map", display);
        waitKey(20);
    }

    cout << "\n\n -- BF DONE -- \n\n";

}

void mp::findAreas() {
  while (findCorners(bitmap_t)) {
    area_t.clear();

    for (int i = 0; i < cnr_t.size(); i++) {

      int dy = 1;
      int dx = 0;
      corner *a = &cnr_t[i];
      while ((bitmap_t.at<uchar>(a->P.y, a->P.x + (a->D.x * dx++))) != 0)
        ;

      while (!isBlackBetween(bitmap_t, a->P + Point(0, a->D.y * dy++),
                             Point(a->D.x, 0), dx - 1))
        ;

      Point ul;
      if (a->D.x > 0) {
        ul.x = a->P.x;
      } else if (a->D.x < 0) {
        ul.x = a->P.x - (dx - 2);
      }
      if (a->D.y > 0) {
        ul.y = a->P.y;
      } else if (a->D.y < 0) {
        ul.y = a->P.y - (dy - 2);
      }

      /*cout << i << ":  --------------------------------------" << endl;
      cout << "dxdy: " << dx << " ," << dy << endl;
      cout << "point a: " << a->P.x << " ," << a->P.y << endl;

      cout << "ul: " << ul.x << " ;" << ul.y << endl;*/
      area_t.push_back(Rect(ul, Size(dx - 1, dy - 1)));

      //--------------------------------------------------------------------------------
      dy = 0;
      dx = 1;

      while (bitmap_t.at<uchar>(a->P.y + (a->D.y * dy++), a->P.x) != 0)
        ;

      while (!isBlackBetween(bitmap_t, a->P + Point(a->D.x * dx++, 0),
                             Point(0, a->D.y), dy - 1))
        ;

      if (a->D.x > 0) {
        ul.x = a->P.x;
      } else if (a->D.x < 0) {
        ul.x = a->P.x - (dx - 2);
      }
      if (a->D.y > 0) {
        ul.y = a->P.y;
      } else if (a->D.y < 0) {
        ul.y = a->P.y - (dy - 2);
      }
      area_t.push_back(Rect(ul , Size(dx - 1, dy - 1)));
    }

    sort(area_t.begin(), area_t.end(), &largestArea);
    rectangle(bitmap_t, area_t[0], Scalar(0, 0, 0), FILLED);
    area.push_back(Rect(Point(area_t[0].x, area_t[0].y)*4, area_t[0].size()*4 ));

  }
}

bool mp::isBlackBetween(Mat m, Point a, Point d, int l) {
  for (int i = 0; i < l; i++) {
    if (d.y == 0) {
      if (m.at<uchar>(a.y, a.x + (d.x * i)) == 0)
        return true;
    } else {
      if (m.at<uchar>(a.y + (d.y * i), a.x) == 0)
        return true;
    }
  }
  return false;
}

void mp::displayMap() { imshow("map", display); }

void mp::drawRect() {
  for (int i = 0; i < area.size(); i++) {
    rectangle(display, area[i],
              Scalar(rand() % 235 + 20, rand() % 235 + 20, rand() % 235 + 20),
              FILLED, LINE_8);
    //imshow("map", display);
    //waitKey(0);
  }
}

void mp::findConnections(){

    for(int i = 0; i < area.size() -1; i++){
        Rect a = area[i];
        for(int j = i + 1; j < area.size(); j++){
            Rect b = area[j];
            if(     a.x + a.width  == b.x && a.y + a.height > b.y && b.y + b.height > a.y ||
                    b.x + b.width  == a.x && a.y + a.height > b.y && b.y + b.height > a.y ||
                    a.y + a.height == b.y && a.x + a.width  > b.x && b.x + b.width  > a.x ||
                    b.y + b.height == a.y && a.x + a.width  > b.x && b.x + b.width  > a.x
                    ){

                line(display,Point(a.x + a.width/2, a.y + a.height/2),Point(b.x + b.width/2, b.y + b.height/2),Scalar(255,0,0),1,LINE_AA);
            }
        }
    }
}


bool mp::largestArea(Rect const &a, Rect const &b) {
  return (a.area() > b.area());
}

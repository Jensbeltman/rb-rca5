#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "mp.h"

#include "cv_operators.cpp"
#include "AvlTree.h"
#include "indxavltree.h"

#include "graph.h"



using namespace std;
using namespace cv;
/*
bool operator==(const cv::Point& l, const cv::Point& r)
{
    return (l.x == r.x && l.x == r.y);
}

bool operator<(const Point& l, const Point& r)
{
    if(l.x == r.x)
        return (l.y < r.y);
    return (l.x < r.x);
}

bool operator>(const Point& l, const Point& r)
{
    if(l.x == r.x)
        return (l.y > r.y);
    return (l.x > r.x);
}
*/
int main(int _argc, char **_argv) {

    IndxAvlTree<Point, float> adjp;

    IndxAvlTree<int, float> adj;

//    adj.insert(1,4.9);
//    adj.insert(2,3.4);
//    adj.insert(3,5.7);
//    adj.insert(4,1.5);
//    adj.insert(5,3.4);
//    adj.insert(6,3.9);
//    adj.insert(7,5.4);
//    adj.insert(8,1.7);
//    adj.insert(9,3.5);
//    adj.insert(10,4.4);

    adjp.insert(Point(4,9),4.9);
    adjp.insert(Point(3,4),3.4);
    adjp.insert(Point(5,7),5.7);
    adjp.insert(Point(1,5),1.5);
    adjp.insert(Point(3,4),3.4);
    adjp.insert(Point(3,9),3.9);
    adjp.insert(Point(5,4),5.4);
    adjp.insert(Point(1,7),1.7);
    adjp.insert(Point(3,5),3.5);
    adjp.insert(Point(4,4),4.4);

    adjp.printTree();

    cout << adjp.find(Point(4,4))->data << endl;


    /*AvlTree<edge> tree;

    vertex a(2,3);
    vertex b(3,3);
    vertex c(3,2);
    vertex d(2,2);
    vertex e(4,3);
    vertex f(2,3);

    e.adj.insert(edge(&a,2.2));
    e.adj.insert(edge(&c,2.4));

    tree.insert(edge(&a,2.2));
    tree.insert(edge(&b,4.5));
    tree.insert(edge(&c,7.6));
    tree.insert(edge(&d,8.9));
    tree.insert(edge(&e,4.2));
    tree.insert(edge(&f,2.1));

    adj.insert(Point(4,9),4.9);
    adj.insert(Point(3,4),3.4);
    adj.insert(Point(5,7),5.7);
    adj.insert(Point(1,5),1.5);
    adj.insert(Point(3,4),3.4);

    if(tree.contains(edge(&f,2.1))) cout << "\n\nFound it\n\n";

    AvlTree<vertex*> vertices;

    vertices.insert(&a);
    vertices.insert(&b);
    vertices.insert(&c);
    vertices.insert(&d);
    vertices.insert(&e);
    vertices.insert(&f);

    vertices.printTree();

    if(vertices.contains(&e)) cout << "\n\nFound it\n\n";
    vertex* ep = vertices.find(&e);

    ep->adj.insert(edge(&b,8.9));

    int s = ep->adj.size();
    edge edg = e.adj.at(1);
    for(unsigned int i = 0; i < e.adj.size(); i++)
        cout << e.adj.at(i);

*/
  cv::Mat kort =
	  imread("../models/bigworld/meshes/floor_plan.png", IMREAD_COLOR);

  mp krt(kort);

  //krt.drawRect();
  //krt.connectCorners();
  krt.brushfire();
  while (1) {
    krt.displayMap();


    waitKey(0);
  }

}

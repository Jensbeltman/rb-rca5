#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "mp.h"

#include "AvlTree.h"
#include "graph.h"



using namespace std;
// using namespace cv;

int main(int _argc, char **_argv) {


    AvlTree<edge> tree;

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

    tree.printTree();

    if(tree.contains(edge(&f,2.1))) cout << "\n\nFound it\n\n";

    AvlTree<vertex> vertices;

    vertices.insert(a);
    vertices.insert(b);
    vertices.insert(c);
    vertices.insert(d);
    vertices.insert(e);
    vertices.insert(f);

    vertices.printTree();

    if(vertices.contains(e)) cout << "\n\nFound it\n\n";
    vertex ep = vertices.find(e);

    int s = ep.adj.size();
    edge edg = ep.adj.at(1);
    for(unsigned int i = 0; i < ep.adj.size(); i++)
        cout << ep.adj.at(i);


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

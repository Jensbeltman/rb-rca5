#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cv_operators.h"
#include "indxavltree.h"

using namespace cv;

 class Graph
{
public:
    Graph();

    void add(Point v);
    void remove(Point v);

    void connect(Point v, Point u, double weight);
    void disconnect(Point v, Point u);

    void print(Mat img);
    vector<array<Point,2>> lines();

private:
    IndxAvlTree<Point, IndxAvlTree<Point, float>*  > vertices;
};

#endif // GRAPH_H

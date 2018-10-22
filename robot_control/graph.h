#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <opencv2/opencv.hpp>
//#include "cv_operators.h"


#include "indxavltree.h"



using namespace cv;

/*
struct edge;

struct vertex {
    int x;
    int y;
    AvlTree<edge> adj;

    vertex(int x, int y) : x{x}, y{y}{}

    bool operator==(const vertex& e) const
    {
        return (x == e.x && y == e.y);
    }

    bool operator<(const vertex& e) const
    {
        if(x == e.x)
            return (y < e.y);
        return (x < e.x);
    }

    bool operator>(const vertex& e) const
    {
        if(x == e.x)
            return (y > e.y);
        return (x > e.x);
    }
    friend std::ostream& operator<<(std::ostream &os, const vertex &e) {
        os << "[" << e.x << "," << e.y << "]";
        return os;
    }
};

struct edge {

    vertex* vptr;
    double weight = 1;

    edge(vertex* el, double w) : vptr{el}, weight {w} {}

    bool operator==(const edge& e) const
    {
        return (*vptr == *(e.vptr));
    }

    bool operator<(const edge& e) const
    {
        return (*vptr < *(e.vptr));
    }

    bool operator>(const edge& e) const
    {
        return (*vptr > *(e.vptr));
    }

    friend std::ostream& operator<<(std::ostream &os, const edge &e) {
        os << *(e.vptr);
        return os;
    }
};
*/

class Graph
{
public:
    Graph();

    void add(cv::Point v);
    void remove(cv::Point v);

    void connect(cv::Point v, cv::Point u, double weight);

private:
    IndxAvlTree<Point, IndxAvlTree<Point, float>> vertices;
};

#endif // GRAPH_H

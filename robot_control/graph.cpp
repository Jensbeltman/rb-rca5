#include "graph.h"

#include <opencv2/opencv.hpp>

#include <vector>
#include "AvlTree.h"

using namespace cv;




Graph::Graph()
{
    AvlTree<int> adj_list();
}

void Graph::remove(Point v)
{
   /* vertex vtx = vertices.
    for(int
        )*/
}

void Graph::add(Point v)
{
    vertices.insert(vertex(v.x,v.y));
}


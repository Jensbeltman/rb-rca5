#include "graph.h"


Graph::Graph()
{
    
}

void Graph::remove(Point v)
{
    IndxAvlTree<Point,float>* adj = vertices.find(v)->data;
    for(int i = 0; i < adj->size(); i++){
        vertices.find(adj->at(i)->index)->data->remove(v);
    }
    vertices.remove(v);
}

void Graph::connect(Point v, Point u, double weight)
{
    IndxAvlTree<Point,float>* v_adj = vertices.find(v)->data;
    v_adj->insert(u,weight);

    IndxAvlTree<Point,float>* u_adj = vertices.find(u)->data;
    u_adj->insert(v,weight);

}

void Graph::disconnect(Point v, Point u)
{
    IndxAvlTree<Point,float>* v_adj = vertices.find(v)->data;
    v_adj->remove(u);

    IndxAvlTree<Point,float>* u_adj = vertices.find(u)->data;
    u_adj->remove(v);
}

void Graph::print(Mat img)
{
    for(unsigned int i = 0; i < vertices.size(); i++){
        IndxAvlNode<Point,IndxAvlTree<Point,float>*>* vert = vertices.at(i);
        for(unsigned int j = 0; j < vert->data->size(); j++){
            if(vert->index > vert->data->at(j)->index)
                line(img, vert->index,vert->data->at(j)->index,Scalar(255,255,255));
        }
    }
}

vector<array<Point,2>> Graph::lines()
{
    vector<array<Point,2>> lines;
    for(unsigned int i = 0; i < vertices.size(); i++){
        IndxAvlNode<Point,IndxAvlTree<Point,float>*>* vert = vertices.at(i);
        for(unsigned int j = 0; j < vert->data->size(); j++){
            if(vert->index > vert->data->at(j)->index)
                lines.push_back({vert->index, vert->data->at(j)->index});
        }
    }
    return lines;
}

void Graph::add(Point v)
{
    vertices.insert(v,new IndxAvlTree<Point,float>);
}


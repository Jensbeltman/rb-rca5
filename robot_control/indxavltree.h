#ifndef INDXAVLTREE_H
#define INDXAVLTREE_H

#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

template <typename Ti, typename Td>
class IndxAvlNode
{
public:
    IndxAvlNode(Ti i,Td d, IndxAvlNode *lt, IndxAvlNode *rt, int h = 0)
        : index{ i }, data{ d }, left{ lt }, right{ rt }, height{ h } { }

    ~IndxAvlNode() {}

    Ti index;
    Td data;
    IndxAvlNode<Ti, Td>   *left;
    IndxAvlNode<Ti, Td>   *right;
    int height;
};

template <typename Ti, typename Td>
class IndxAvlTree
{
public:
    IndxAvlTree();
    IndxAvlTree(const IndxAvlTree & rhs);
    IndxAvlTree(IndxAvlTree && rhs);
    ~IndxAvlTree();


    const Ti& findMin() const;
    const Ti& findMax() const;
    bool contains(const Ti& index) const;
    IndxAvlNode<Ti, Td>* find(const Ti& index) const;
    bool isEmpty() const;
    void printTree() const;
    void makeEmpty();
    void insert(const Ti& index, const Td& data);
    void insert(Ti&& index, Td&& data);
    void remove(const Ti& index);
    unsigned int size();

    IndxAvlNode<Ti, Td> * at(int i);

private:

    IndxAvlNode<Ti, Td> *root;
    unsigned int _size;

    void insert(const Ti& x, const Td& d, IndxAvlNode<Ti, Td> * & t);
    void insert(Ti&& x, Td&& d, IndxAvlNode<Ti, Td> * & t);
    void remove(const Ti& x, IndxAvlNode<Ti, Td> * & t);

    static const int ALLOWED_IMBALANCE = 1;

    // Assume t is balanced or within one of being balanced
    void balance(IndxAvlNode<Ti, Td> * & t);
    IndxAvlNode<Ti, Td>* findMin(IndxAvlNode<Ti,Td> *t) const;
    IndxAvlNode<Ti, Td>* findMax(IndxAvlNode<Ti, Td> *t) const;
    bool contains(const Ti& index, IndxAvlNode<Ti, Td> *t) const;
    IndxAvlNode<Ti, Td>* find(const Ti& index, IndxAvlNode<Ti, Td> *t) const;
    void makeEmpty(IndxAvlNode<Ti, Td> * & t);
    void printTree(IndxAvlNode<Ti, Td> *t) const;
    IndxAvlNode<Ti, Td>* clone(IndxAvlNode<Ti, Td> *t) const;
    int height(IndxAvlNode<Ti, Td> *t) const;
    int max(int lhs, int rhs) const;
    void rotateWithLeftChild(IndxAvlNode<Ti, Td> * & k2);
    void rotateWithRightChild(IndxAvlNode<Ti, Td> * & k1);
    void doubleWithLeftChild(IndxAvlNode<Ti, Td> * & k3);
    void doubleWithRightChild(IndxAvlNode<Ti, Td> * & k1);

    IndxAvlNode<Ti, Td>* at(int* i, IndxAvlNode<Ti, Td> *t);

};

#include "indxavltree.cpp"

//template <cv::Point, float>
#endif // INDXAVLTREE_H

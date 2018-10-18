#ifndef AVL_TREE_H
#define AVL_TREE_H
#include <algorithm>
#include <iostream>

using namespace std;

template <typename T>
class AvlNode
{
public:
    AvlNode(T ele, AvlNode *lt, AvlNode *rt, int h = 0)
		: element{ ele }, left{ lt }, right{ rt }, height{ h } { }

	//AvlNode(int && ele, AvlNode *lt, AvlNode *rt, int h = 0)
	//	: element{ std::move(ele) }, left{ lt }, right{ rt }, height{ h } { }
	~AvlNode() {}
 
    T element;
    AvlNode<T>   *left;
    AvlNode<T>
    *right;
	int       height;
};

template <typename T>
class AvlTree
{
public:
    AvlTree();
    AvlTree(const AvlTree & rhs);
    AvlTree(AvlTree && rhs);
    ~AvlTree();
	

	const int& findMin() const;
	const int& findMax() const;
    bool contains(const T& x) const;
    T find(const T& x) const;
	bool isEmpty() const;
	void printTree() const;
	void makeEmpty();
    void insert(const T& x);
    void insert(T&& x);
    void remove(const T& x);
    unsigned int size();

    T at(int i);
    T &operator[](int i);
	
private:
	
    AvlNode<T> *root;
    unsigned int _size;

    void insert(const T& x, AvlNode<T> * & t);
    void insert(T&& x, AvlNode<T> * & t);
    void remove(const T& x, AvlNode<T> * & t);
	
	static const int ALLOWED_IMBALANCE = 1;

	// Assume t is balanced or within one of being balanced
    void balance(AvlNode<T> * & t);
    AvlNode<T>* findMin(AvlNode<T> *t) const;
    AvlNode<T>* findMax(AvlNode<T> *t) const;
    bool contains(const T& x, AvlNode<T> *t) const;
    T find(const T& x, AvlNode<T> *t) const;
    void makeEmpty(AvlNode<T> * & t);
    void printTree(AvlNode<T> *t) const;
    AvlNode<T>* clone(AvlNode<T> *t) const;
    int height(AvlNode<T> *t) const;
	int max(int lhs, int rhs) const;
    void rotateWithLeftChild(AvlNode<T> * & k2);
    void rotateWithRightChild(AvlNode<T> * & k1);
    void doubleWithLeftChild(AvlNode<T> * & k3);
    void doubleWithRightChild(AvlNode<T> * & k1);

    T at(int* i, AvlNode<T> *t);
	
};

#include "AvlTree.cpp"
#endif



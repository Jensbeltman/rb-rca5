
#ifndef AVL_TREE_CPP
#define AVL_TREE_CPP
#include "AvlTree.h"

template <typename T>
AvlTree<T>::AvlTree() : root{ nullptr }, _size {0}
{ }

template <typename T>
AvlTree<T>::AvlTree(const AvlTree & rhs) : root{ nullptr }
{
	root = clone(rhs.root);
    _size = rhs._size;
}

template <typename T>
AvlTree<T>::AvlTree(AvlTree && rhs) : root{ rhs.root }
{
	rhs.root = nullptr;
}

template <typename T>
AvlTree<T>::~AvlTree()
{
	makeEmpty();
}
template <typename T>
const int & AvlTree<T>::findMin() const
{
	if (isEmpty())
		return -1;
	return findMin(root)->element;
}
template <typename T>
const int& AvlTree<T>::findMax() const
{
	if (isEmpty())
		return -1;
	return findMax(root)->element;
}
template <typename T>
bool AvlTree<T>::contains(const T& x) const
{
    return contains(x, root);
}

template<typename T>
T AvlTree<T>::find(const T &x) const
{
    return find(x, root);
}
template <typename T>
bool AvlTree<T>::isEmpty() const
{
	return root == nullptr;
}
template <typename T>
void AvlTree<T>::printTree() const
{
	if (isEmpty())
		cout << "Empty tree" << endl;
	else
		printTree(root);
}
template <typename T>
void AvlTree<T>::makeEmpty()
{
    _size = 0;
	makeEmpty(root);
}
template <typename T>
void AvlTree<T>::insert(const T& x)
{
	insert(x, root);
}
template <typename T>
void AvlTree<T>::insert(T&& x)
{
	insert(std::move(x), root);
}
template <typename T>
void AvlTree<T>::remove(const T& x)
{
    remove(x, root);
}

template<typename T>
unsigned int AvlTree<T>::size()
{
    return _size;
}

template<typename T>
T AvlTree<T>::at(int i)
{
    int ip = i;
    AvlNode<T> * ret = at(&ip, root);
    return ret->element;
}

template <typename T>
void AvlTree<T>::insert(const T& x, AvlNode<T> * & t)
{
    if (t == nullptr){
        _size+=1;
        t = new AvlNode<T>{ x, nullptr, nullptr };
    }else if (x < t->element)
		insert(x, t->left);
	else if (t->element < x)
		insert(x, t->right);

	balance(t);
}
template <typename T>
void AvlTree<T>::insert(T && x, AvlNode<T> * & t)
{
    if (t == nullptr){
       _size+=1;
        t = new AvlNode<T>{ std::move(x), nullptr, nullptr };
    }else if (x < t->element)
		insert(std::move(x), t->left);
	else if (t->element < x)
		insert(std::move(x), t->right);

	balance(t);
}
template <typename T>
void AvlTree<T>::remove(const T& x, AvlNode<T> * & t)
{
	if (t == nullptr)
		return;   // Item not found; do nothing

	if (x < t->element)
		remove(x, t->left);
	else if (t->element < x)
		remove(x, t->right);
	else if (t->left != nullptr && t->right != nullptr) // Two children
	{
		t->element = findMin(t->right)->element;
		remove(t->element, t->right);
	}
	else
	{
        _size--;
        AvlNode<T> *oldNode = t;
		t = (t->left != nullptr) ? t->left : t->right;
		delete oldNode;
	}

	balance(t);
}
template <typename T>
void AvlTree<T>::balance(AvlNode<T> * & t)
{
	if (t == nullptr)
		return;

	if (height(t->left) - height(t->right) > ALLOWED_IMBALANCE)
		if (height(t->left->left) >= height(t->left->right))
			rotateWithLeftChild(t);
		else
			doubleWithLeftChild(t);
	else
		if (height(t->right) - height(t->left) > ALLOWED_IMBALANCE)
			if (height(t->right->right) >= height(t->right->left))
				rotateWithRightChild(t);
			else
				doubleWithRightChild(t);

	t->height = max(height(t->left), height(t->right)) + 1;
}
template <typename T>
AvlNode<T> * AvlTree<T>::findMin(AvlNode<T> *t) const
{
	if (t == nullptr)
		return nullptr;
	if (t->left == nullptr)
		return t;
	return findMin(t->left);
}
template <typename T>
AvlNode<T> * AvlTree<T>::findMax(AvlNode<T> *t) const
{
	if (t != nullptr)
		while (t->right != nullptr)
			t = t->right;
	return t;
}
template <typename T>
bool AvlTree<T>::contains(const T& x, AvlNode<T> *t) const
{
	if (t == nullptr)
		return false;
	else if (x < t->element)
		return contains(x, t->left);
	else if (t->element < x)
		return contains(x, t->right);
	else
        return true;    // Match
}

template<typename T>
T AvlTree<T>::find(const T &x, AvlNode<T> *t) const
{
    if (t == nullptr)
        return x;
    else if (x < t->element)
        return find(x, t->left);
    else if (t->element < x)
        return find(x, t->right);
    else
        return t->element;    // Match
}
template <typename T>
void AvlTree<T>::makeEmpty(AvlNode<T> * & t)
{
	if (t != nullptr)
	{
		makeEmpty(t->left);
		makeEmpty(t->right);
		delete t;
	}
	t = nullptr;
}
template <typename T>
void AvlTree<T>::printTree(AvlNode<T> *t) const
{
	if (t != nullptr)
	{
		printTree(t->left);
		cout << t->element << endl;
		printTree(t->right);
	}
}
template <typename T>
AvlNode<T> * AvlTree<T>::clone(AvlNode<T> *t) const
{
	if (t == nullptr)
		return nullptr;
	else
        return new AvlNode<T>{ t->element, clone(t->left), clone(t->right), t->height };
}
template <typename T>
int AvlTree<T>::height(AvlNode<T> *t) const
{
	return t == nullptr ? -1 : t->height;
}
template <typename T>
int AvlTree<T>::max(int lhs, int rhs) const
{
	return lhs > rhs ? lhs : rhs;
}
template <typename T>
void AvlTree<T>::rotateWithLeftChild(AvlNode<T> * & k2)
{
    AvlNode<T> *k1 = k2->left;
	k2->left = k1->right;
	k1->right = k2;
	k2->height = max(height(k2->left), height(k2->right)) + 1;
	k1->height = max(height(k1->left), k2->height) + 1;
	k2 = k1;
}
template <typename T>
void AvlTree<T>::rotateWithRightChild(AvlNode<T> * & k1)
{
    AvlNode<T> *k2 = k1->right;
	k1->right = k2->left;
	k2->left = k1;
	k1->height = max(height(k1->left), height(k1->right)) + 1;
	k2->height = max(height(k2->right), k1->height) + 1;
	k1 = k2;
}
template <typename T>
void AvlTree<T>::doubleWithLeftChild(AvlNode<T> * & k3)
{
	rotateWithRightChild(k3->left);
	rotateWithLeftChild(k3);
}
template <typename T>
void AvlTree<T>::doubleWithRightChild(AvlNode<T> * & k1)
{
	rotateWithLeftChild(k1->right);
	rotateWithRightChild(k1);
}

template<typename T>
AvlNode<T>* AvlTree<T>::at(int* i, AvlNode<T> *t)
{
    if (t == nullptr || *i < 0) return nullptr;

    if(*i == 0) {
        return t;
    }
    *i = *i - 1;
    AvlNode<T>* ret;
    ret = at(i,t->left);
    if(ret != nullptr) return ret;
    return at(i,t->right);
}


#endif

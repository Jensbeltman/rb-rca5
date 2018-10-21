
#ifndef INDX_AVL_TREE_CPP
#define INDX_AVL_TREE_CPP
#include "indxavltree.h"

template <typename Ti, typename Td>
IndxAvlTree<Ti, Td>::IndxAvlTree() : root{ nullptr }, _size {0}
{ }

template <typename Ti, typename Td>
IndxAvlTree<Ti, Td>::IndxAvlTree(const IndxAvlTree & rhs) : root{ nullptr }
{
    root = clone(rhs.root);
    _size = rhs._size;
}

template <typename Ti, typename Td>
IndxAvlTree<Ti,  Td>::IndxAvlTree(IndxAvlTree && rhs) : root{ rhs.root }
{
    rhs.root = nullptr;
}

template <typename Ti, typename Td>
IndxAvlTree<Ti,  Td>::~IndxAvlTree()
{
    makeEmpty();
}

template <typename Ti, typename Td>
const Ti & IndxAvlTree<Ti,  Td>::findMin() const
{
    if (isEmpty())
        return -1;
    return findMin(root)->index;
}

template <typename Ti, typename Td>
const Ti & IndxAvlTree<Ti,  Td>::findMax() const
{
    if (isEmpty())
        return -1;
    return findMax(root)->index;
}

template <typename Ti, typename Td>
bool IndxAvlTree<Ti,  Td>::contains(const Ti& x) const
{
    return contains(x, root);
}

template<typename Ti, typename Td>
IndxAvlNode<Ti, Td> IndxAvlTree<Ti,  Td>::find(const Ti &x) const
{
    return find(x, root);
}
template <typename Ti, typename Td>
bool IndxAvlTree<Ti,  Td>::isEmpty() const
{
    return root == nullptr;
}
template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::printTree() const
{
    if (isEmpty())
        cout << "Empty tree" << endl;
    else
        printTree(root);
}
template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::makeEmpty()
{
    _size = 0;
    makeEmpty(root);
}
template <typename Ti, typename Td>

void IndxAvlTree<Ti,  Td>::insert(const Ti& x, const Td& d)
{
    insert(x, d, root);
}
template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::insert(Ti&& x, Td&& d)
{
    insert(std::move(x), std::move(d), root);
}
template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::remove(const Ti& x)
{
    remove(x, root);
}

template<typename Ti, typename Td>
unsigned int IndxAvlTree<Ti,  Td>::size()
{
    return _size;
}

template<typename Ti, typename Td>
IndxAvlNode<Ti,  Td> * IndxAvlTree<Ti,  Td>::at(int i)
{
    int ip = i;
    IndxAvlNode<Ti,  Td> * ret = at(&ip, root);
    return ret;
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::insert(const Ti& x, const Td& d, IndxAvlNode<Ti,  Td> * & t)
{
    if (t == nullptr){
        _size+=1;
        t = new IndxAvlNode<Ti,  Td>{ x, d, nullptr, nullptr };
    }else if (x < t->index)
        insert(x, t->left);
    else if (t->index < x)
        insert(x, t->right);

    balance(t);
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::insert(Ti && x, Td && d, IndxAvlNode<Ti,  Td> * & t)
{
    if (t == nullptr){
        _size+=1;
        t = new IndxAvlNode<Ti,  Td>{ std::move(x), std::move(d),nullptr, nullptr };
    }else if (x < t->index)
        insert(std::move(x), std::move(d), t->left);
    else if (t->index < x)
        insert(std::move(x), std::move(d), t->right);

    balance(t);
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::remove(const Ti& x, IndxAvlNode<Ti,  Td> * & t)
{
    if (t == nullptr)
        return;   // Item not found; do nothing

    if (x < t->index)
        remove(x, t->left);
    else if (t->index < x)
        remove(x, t->right);
    else if (t->left != nullptr && t->right != nullptr) // Two children
    {
        t->index = findMin(t->right)->index;
        remove(t->index, t->right);
    }
    else
    {
        _size--;
        IndxAvlNode<Ti,  Td> *oldNode = t;
        t = (t->left != nullptr) ? t->left : t->right;
        delete oldNode;
    }

    balance(t);
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::balance(IndxAvlNode<Ti,  Td> * & t)
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

template <typename Ti, typename Td>
IndxAvlNode<Ti,  Td> * IndxAvlTree<Ti,  Td>::findMin(IndxAvlNode<Ti,  Td> *t) const
{
    if (t == nullptr)
        return nullptr;
    if (t->left == nullptr)
        return t;
    return findMin(t->left);
}

template <typename Ti, typename Td>
IndxAvlNode<Ti,  Td> * IndxAvlTree<Ti,  Td>::findMax(IndxAvlNode<Ti,  Td> *t) const
{
    if (t != nullptr)
        while (t->right != nullptr)
            t = t->right;
    return t;
}
template <typename Ti, typename Td>

bool IndxAvlTree<Ti,  Td>::contains(const Ti& x, IndxAvlNode<Ti,  Td> *t) const
{
    if (t == nullptr)
        return false;
    else if (x < t->index)
        return contains(x, t->left);
    else if (t->index < x)
        return contains(x, t->right);
    else
        return true;    // Match
}

template<typename Ti, typename Td>
IndxAvlNode<Ti, Td> IndxAvlTree<Ti,  Td>::find(const Ti &x, IndxAvlNode<Ti,  Td> *t) const
{
    if (t == nullptr)
        return x;
    else if (x < t->index)
        return find(x, t->left);
    else if (t->index < x)
        return find(x, t->right);
    else
        return t;    // Match
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::makeEmpty(IndxAvlNode<Ti,  Td> * & t)
{
    if (t != nullptr)
    {
        makeEmpty(t->left);
        makeEmpty(t->right);
        delete t;
    }
    t = nullptr;
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::printTree(IndxAvlNode<Ti,  Td> *t) const
{
    if (t != nullptr)
    {
        printTree(t->left);
        cout << t->index << endl;
        printTree(t->right);
    }
}

template <typename Ti, typename Td>
IndxAvlNode<Ti,  Td> * IndxAvlTree<Ti,  Td>::clone(IndxAvlNode<Ti,  Td> *t) const
{
    if (t == nullptr)
        return nullptr;
    else
        return new IndxAvlNode<Ti,  Td>{ t->index, clone(t->left), clone(t->right), t->height };
}

template <typename Ti, typename Td>
int IndxAvlTree<Ti,  Td>::height(IndxAvlNode<Ti,  Td> *t) const
{
    return t == nullptr ? -1 : t->height;
}

template <typename Ti, typename Td>
int IndxAvlTree<Ti,  Td>::max(int lhs, int rhs) const
{
    return lhs > rhs ? lhs : rhs;
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::rotateWithLeftChild(IndxAvlNode<Ti,  Td> * & k2)
{
    IndxAvlNode<Ti,  Td> *k1 = k2->left;
    k2->left = k1->right;
    k1->right = k2;
    k2->height = max(height(k2->left), height(k2->right)) + 1;
    k1->height = max(height(k1->left), k2->height) + 1;
    k2 = k1;
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::rotateWithRightChild(IndxAvlNode<Ti,  Td> * & k1)
{
    IndxAvlNode<Ti,  Td> *k2 = k1->right;
    k1->right = k2->left;
    k2->left = k1;
    k1->height = max(height(k1->left), height(k1->right)) + 1;
    k2->height = max(height(k2->right), k1->height) + 1;
    k1 = k2;
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::doubleWithLeftChild(IndxAvlNode<Ti,  Td> * & k3)
{
    rotateWithRightChild(k3->left);
    rotateWithLeftChild(k3);
}

template <typename Ti, typename Td>
void IndxAvlTree<Ti,  Td>::doubleWithRightChild(IndxAvlNode<Ti,  Td> * & k1)
{
    rotateWithLeftChild(k1->right);
    rotateWithRightChild(k1);
}

template<typename Ti, typename Td>
IndxAvlNode<Ti,  Td>* IndxAvlTree<Ti,  Td>::at(int* i, IndxAvlNode<Ti,  Td> *t)
{
    if (t == nullptr || *i < 0) return nullptr;

    if(*i == 0) {
        return t;
    }
    *i = *i - 1;
    IndxAvlNode<Ti,  Td>* ret;
    ret = at(i,t->left);
    if(ret != nullptr) return ret;
    return at(i,t->right);
}

#endif

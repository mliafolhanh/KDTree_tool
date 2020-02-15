/**
 * File: KDTree.h
 * -------------------------------
 * The tree build from a list of point. Using the tree to find k nearest neighbor
 */
#ifndef KDTREE_INCLUDED
#define KDTREE_INCLUDED
#include <vector>
#include <map>
#include <limits>
#include <cmath> 
#include <iostream>
#include <algorithm>
using namespace std;

template <size_t N>
class Point
{
private:
  vector<float> coords;
  int index;

public:
  Point(vector<float> p, int index_)
  {
    coords.empty();
    for (int i = 0; i < N; i++)
      coords.push_back(p[i]);
    index = index_;
  }

  float operator[](size_t index) const
  {
    return coords[index];
  }
  
  int getID()
  {
    return index;
  }

  virtual bool isSatisfied(Point<N>& other)
  {
    return true;
  }
};

template<size_t N>
float distance(const Point<N>& one, const Point<N>& two)
{
  float result = 0.0;
  for (unsigned int i = 0; i < N; i++)
    result += (one[i] - two[i]) * (one[i] - two[i]);
  return result;
}

template <typename T>
class BoundedQueue
{
private:
  std::multimap<float, T> elems;
  size_t maximumSize;

public:
  BoundedQueue(size_t maxSize)
  {
    maximumSize = maxSize;
  }

  //insert to the queue, if exceeds the max size then remove the worst one
  void enqueue(const T& value, float priority)
  {
    elems.insert(std::make_pair(priority, value));
    if (elems.size() > maximumSize)
    {
      typename std::multimap<float, T>::iterator last = elems.end();
      last--;
      elems.erase(last);
    }
  }

  //copy the best one and remove it from the queue
  T dequeue()
  {
    T result = elems.begin()->second;
    elems.erase(elems.begin());
    return result;
  }
  
  //get begin
  typename std::multimap<float, T>::iterator begin()
  {
    return elems.begin();
  }
  
   //get begin
  typename std::multimap<float, T>::iterator end()
  {
    return elems.end();
  }
  
  //get size
  size_t size()
  {
    return elems.size();
  }

  //get max_size
  size_t maxSize()
  {
    return maximumSize;
  }
  //check the queue empty or not
  bool empty()
  {
    return elems.empty();
  }

  //get the best
  float best()
  {
    return empty() ? std::numeric_limits<float>::infinity() : elems.begin()->first;
  }

  //get the worst
  float worst()
  {
    return empty() ? std::numeric_limits<float>::infinity() : elems.rbegin()->first;
  }

  vector<T> getAttribute()
  {
    vector<T> result;
    result.clear();
    for (typename std::multimap<float, T>::iterator iter = elems.begin(); iter != elems.end(); iter++)
      result.push_back(iter->second);
    return result;	
  }

};

template <size_t N>
class KDTree
{
private:
  struct Node
  {
    Point<N> point;
    Node *left;
    Node *right;
    int level;
    Node(const Point<N>& _pt, int _level):
        point(_pt), level(_level) {}
  };
  Node* root; //root node of KDTree
  size_t size_; //size of tree
  /**
   * Recursivly build a subtree that satisfy KD-Tree using point in [start, end)
   * Each level, we split points into two halves using the median of the points
   * O(n) time paritioning algorithm is used to locate the median element
   */
  Node* buildTree(vector<Point<N>>& points, int start, int end, int currentLevel);
  //Recursive free up all resoureces of subtree rooted at 'current Node'
  void freeResource(Node* currentNode);
  void nearestNeighborRecursive(
                               const Point<N>& key, 
                               Node * currentNode, 
                               BoundedQueue<Point<N>> &pQueue);
public:
  KDTree(); //construct an empty tree
  KDTree(vector<Point<N>> & points); 
  ~KDTree(); //Free up dynamically allocated resoureces;
  /**
   * Given a point and integer k, finds the k points in KDTree
   * which nearest to v
   */
  vector<Point<N>>  kNNValue(
                            Point<N>& key,
                            size_t k); 
};

template <size_t N>
KDTree<N>::KDTree():
    root(NULL), size_(0) { }

template <size_t N>
typename KDTree<N>::Node*
KDTree<N>::buildTree(
                 vector<Point<N>>& points,
                 int start,
                 int end,
                 int currentLevel)
{
  if (start >= end)
    return NULL;
  int axis = currentLevel % N ;// the axis to split on
  auto mid = (start + end)/2;
  auto cmp = [axis](const Point<N>& p1, const Point<N>& p2)
  {
    return (p1[axis] < p2[axis]);
  };
  typename vector<Point<N>>::iterator st_vec = points.begin();
  std::nth_element(st_vec + start, st_vec + mid, st_vec + end, cmp);
  //cout << "mid value:"<< points[mid][0] << " " << points[mid][1] << endl;
  while (mid > start && points[mid][axis] == points[mid - 1][axis])
    mid--;
  typename KDTree<N>::Node* newNode = new Node(points[mid], currentLevel);
  newNode->left = buildTree(points, start, mid, currentLevel + 1);
  newNode->right = buildTree(points, mid + 1, end, currentLevel + 1);
  return newNode;
}

template <size_t N>
KDTree<N>::KDTree(
              vector<Point<N> >& points) 
{
  size_ = points.size();
  root = buildTree(points, 0, size_, 0);
}

template <size_t N>
void
KDTree<N>::freeResource(
                   typename KDTree<N>::Node* currentNode)
{
  if (currentNode == NULL)
    return;
  freeResource(currentNode->left);
  freeResource(currentNode->right);
  delete currentNode;
}

template <size_t N>
KDTree<N>::~KDTree()
{
  freeResource(root);
}

template <size_t N>
void
KDTree<N>::nearestNeighborRecursive(
                        const Point<N>& key,
                        typename KDTree<N>::Node* currentNode,
                        BoundedQueue<Point<N>> &pQueue)
{
  if (currentNode == NULL)
    return;
  Point<N>& currentPoint = currentNode->point;
  if (currentPoint.isSatisfied(num, id))
  {
    pQueue.enqueue(currentPoint, distance(currentPoint, key));
  }
  int currentLevel = currentNode->level;
  bool isLeftTree;
  if (key[currentLevel%N] < currentPoint[currentLevel % N])
  {
    nearestNeighborRecursive(key, currentNode->left, pQueue, num, id);
    isLeftTree = true;
  }
  else
  {
    nearestNeighborRecursive(key, currentNode->right, pQueue, num, id);
    isLeftTree = false;
  }
  float currDistance = key[currentLevel%N] - currentPoint[currentLevel % N];
  currDistance = currDistance * currDistance;
  if (pQueue.size() < pQueue.maxSize() || currDistance < pQueue.worst())
  {
    if (isLeftTree)
      nearestNeighborRecursive(key, currentNode->right, pQueue,  num, id);
    else
      nearestNeighborRecursive(key, currentNode->left, pQueue, num, id);
  }
}

template <size_t N>
vector<Point<N>>
KDTree<N>::kNNValue(
                Point<N>& key,
                size_t k)
{
  BoundedQueue<Point<N>> pQueue(k);
  nearestNeighborRecursive(key, root, pQueue);
  vector<Point<N>> result = pQueue.getAttribute();
  return result;
}
#endif //KDTREE_INCLUDED

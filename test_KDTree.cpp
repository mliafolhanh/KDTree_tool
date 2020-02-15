#include "KDTree.h"
#include <vector>
#include <iostream>

using namespace std;
int main()
{
  vector<Point<2>> points;
  points.clear();
  for (int i = 0; i < 10; i++)
  {
    vector<float> v;
    v.clear();
    v.push_back(0);
    v.push_back(i);
    points.push_back(Point<2>(v, i));
  }
  for (int i = 0; i < 10; i++)
  {
    cout << points[i][0] << " " << points[i][1] << endl;
  }

  KDTree<2> kdTree = KDTree<2>(points);
  for (int i = 0; i < 10; i++)
  {
    cout << points[i][0] << " " << points[i][1] << endl;
  }
  vector<float> a{0, 9};
  Point<2> p = Point<2>(a, -1);
  vector<Point<2>> neighbors = kdTree.kNNValue(p, 5);
  cout << "Search neighbors" << endl;
  for (int i = 0; i < 5; i++)
    cout << neighbors[i][0] << " " << neighbors[i][1] << endl;
  return 0;
}

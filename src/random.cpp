#include <algorithm>
#include <bits/stdc++.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

/**
 * Our graph will be defined as follows
 *  Node number
 *    1 2 3 4
 *    _ _ _ _
 * 1 |
 * 2 |
 * 3 |
 * 4 |
 *
 *   Nodes without any connection will be marked as -1
 *   The output will be a symmetric matrix with diagonal elements as -1 because
 *there is no self looping
 **/

/**
 * Brief: Check if the co-linear lines have an intersection such that the point
 *of the edge 2
 *          lies on edge1
 * param[in]: a - Co-ordinate of edge 1 vertex 1
 * param[in]: b - Co-ordinate of edge 1 vertex 2
 * param[in]: c - Co-ordinate of edge 2 vertex 1
 * return : bool - True if the edges intersects
 **/
bool pointOnSegment(std::tuple<int, int> a, std::tuple<int, int> b,
                    std::tuple<int, int> c) {
  if (std::get<0>(b) <= std::max(std::get<0>(a), std::get<0>(c)) &&
      std::get<0>(b) >= std::min(std::get<0>(a), std::get<0>(c)) &&
      std::get<1>(b) <= std::max(std::get<1>(a), std::get<1>(c)) &&
      std::get<1>(b) >= std::min(std::get<1>(a), std::get<1>(c)))
    return true;
  return false;
}

/**
 * Brief: Finds the orientation of edges (clockwise, counterclockwise or
 *co-linear)
 * param[in]: a - Co-ordinate of edge 1 vertex 1
 * param[in]: b - Co-ordinate of edge 1 vertex 2
 * param[in]: c - Co-ordinate of edge 2 vertex 1
 * return : int - clockwise: 1, counterclockwise: 2, co-linear: 0
 **/
int checkOrientation(std::tuple<int, int> a, std::tuple<int, int> b,
                     std::tuple<int, int> c) {
  int orientation =
      (std::get<1>(b) - std::get<1>(a)) * (std::get<0>(c) - std::get<0>(b)) -
      (std::get<0>(b) - std::get<0>(a)) * (std::get<1>(c) - std::get<1>(b));
  if (orientation == 0)
    return 0; // Co-linear
  else if (orientation > 0)
    return 1; // Clockwise
  else
    return 2; // Anticlockwise
}

/**
 * Brief: Checks if the two edges are intersecting
 * param[in]: a - Co-ordinate of edge 1 vertex 1
 * param[in]: b - Co-ordinate of edge 1 vertex 2
 * param[in]: c - Co-ordinate of edge 2 vertex 1
 * param[in]: d - Co-ordinate of edge 2 vertex 2
 * return : bool - True if the edges intersects
 **/
bool isIntersecting(std::tuple<int, int> a, std::tuple<int, int> b,
                    std::tuple<int, int> c, std::tuple<int, int> d) {
  // Reference material:
  // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
  int case1 = checkOrientation(a, b, c);
  int case2 = checkOrientation(a, b, d);
  int case3 = checkOrientation(c, d, a);
  int case4 = checkOrientation(c, d, b);

  // If the orientation of the edges in different cases is different then the
  // edges intersects
  if (case1 != case2 && case3 != case4)
    return true;
  // Checking special case if there are co-linear lines
  if (case1 == 0 && pointOnSegment(a, c, b))
    return true;
  if (case2 == 0 && pointOnSegment(a, d, b))
    return true;
  if (case3 == 0 && pointOnSegment(c, a, d))
    return true;
  if (case4 == 0 && pointOnSegment(c, b, d))
    return true;
  return false;
}

/**
 * Brief: Apply the heuristic by removing the intersections in the tour
 * param[in]: final_path - A vector of vetrices describing path
 * param[in]: vertices - The graph representing the keys and co-ordinates of
 *vertex
 * return : final_path - A vector of vertices describing tour after applying
 *heuristics
 **/
std::vector<int>
apply_heuristic_to_tour(std::vector<int> final_path,
                        std::map<int, std::tuple<int, int>> vertices) {
  std::vector<int> new_path;
  new_path = final_path;
  bool break_inner_loop = false;
  bool break_outer_loop = false;
  int count = 0;
  int terminating_iteration = 0;
  while (count != -1) {
    for (int i = 0; i < final_path.size() - 1;
         i++) // Check every edge with every other edge in the tour
    {
      int a = final_path[i];
      int b = final_path[i + 1];
      for (int j = 1; j < final_path.size() - 1; j++) {
        if (a != final_path[j] && a != final_path[j - 1] &&
            a != final_path[j + 1]) {
          int c = final_path[j];
          int d = final_path[j + 1];
          if (isIntersecting(
                  vertices[a], vertices[b], vertices[c],
                  vertices[d])) { // If edges intersect then reconnect the paths
            if (i < j)
              std::reverse(new_path.begin() + i + 1,
                           new_path.begin() + (j + 1));
            else
              std::reverse(new_path.begin() + j + 1,
                           new_path.begin() + (i + 1));

            final_path =
                new_path; // Use the new path for removing more intersections
            break_inner_loop = true;
          }
        }
        if (break_inner_loop) {
          break_inner_loop = false;
          break_outer_loop = true;
          break;
        }
      }
      if (break_outer_loop) {
        break_outer_loop = false;
        break;
      }
      if (i == final_path.size() - 2)
        count = -1; // Run untill there are no intersections in the tour path
    }
    terminating_iteration++;
    if (terminating_iteration > 1000)
      break;
  }
  return final_path;
}

/**
 * Brief: Finds the route with MST using DFS
 * param[out]: final_path - A vector of vetrices describing path
 * param[in]: number_of_node - The number of vertices in graph
 * param[in]: parent_child_list - Same as parent list but shows the list of
 *child vertex each parent has
 * param[in]:  parent_list - A 1-D array of the parent-child adjancency list
 *(MST of graph)
 * return : cost - Total cost of the final tour
 **/
float get_final_path(std::vector<int> &final_path,
                     std::map<int, std::vector<int>> &parent_child_list,
                     int *parent_list, float **graph, int number_of_nodes) {
  float cost = 0;
  final_path.push_back(1); // Our root node is 1
  int current_node = 1;
  int temp_node;
  while (final_path.size() <
         number_of_nodes) // Loops till we include all vertices
  {
    if (parent_child_list[current_node].size() >
        0) { // Condition is true if we are not at a leaf node
      temp_node = parent_child_list[current_node][0];
      final_path.push_back(temp_node);
      cost += graph[final_path[final_path.size() - 1]]
                   [final_path[final_path.size() - 2]];
      parent_child_list[current_node].erase(
          parent_child_list[current_node].begin());
      current_node = temp_node;
    } else
      current_node =
          parent_list[current_node]; // Backtrack if we reach leaf node
  }
  return cost;
}

/**
 * Brief: Calculates the minimum spanning tree for a weighted symmetric graph
 * param[in]: graph - A 2-D array which contains the graph data
 * param[in]: number_of_node - The number of vertices in graph
 * param[out]: parent_child_list - Same as parent list but shows the list of
 *child vertex each parent has
 * return: int* - A 1-D array of the parent-child adjancency list (MST of graph)
 **/
int *get_minimum_spanning_tree(
    float **graph, int number_of_nodes,
    std::map<int, std::vector<int>> &parent_child_list) {
  // Tuple is <parent key, child key>, This pair is <edge_cost, edge> used for
  // priority queue
  typedef std::pair<float, std::tuple<int, int>> cost_parent;

  // This is the MST as adjancency list
  int *parent_list;
  parent_list = new int[number_of_nodes + 1];

  // This defines visited vertex stored in set
  std::set<int> mst_list;
  std::set<int>::iterator it = mst_list.begin();
  std::set<int>::iterator it1, it2;
  std::pair<std::set<int>::iterator, bool> ptr;

  std::priority_queue<cost_parent, std::vector<cost_parent>,
                      std::greater<cost_parent>>
      weight_priority;

  // We start the exploration from vertex id 1
  ptr = mst_list.emplace(1);

  parent_list[0] =
      -1; // There is no vertex id 0 so we store -1 at the start of list
  parent_list[1] = -1; // Root vertex is 1 so it has no parent
  int new_element = 1;
  while (mst_list.size() !=
         number_of_nodes) // Loop until all the vetrex are explored
  {
    for (int i = 1; i < number_of_nodes + 1;
         i++) // Updating weight_queue one node at a time
    {
      if (graph[new_element][i] > 0) {
        std::tuple<int, int> parent_child(new_element, i);
        weight_priority.push(
            std::make_pair(graph[new_element][i], parent_child));
      }
    }
    bool avoid_cycle = false;
    // Cycle occurs when we select the edge end as a vertex which was already
    // explored(int the MST list)
    while (!avoid_cycle) {
      cost_parent new_link =
          weight_priority.top(); // Vertex with lowest edge cost
      ptr = mst_list.emplace(std::get<1>(new_link.second));
      if (ptr.second) {
        weight_priority.pop();
        parent_list[std::get<1>(new_link.second)] =
            std::get<0>(new_link.second);
        parent_child_list[std::get<0>(new_link.second)].push_back(
            std::get<1>(new_link.second));
        new_element = std::get<1>(new_link.second);
        avoid_cycle = true;
      } else {
        weight_priority.pop();
        avoid_cycle = false;
      }
    }
  }
  return parent_list;
}

/**
 * Brief: Calculates the edge cost by Eucledian Distance
 * param[in]: p1 - Co-ordinate of vertex1
 * param[in]: p2 - Co-ordinate of vertex2
 * return: float - Edge cost in float
 **/
float give_edge_weights(std::tuple<int, int> p1, std::tuple<int, int> p2) {
  return sqrt(pow((std::get<0>(p1) - std::get<0>(p2)), 2) +
              pow((std::get<1>(p1) - std::get<1>(p2)), 2));
}

int main() {
  using namespace std::chrono;
  srand(time(NULL));
  std::map<int, std::tuple<int, int>> vertices;
  int number_of_nodes = 300;
  for (int i = 1; i < number_of_nodes + 1; i++) {
    std::tuple<int, int> p(
        static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 100.0)),
        static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 100.0)));
    vertices[i] = p;
  }

  // For Graph as an adjacency matrix store the cost as the eucledian distance
  // between edges
  float **graph;
  graph = new float *[number_of_nodes + 1];
  for (int k = 1; k < number_of_nodes + 1; k++)
    graph[k] = new float[number_of_nodes + 1];
  for (int i = 1; i < number_of_nodes + 1; i++) {
    for (int j = 1; j < number_of_nodes + 1; j++) {
      if (i != j)
        graph[i][j] = give_edge_weights(vertices[i], vertices[j]);
      else
        graph[i][j] = -1;
    }
  }

  // graph = get_random_vertices(number_of_nodes);
  auto start_time = high_resolution_clock::now();
  // Get Maximum spanning tree
  int *parent_list;
  // In MST every vertex exept root will have 1 parent and can have n child
  // vertices
  std::map<int, std::vector<int>> parent_child_list;
  parent_list =
      get_minimum_spanning_tree(graph, number_of_nodes, parent_child_list);

  // Getting the final path using DFS
  std::vector<int> final_path;

  float cost = get_final_path(final_path, parent_child_list, parent_list, graph,
                              number_of_nodes);

  auto time_after_finding_tour = high_resolution_clock::now();
  auto duration_to_find_tour =
      duration_cast<microseconds>(time_after_finding_tour - start_time);
  std::cout << "Time to find tour without huristic: "
            << duration_to_find_tour.count() * 0.000001 << " seconds"
            << std::endl;

  float mst_cost = 0;
  for (int e = 1; e < number_of_nodes + 1; e++)
    mst_cost += graph[e][parent_list[e]];
  std::cout << "Cost of MST: " << mst_cost << std::endl;
  std::cout << "Cost of tour without huristic: " << cost << std::endl;

  std::vector<int> new_path;
  new_path = apply_heuristic_to_tour(final_path, vertices);
  auto time_after_finding_tour_with_huristic = high_resolution_clock::now();
  auto duration_to_find_tour_with_huristic = duration_cast<microseconds>(
      time_after_finding_tour_with_huristic - start_time);
  std::cout << "Time to find tour with huristic: "
            << duration_to_find_tour_with_huristic.count() * 0.000001
            << " seconds" << std::endl;
  cost = 0;
  int previous = 1;
  for (int i = 0; i < number_of_nodes; i++) {
    cost += graph[previous][new_path[i]];
    previous = new_path[i];
  }
  std::cout << "Cost of tour with huristic: " << cost << std::endl;

  return 0;
}
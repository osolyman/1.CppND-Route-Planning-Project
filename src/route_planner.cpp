#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // 2nd Implementation: Finding the closest node to the starting and ending coordinates
	  start_node = &m_Model.FindClosestNode(start_x, start_y);
  	end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// 3rd Implementation: Calculating the h-value.
// the distance to the end_node is used for the calculation.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node); // calling the distance method from class Node in route_model.h
}

// 4th Implementation: expanding the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors(); //first we find the neighbors of the node by calling the the necessary method for that.
  
  //iterating over each node in the neighbors
  for(RouteModel::Node *neighborNode : current_node->neighbors){
    neighborNode->h_value = CalculateHValue(neighborNode); // setting h_value for each neighbor node
	  neighborNode->parent = current_node; //setting parent_node for each neighbor node
  	neighborNode->g_value = current_node->g_value + current_node->distance(*neighborNode); //setting g_value for each neighbor node
    //if that neigbor node still not visited
    if(!neighborNode->visited){
      open_list.emplace_back(neighborNode); //then add it to the open_list.
      neighborNode->visited = true; //now mark it as visited
    }
  }
}

// 5th Implementation: sorting the open list and returning the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  //using sort function from algorithm library with lambda function to costum the sorting criteria
  std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *n1, RouteModel::Node *n2) {
    return (n1->h_value + n1->g_value) > (n2->h_value + n2->g_value); }); 
  
  RouteModel::Node *listNode = open_list.back(); //getting the node with the lowest sum of g and h value. 
  open_list.pop_back(); //removing that node from the open_list
  return listNode; //returning the pointer to that node
}

// 6th Implementation: returning the final path found from my A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // iterating backwards until the starting node is found.
	while(current_node != nullptr) {
      path_found.push_back(*current_node); //pushing the current node to vector of nodes
      
      //checking if the current node has a parent (except for the starting node)
      if(current_node->parent != nullptr) {
        distance += current_node->distance(*(current_node->parent)); //then adding the distance of the current node to its parent to the total distance
      }
      current_node = current_node->parent; // now updating the current node to become its parent
    }
  
    reverse(path_found.begin(), path_found.end()); //reversing the path_found vector to obtain the path from the starting node to the end
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// 7th Implementation: Writing the A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr; //declaring a pointer to a Node and initializing it to nullptr
	  
    // TODO: Implement your solution here.
    start_node->visited = true; // marking the starting node as visited
  	open_list.push_back(start_node); // adding the starting node to the open_list

      // iterating through the open_list until it becomes empty
      while(!open_list.empty()) {
        current_node = NextNode(); // the current node is now pointing to the next node in the open_list
        // returning and indicating that the final path has been found in case the current node is equal to the end node
        if(current_node == end_node) {
          m_Model.path = ConstructFinalPath(current_node);
          return;
        }
        // if the end node hasn't been reached yet, we add the neighbors of the current node to the open list.  
  	    AddNeighbors(current_node);
     }
     std::cout << "no path found!\n";
}
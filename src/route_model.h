#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
      
        // A `Node` pointer `parent`, which is initialized to a `nullptr`.
      	Node * parent = nullptr;
      
      	// A `float` `h_value`, 
      	// which is initialized to the maximum possible
      	float h_value = std::numeric_limits<float>::max();
      
      	// A `float` `g_value`, which is initialized to 0.0
      	float g_value = 0.0;
      
      	// A `bool` `visited`, which is initialized to `false`
      	bool visited = false;
      
      	// A vector of `Node` pointers named `neighbors`
      	std::vector<Node*> neighbors;
      
      	// A `distance` declaration. 
      	// This method should take a `Node` object as the argument, 
      	// and it should return a `float`. 
      	// The `distance` method shouldn't change the object being passed.
      	float distance(Node point) const { return std::sqrt(std::pow((x - point.x),2) + std::pow((y - point.y),2));}
      
      	Node* FindNeighbor(std::vector<int> node_indices);
      	void FindNeighbors();
      
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
  	// This variable will eventually store the path that is found by the A* search.
    std::vector<Node> path; 
  
   	// A public "getter" method `SNodes`. 
  	// This method returns a reference to the vector of `Nodes` stored as `m_Nodes`.
  	auto& SNodes() {return m_Nodes;}
  
  	auto& GetNodeToRoadMap() {return node_to_road;}
  
  	Node& FindClosestNode(float x, float y);

  private:
    // Add private RouteModel variables and methods here.
  
  	// A private vector of `Node` objects named `m_Nodes`
      std::vector<Node> m_Nodes;
  
  	  std::unordered_map<int,std::vector<const Model::Road*> > node_to_road;
  	  void CreateNodeToRoadHashmap();
};

#endif
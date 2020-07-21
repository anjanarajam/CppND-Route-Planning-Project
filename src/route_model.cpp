#include "route_model.h"
#include <iostream>

// Define the constructor of the route model by giving io2D as the input
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        //Nodes from the model class are added to route model nodes along with an index and the type of model
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }

    CreateNodeToRoadHashmap();
}

// Mapping  an index of nodes of a way belonging to a particular node
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        // If road is not of type footway
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {                
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    // if the node is the end of the vector, assign an empty node 
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                // map the road to the node
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

// Find the neighbour of a node of a way belonging to a particular road
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        // Get nodes from the RouteModel; m_Nodes
        node = parent_model->SNodes()[node_index];
        //if the node is not the current node and it is not visited
        if (this->distance(node) != 0 && !node.visited) {            
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

//Find the neighbour node from ways belonging to each road and add to the vector of neighbours
void RouteModel::Node::FindNeighbors() {
    //for each of the roads of the particular node index
    for (auto & road : parent_model->node_to_road[this->index]) {
        // for a particular way belonging to the road, find neighbour from the vector of nodes 
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            //for every way, a neighbour node is selected.
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}

// Find the node closest in the data from the open street map, given x and y coordinates.
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}
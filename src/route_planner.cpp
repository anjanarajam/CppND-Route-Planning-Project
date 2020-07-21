#include "route_planner.h"
#include <algorithm>

/* Define the route planner constructor by taking model as the input */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model_(model) {
    /* Convert inputs to percentage */
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    /*Find the start node and the end node from the open street data closest to the given x
    and y co-ordinates*/
    start_node_ = &(m_Model_.FindClosestNode(start_x, start_y));
    end_node_ = &(m_Model_.FindClosestNode(end_x, end_y));
}

/* Implement the CalculateHValue method */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value{};

    /* h-value is the distance of the node to the end node */
    h_value = node->distance(*end_node_);

    return h_value;
}

/*  Function to expand the current node by adding all unvisited neighbors to the open list */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    /*Find neighbours of the current node */
    current_node->FindNeighbors();

    /* For every neighbour */
    for (const auto& neighbour_node : current_node->neighbors) {
        /*If neighbour is not visited */
        if (!neighbour_node->visited) {
            /*Assign current node as its parent */
            neighbour_node->parent = current_node;
            /*g-value is the sum of the current node's g-value and the distance of the neighbour node
            from current node */
            neighbour_node->g_value += current_node->g_value + current_node->distance(*neighbour_node);
            /*Calculate h-value of the neighbour node*/
            neighbour_node->h_value = CalculateHValue(neighbour_node);
            /*Add neighbour to the open list */
            open_list_.push_back(neighbour_node);
            /*Mark the neighbour as visited */
            neighbour_node->visited = true;
        }
    }
}

/* Comparator function to sort the open list based on the lowest f value */
bool comparator(RouteModel::Node* node_1, RouteModel::Node* node_2) {
    float f1_value = node_1->g_value + node_1->h_value;
    float f2_value = node_2->g_value + node_2->h_value;

    return (f1_value > f2_value);
}

/* Function to sort the open list and return the next node */
RouteModel::Node *RoutePlanner::NextNode() {
     /* Arrange the list in descending order */
     std::sort(open_list_.begin(), open_list_.end(), comparator);
     /* Reverse the list to extract the node with the lowest f value */
     RouteModel::Node* next_node = open_list_.back();
     /* Remove the node from the open list */
     open_list_.pop_back();

     return next_node;
}


/* Function to return the final path found from your A* search */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    /* Initialize distance */
    distance_ = 0.0f;
    /* Initialize path found vector */
    std::vector<RouteModel::Node> path_found{};
    /* Initialize parent node as null pointer */
    RouteModel::Node* parent_node = nullptr;     

    /* If the current node is not the start node */
    while (current_node->distance(*start_node_) != 0) {
        /* Set the parent node as the current node's parent */
        parent_node = current_node->parent;
        /* Add the parent node to the path */
        path_found.push_back(*parent_node);
        /* Increment distance by the distance between current node and its parent */
        distance_ += current_node->distance(*parent_node);
        /* Set the current node as the parent node */
        current_node = parent_node;
    }

    /*Once current node is same as the start node, add start node to the path */
    path_found.push_back(*start_node_);

    /* Multiply the distance by the scale of the map to get meters */
    distance_ *= m_Model_.MetricScale();

    /* Reverse the path so that the start node comes in the begining */
    std::reverse(path_found.begin(), path_found.end());

    return path_found;    
}

/* Function impleemnting A* Search algorithm */
void RoutePlanner::AStarSearch() {
    /* Initialize current node as null pointer */
    RouteModel::Node *current_node = nullptr;
    /* Initialize path found vector */
    std::vector<RouteModel::Node> path_found{};

    /* Add start node to the open list */
    open_list_.push_back(start_node_);
    /* Mark start node as visited */
    start_node_->visited = true;

    /* if open list is not empty */
    while (!open_list_.empty()) {
        /* Get the node with the lowest f value from the open list */
        current_node = NextNode();

        /* If current node is the end node, find the final path from begining to end */
        if (current_node->distance(*end_node_) == 0) {
            path_found = ConstructFinalPath(current_node);
            /* Exit while loop */
            break;            
        }

        /* If current node is not end node, add its neighbours to the open list */
        AddNeighbors(current_node); 
    }

    /* Assign path_found to the member variable */
    m_Model_.path = std::move(path_found);
}
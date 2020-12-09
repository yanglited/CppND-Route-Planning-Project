#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x   *= 0.01;
    end_y   *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node   = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*end_node);

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for(auto & neighbor : current_node->neighbors)
    {
        neighbor->parent  = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }

}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open list according to g+h values, ascending:
    std::sort(open_list.begin(), open_list.end(),
              [](RouteModel::Node const* lhs, RouteModel::Node const* rhs) {
                  return (lhs->g_value + lhs->h_value) > (rhs->g_value + rhs->h_value);
              });

    // Get the pointer to the next node from the end of the vector:
    RouteModel::Node *next_node = open_list.back();

    // Remove the pointer
    open_list.pop_back();

    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(true)
    {
        path_found.push_back(*current_node);
        if(current_node->x == start_node->x && current_node->y == start_node->y)
        {
            break;
        }
        else
        {
            distance += current_node->distance(*(current_node->parent));
            current_node = current_node->parent;
        }
    }

    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Set current node to the start node and mark it as visited:
    current_node = start_node;
    current_node->visited = true; // Note: I do think though this should be done in the constructors.

    int stepCount = 0;
    while(true)
    {
        stepCount++;
        AddNeighbors(current_node);
        current_node = NextNode();
        if(current_node->x == end_node->x && current_node->y == end_node->y)
        {
            break;
        }
    }
    m_Model.path = ConstructFinalPath(current_node);
}
#include "route_planner.h"
#include <algorithm>

/**
 * @brief Constructor for RoutePlanner class.
 * 
 * @param model Reference to the RouteModel object.
 * @param start_x X coordinate of the start point.
 * @param start_y Y coordinate of the start point.
 * @param end_x X coordinate of the end point.
 * @param end_y Y coordinate of the end point.
 */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    start_node->parent = nullptr;
    end_node->parent = nullptr;

    // Initialize the open_list and distance.
    open_list.clear();
    distance = 0.0f;
}

/**
 * @brief Calculates the heuristic value (H value) for a given node.
 * 
 * @param node Pointer to the node for which H value is to be calculated.
 * @return The calculated H value.
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

/**
 * @brief Adds neighboring nodes to the open list.
 * 
 * @param current_node Pointer to the current node.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors)
    {
        if (!neighbor->visited)
        {
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

            open_list.push_back(neighbor);
            neighbor->visited = true;
        }
    }
}

/**
 * @brief Finds the next node to explore from the open list.
 * 
 * @return Pointer to the next node to explore.
 */
RouteModel::Node *RoutePlanner::NextNode()
{
    auto lowest = std::min_element(open_list.begin(), open_list.end(),
                                   [](const RouteModel::Node *a, const RouteModel::Node *b)
                                   {
                                       return a->g_value + a->h_value < b->g_value + b->h_value;
                                   });
    RouteModel::Node *next_node = *lowest;
    open_list.erase(lowest);
    return next_node;
}

/**
 * @brief Constructs the final path from start to end node.
 * 
 * @param current_node Pointer to the current node.
 * @return A vector of nodes representing the final path.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    std::vector<RouteModel::Node> path_found;

    while (current_node != nullptr)
    {
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());
    distance = 0.0f;
    for (size_t i = 0; i < path_found.size() - 1; i++)
    {
        distance += path_found[i].distance(path_found[i + 1]);
    }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}

/**
 * @brief Performs A* search to find the shortest path.
 */
void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = start_node;
    current_node->g_value = 0;
    current_node->h_value = CalculateHValue(current_node);
    current_node->visited = true;
    open_list.push_back(current_node);

    while (!open_list.empty())
    {
        current_node = NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }

    // If the open_list becomes empty without reaching the end node, there's no path
    std::cout << "No valid path found!" << std::endl;
    m_Model.path.clear();
}
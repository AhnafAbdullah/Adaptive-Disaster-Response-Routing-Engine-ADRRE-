#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <map>
#include <unordered_map>
#include <string>
#include <functional>
#include <tuple>
#include <fstream>
#include "json.hpp"
using json = nlohmann::json;

using namespace std;

struct Edge {
    int to;
    double timeCost;
    double rel;
};

struct Nodeinfo {
    int id;
    int demand;
    int priority;
    //double x, y;
};

struct Vehicle {
    int id;
    int capacity;
    int remaining;
};

bool loadFromJSON(const string& filename,
    vector<Nodeinfo>& nodes,
    vector<vector<Edge>>& graph_adj,
    vector<Vehicle>& vehicles)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open JSON file.\n";
        return false;
    }

    json j;
    file >> j;

    // --- Load Nodes ---
    nodes.clear();
    for (auto& n : j["nodes"]) {
        Nodeinfo node;
        node.id = n["id"];
        node.demand = n["demand"];
        node.priority = n["priority"];
        nodes.push_back(node);
    }

    // Resize graph to match number of nodes
    int N = nodes.size();
    graph_adj.clear();
    graph_adj.resize(N);

    // --- Load Edges (undirected) ---
    for (auto& e : j["edges"]) {
        int u = e["u"];
        int v = e["v"];
        double cost = e["cost"];
        double rel = e["reliability"];

        graph_adj[u].push_back({ v, cost, rel });
        graph_adj[v].push_back({ u, cost, rel }); // undirected
    }

    // --- Load Vehicles ---
    vehicles.clear();
    for (auto& v : j["vehicles"]) {
        Vehicle veh;
        veh.id = v["id"];
        veh.capacity = v["capacity"];
        veh.remaining = veh.capacity;
        vehicles.push_back(veh);
    }

    return true;
}

const int N = 10; // nodes 0..4

// Graph adjacency (undirected)
vector<vector<Edge>> graph_adj(N);

// Add undirected edge
void add_edge(int u, int v, double timeCost, double rel) {
    graph_adj[u].push_back({ v, timeCost, rel });
    graph_adj[v].push_back({ u, timeCost, rel });
}

// Dijkstra returning pair<distance, path vector<int>>
pair<double, vector<int>> dijkstra(int src, int dest) {
    const double DOUBLE_MAX = 1e18;
    vector<double> dist(N, DOUBLE_MAX);
    vector<int> parent(N, -1);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    dist[src] = 0;
    pq.push({ 0, src });
    while (!pq.empty()) {
        auto thisEdge = pq.top(); pq.pop();
        double d = thisEdge.first;
        int u = thisEdge.second;
        if (d > dist[u]) continue;
        if (u == dest) break;
        for (auto& e : graph_adj[u]) {
            int v = e.to;
            double w = e.timeCost;
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({ dist[v], v });
            }
        }
    }
    vector<int> path;
    if (dist[dest] >= INT_MAX) return { INT_MAX, path };
    int cur = dest;
    while (cur != -1) {
        path.push_back(cur);
        cur = parent[cur];
    }
    reverse(path.begin(), path.end());
    return { dist[dest], path };
}

// Compute timeCost and reliability product for a full route (sequence of nodes visited in order).
// The route is a list of nodes visited sequentially; edges between consecutive nodes must exist (we assume they do
// because they are built using shortest paths).
pair<double, double> calculateRouteTimetimeCostReliability(vector<int>& route) {
    double total_timeCost = 0.0;
    double reliability_product = 1.0;
    for (int i = 1; i < route.size(); ++i) {
        int u = route[i - 1], v = route[i];
        // find edge u->v
        bool found = false;
        for (auto& e : graph_adj[u]) {
            if (e.to == v) {
                total_timeCost += e.timeCost;
                reliability_product *= e.rel;
                found = true;
                break;
            }
        }
        if (!found) {
            // This shouldn't happen if we build the route by shortest-path chaining.
            cerr << "Warning: edge missing between " << u << " and " << v << "\n";
        }
    }
    return { total_timeCost, reliability_product };
}


int getNumberOfNodesFromJSON(const std::string& filename) {
    fstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << filename << "\n";
        return -1;
    }

    json j;
    file >> j;

    if (!j.contains("nodes") || !j["nodes"].is_array()) {
        std::cerr << "Error: JSON does not contain a valid 'nodes' array.\n";
        return -1;
    }

    return j["nodes"].size();
}

// Assign nodes to vehicles using a greedy knapsack heuristic
vector<vector<int>> assign_nodes_knapsack(vector<Nodeinfo>& nodes, vector<Vehicle>& vehicles,int depot) {
    int n = nodes.size();
    vector<bool> assigned(n, false);          // track assigned nodes
    vector<vector<int>> vehicle_nodes(vehicles.size());

    for (int vi = 0; vi < vehicles.size(); vi++) {
        int capacity_left = vehicles[vi].capacity;

        // Compute value/weight ratio for unassigned nodes
        vector<pair<double, int>> ratios; // {priority/demand, node_id}
        for (int i = 0; i < n; ++i) {
            if (i == depot || assigned[i]) continue;
            double ratio = double(nodes[i].priority) / nodes[i].demand;
            ratios.push_back({ ratio, i });
        }

        // Sort descending by ratio
        sort(ratios.rbegin(), ratios.rend());

        // Assign nodes to this vehicle while respecting capacity
        for (auto& thisRatio : ratios) {
            double ratio = thisRatio.first;
            int idx = thisRatio.second;
            if (nodes[idx].demand <= capacity_left) {
                vehicle_nodes[vi].push_back(idx);
                capacity_left -= nodes[idx].demand;
                assigned[idx] = true;
            }
        }
    }

    return vehicle_nodes;
}

double computeObjective(
    const vector<vector<int>>& full_routes,
    const vector<double>& route_timeCosts,
    const vector<double>& route_reliability,
    const vector<int>& delivered_demand,
    const vector<Nodeinfo>& nodes,
    const vector<Vehicle>& vehicles,
    double alpha = 1.0,
    double beta = 1.0,
    double gamma = 1.0)
{
    double obj = 0.0;

    // α term: sum of priority * time at each node
    double sumPriorityTime = 0.0;
    for (int vi = 0; vi < full_routes.size(); vi++) {
        double current_time = 0.0;
        for (int j = 1; j < full_routes[vi].size(); j++) {
            int u = full_routes[vi][j - 1];
            int v = full_routes[vi][j];
            // find edge timeCost
            for (auto& e : graph_adj[u]) {
                if (e.to == v) {
                    current_time += e.timeCost;
                    break;
                }
            }
            // add priority*time of node v
            if (v != 0) // skip depot
                sumPriorityTime += nodes[v].priority * current_time;
        }
    }

    // β term: sum of edge unreliability
    double sumUnreliability = 0.0;
    for (int vi = 0; vi < full_routes.size(); ++vi) {
        for (int j = 1; j < full_routes[vi].size(); ++j) {
            int u = full_routes[vi][j - 1];
            int v = full_routes[vi][j];
            for (auto& e : graph_adj[u]) {
                if (e.to == v) {
                    sumUnreliability += (1.0 - e.rel);
                    break;
                }
            }
        }
    }

    // γ term: sum of idle capacity per vehicle
    double sumIdle = 0.0;
    for (int vi = 0; vi < vehicles.size(); ++vi) {
        int delivered = 0;
        for (int node : full_routes[vi]) {
            if (node != 0) delivered += nodes[node].demand;
        }
        sumIdle += (vehicles[vi].capacity - delivered);
    }

    obj = alpha * sumPriorityTime + beta * sumUnreliability + gamma * sumIdle;
    return obj;
}


//int main() {
//    string filename = "input.json";
//
//    // Get number of nodes from JSON
//    int num_nodes = getNumberOfNodesFromJSON(filename);
//    if (num_nodes == -1) return 1; // fail if JSON cannot be read
//
//    // Resize graph adjacency list
//    graph_adj.resize(num_nodes);
//
//    // Now call the existing function to populate nodes, edges, and vehicles
//    vector<Nodeinfo> nodes;
//    vector<Vehicle> vehicles;
//    if (!loadFromJSON(filename, nodes, graph_adj, vehicles)) {
//        cerr << "Failed to load graph data from JSON.\n";
//        return 1;
//    }
//
//    int depot = 0; // assuming depot has id = 0
//
//    // --- Greedy assignment by priority (highest first), respecting vehicle capacity ---
//    vector<int> demand_nodes;
//    for (int i = 0; i < N; ++i) {
//        if (nodes[i].id != depot) demand_nodes.push_back(nodes[i].id);
//    }
//
//    // Sort nodes by priority descending, tie-break by demand desc, then id
//    sort(demand_nodes.begin(), demand_nodes.end(), [&](int a, int b) {
//        if (nodes[a].priority != nodes[b].priority) return nodes[a].priority > nodes[b].priority;
//        if (nodes[a].demand != nodes[b].demand) return nodes[a].demand > nodes[b].demand;
//        return a < b;
//        });
//
//    // Assignment
//    vector<vector<int>> assigned(vehicles.size());
//    vector<bool> served(N, false);
//
//    for (int nd : demand_nodes) {
//        int demand = nodes[nd].demand;
//        bool placed = false;
//        for (int vi = 0; vi < vehicles.size(); ++vi) {
//            if (vehicles[vi].remaining >= demand) {
//                assigned[vi].push_back(nd);
//                vehicles[vi].remaining -= demand;
//                served[nd] = true;
//                placed = true;
//                break;
//            }
//        }
//        if (!placed) {
//            cerr << "Node " << nd << " (demand " << demand << ") could not be assigned to any vehicle.\n";
//        }
//    }
//
//    // --- Route construction and evaluation (unchanged) ---
//    vector<vector<int>> full_routes(vehicles.size());
//    vector<double> route_timeCosts(vehicles.size(), 0.0);
//    vector<double> route_reliability(vehicles.size(), 1.0);
//    vector<int> delivered_demand(vehicles.size(), 0);
//
//    for (int vi = 0; vi < vehicles.size(); vi++) {
//        vector<int> waypoints = assigned[vi];
//        vector<int> visit_order;
//        int current = depot;
//        vector<bool> used_waypoint(waypoints.size(), false);
//        while (true) {
//            int chosen_idx = -1;
//            double best_dist = 1e18;
//            pair<double, vector<int>> best_path;
//            for (int k = 0; k < waypoints.size(); ++k) {
//                if (used_waypoint[k]) continue;
//                int candidate = waypoints[k];
//                auto res = dijkstra(current, candidate);
//                double d = res.first;
//                if (d < best_dist) {
//                    best_dist = d;
//                    chosen_idx = k;
//                    best_path = res;
//                }
//            }
//            if (chosen_idx == -1) break;
//            vector<int> path = best_path.second;
//            if (full_routes[vi].empty()) {
//                for (int node : path) full_routes[vi].push_back(node);
//            }
//            else {
//                for (int p = 1; p < path.size(); ++p) full_routes[vi].push_back(path[p]);
//            }
//            used_waypoint[chosen_idx] = true;
//            current = waypoints[chosen_idx];
//            delivered_demand[vi] += nodes[current].demand;
//        }
//        auto resret = dijkstra(current, depot);
//        vector<int> pathret = resret.second;
//        for (int p = 1; p < pathret.size(); ++p) full_routes[vi].push_back(pathret[p]);
//        if (full_routes[vi].empty() || full_routes[vi].front() != depot) {
//            full_routes[vi].insert(full_routes[vi].begin(), depot);
//        }
//        pair<double, double> res = calculateRouteTimetimeCostReliability(full_routes[vi]);
//        route_timeCosts[vi] = res.first;
//        route_reliability[vi] = res.second;
//    }
//
//    // --- Output formatting (unchanged) ---
//    double total_combined_timeCost = 0.0;
//    double sum_route_reliability = 0.0;
//    int total_delivered_demand = 0;
//    int total_possible_priority = 0;
//    int total_delivered_priority = 0;
//    for (auto& node : nodes) {
//        if (node.id != depot) total_possible_priority += node.priority;
//    }
//
//    for (int vi = 0; vi < vehicles.size(); ++vi) {
//        cout << "Vehicle " << vehicles[vi].id << " Route : ";
//        for (int i = 0; i < full_routes[vi].size(); ++i) {
//            cout << full_routes[vi][i];
//            if (i + 1 < full_routes[vi].size()) cout << " -> ";
//        }
//        cout << "\nDelivered Demand : " << delivered_demand[vi] << " \n";
//        cout << "Total timeCost : " << (long long)llround(route_timeCosts[vi]) << "  \n\n";
//        total_combined_timeCost += route_timeCosts[vi];
//        sum_route_reliability += route_reliability[vi];
//        total_delivered_demand += delivered_demand[vi];
//        int sumpri = 0;
//        for (int nid : assigned[vi]) sumpri += nodes[nid].priority;
//        total_delivered_priority += sumpri;
//    }
//
//    double avg_reliability = sum_route_reliability / vehicles.size();
//    cout << "Total Combined timeCost : " << (long long)llround(total_combined_timeCost) << "\n";
//    cout << "Average Reliability : " << avg_reliability << "\n";
//    cout << "Priority Satisfaction Score : " << total_delivered_priority << "\n";
//    double pri_percent = (total_possible_priority > 0) ? 100.0 * total_delivered_priority / total_possible_priority : 0.0;
//    cout << "Priority Satisfaction (%) : " << pri_percent << "%\n";
//
//    double objScore = computeObjective(full_routes, route_timeCosts, route_reliability, delivered_demand, nodes, vehicles, 1.0, 1.0, 1.0);
//    cout << "Objective Function Score: " << objScore << "\n";
//
//    return 0;
//}

int main() {
    string filename = "input.json";

    // Get number of nodes from JSON
    int num_nodes = getNumberOfNodesFromJSON(filename);
    if (num_nodes == -1) return 1; // fail if JSON cannot be read

    // Resize graph adjacency list
    graph_adj.resize(num_nodes);

    // Now call the existing function to populate nodes, edges, and vehicles
    vector<Nodeinfo> nodes;
    vector<Vehicle> vehicles;
    if (!loadFromJSON(filename, nodes, graph_adj, vehicles)) {
        cerr << "Failed to load graph data from JSON.\n";
        return 1;
    }

    int depot = 0; // assuming depot has id = 0

    // --- Vehicle assignment using knapsack-style heuristic ---
    vector<vector<int>> assigned = assign_nodes_knapsack(nodes, vehicles, depot);

    // --- Route construction using existing nearest-next Dijkstra logic ---
    vector<vector<int>> full_routes(vehicles.size());
    vector<double> route_timeCosts(vehicles.size(), 0.0);
    vector<double> route_reliability(vehicles.size(), 1.0);
    vector<int> delivered_demand(vehicles.size(), 0);

    for (int vi = 0; vi < vehicles.size(); ++vi) {
        vector<int> waypoints = assigned[vi]; // nodes to serve
        vector<int> visit_order;
        int current = depot;
        vector<bool> used_waypoint(waypoints.size(), false);

        while (true) {
            int chosen_idx = -1;
            double best_dist = 1e18;
            pair<double, vector<int>> best_path;
            for (int k = 0; k < waypoints.size(); ++k) {
                if (used_waypoint[k]) continue;
                int candidate = waypoints[k];
                auto res = dijkstra(current, candidate); // use your fixed Dijkstra
                double d = res.first;
                if (d < best_dist) {
                    best_dist = d;
                    chosen_idx = k;
                    best_path = res;
                }
            }
            if (chosen_idx == -1) break;

            vector<int> path = best_path.second;
            if (full_routes[vi].empty()) {
                for (int node : path) full_routes[vi].push_back(node);
            }
            else {
                for (int p = 1; p < path.size(); ++p) full_routes[vi].push_back(path[p]);
            }

            used_waypoint[chosen_idx] = true;
            current = waypoints[chosen_idx];
            delivered_demand[vi] += nodes[current].demand;
        }

        // Return to depot
        if (!full_routes[vi].empty()) {
            auto resret = dijkstra(current, depot);
            vector<int> pathret = resret.second;
            for (int p = 1; p < pathret.size(); ++p) full_routes[vi].push_back(pathret[p]);
        }
        else {
            full_routes[vi].push_back(depot);
            full_routes[vi].push_back(depot);
        }

        // Ensure route starts at depot
        if (full_routes[vi].front() != depot)
            full_routes[vi].insert(full_routes[vi].begin(), depot);

        // Evaluate route
        auto res = calculateRouteTimetimeCostReliability(full_routes[vi]);
        route_timeCosts[vi] = res.first;
        route_reliability[vi] = res.second;
    }

    // --- Output (same as your current version) ---
    double total_time = 0.0, total_reliability = 0.0;
    int total_delivered_priority = 0, total_possible_priority = 0;
    for (int i = 1; i < nodes.size(); ++i) total_possible_priority += nodes[i].priority;

    for (int vi = 0; vi < vehicles.size(); ++vi) {
        cout << "Vehicle " << vehicles[vi].id << " Route: ";
        for (int i = 0; i < full_routes[vi].size(); ++i) {
            cout << full_routes[vi][i];
            if (i + 1 < full_routes[vi].size()) cout << " -> ";
        }
        cout << "\nDelivered Demand: " << delivered_demand[vi]
            << " | Total TimeCost: " << llround(route_timeCosts[vi]) << "\n\n";

            total_time += route_timeCosts[vi];
            total_reliability += route_reliability[vi];

            int sumpri = 0;
            for (int nid : assigned[vi]) sumpri += nodes[nid].priority;
            total_delivered_priority += sumpri;
    }

    cout << "Total Combined timeCost: " << llround(total_time) << "\n";
    cout << "Average Reliability: " << total_reliability / vehicles.size() << "\n";
    cout << "Priority Satisfaction Score: " << total_delivered_priority << "\n";
    cout << "Priority Satisfaction (%): "
        << (100.0 * total_delivered_priority / total_possible_priority) << "%\n";

    double objScore = computeObjective(full_routes, route_timeCosts, route_reliability, delivered_demand, nodes, vehicles, 1.0, 1.0, 1.0);
    cout << "Objective Function Score: " << objScore << "\n";

    return 0;
}

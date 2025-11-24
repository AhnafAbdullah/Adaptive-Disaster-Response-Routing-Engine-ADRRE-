// routing.cpp
// Hybrid routing demo (hardcoded instance from user).
// - Dijkstra for shortest paths (returns both distance and path).
// - Greedy assignment by priority respecting vehicle capacities.
// - Route construction by chaining shortest paths between waypoints.
// - Computes delivered demand, total timeCost, per-route reliability (product of edge reliabilities).
// - Prints routes and global metrics.
//

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
    double x, y;
};

struct Vehicle {
    int id;
    int capacity;
    int remaining;
};

const int N = 5; // nodes 0..4

// Graph adjacency (undirected)
vector<Edge> graph_adj[N];

// Add undirected edge
void add_edge(int u, int v, double timeCost, double rel) {
    graph_adj[u].push_back({ v, timeCost, rel });
    graph_adj[v].push_back({ u, timeCost, rel });
}

// Dijkstra returning pair<distance, path vector<int>>
pair<double, vector<int>> dijkstra_path(int src, int dest) {
    const double DOUBLE_MAX = 1e18;
    vector<double> dist(N, DOUBLE_MAX);
    vector<int> parent(N, -1);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    dist[src] = 0;
    pq.push({ 0, src });
    while (!pq.empty()) {
        auto thisEdge = pq.top(); pq.pop();
        int d = thisEdge.first;
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

int main() {
    // Edges: (bidirectional)
    add_edge(0, 1, 4, 0.9);
    add_edge(0, 2, 6, 0.8);
    add_edge(1, 2, 2, 0.7);
    add_edge(1, 3, 5, 0.95);
    add_edge(2, 3, 3, 0.85);
    add_edge(3, 4, 4, 0.9);

    // Node details: id, demand, priority
    vector<Nodeinfo> nodes(N);
    nodes[0] = { 0, 0, 0, 0.0, 0.0 }; // depot
    nodes[1] = { 1, 3, 5, 0.0, 0.0 };
    nodes[2] = { 2, 2, 3, 0.0, 0.0 };
    nodes[3] = { 3, 4, 4, 0.0, 0.0 };
    nodes[4] = { 4, 1, 2, 0.0, 0.0 };

    // Vehicles: id, capacity
    vector<Vehicle> vehicles;
    vehicles.push_back({ 1, 5, 5 });
    vehicles.push_back({ 2, 6, 6 });

    int depot = 0;

    // --- Greedy assignment by priority (highest first), respecting vehicle capacity ---
    // Build list of demand nodes (exclude depot)
    vector<int> demand_nodes;
    for (int i = 1; i < N; ++i) demand_nodes.push_back(i);
    // sort nodes by priority desc, tie-break by demand desc then id
    sort(demand_nodes.begin(), demand_nodes.end(), [&](int a, int b) {
        if (nodes[a].priority != nodes[b].priority) return nodes[a].priority > nodes[b].priority;
        if (nodes[a].demand != nodes[b].demand) return nodes[a].demand > nodes[b].demand;
        return a < b;
        });

    // Assignment: vector of assigned node lists per vehicle
    vector<vector<int>> assigned(vehicles.size());
    vector<bool> served(N, false);

    for (int nd : demand_nodes) {
        int demand = nodes[nd].demand;
        // Try to assign to first vehicle that fits (greedy). This will replicate the user's expected assignment:
        // vehicle 1 gets node1 (3) then node2 (2). vehicle2 gets remaining nodes.
        bool placed = false;
        for (int vi = 0; vi < vehicles.size(); ++vi) {
            if (vehicles[vi].remaining >= demand) {
                assigned[vi].push_back(nd);
                vehicles[vi].remaining -= demand;
                served[nd] = true;
                placed = true;
                break;
            }
        }
        if (!placed) {
            cerr << "Node " << nd << " (demand " << demand << ") could not be assigned to any vehicle.\n";
        }
    }

    // --- Route construction: For each vehicle, order assigned nodes using nearest-next by shortest-path distance,
    // chaining shortest paths between points to build the full visited-node sequence (including intermediate nodes).
    vector<vector<int>> full_routes(vehicles.size());
    vector<double> route_timeCosts(vehicles.size(), 0.0);
    vector<double> route_reliability(vehicles.size(), 1.0);
    vector<int> delivered_demand(vehicles.size(), 0);

    for (int vi = 0; vi < vehicles.size(); ++vi) {
        vector<int> waypoints = assigned[vi]; // nodes to serve (by id)
        vector<int> visit_order;
        int current = depot;
        // select next as the waypoint with minimum shortest-path distance from current
        vector<bool> used_waypoint(waypoints.size(), false);
        while (true) {
            int chosen_idx = -1;
            double best_dist = 1e18;
            pair<double, vector<int>> best_path;
            for (int k = 0; k < waypoints.size(); ++k) {
                if (used_waypoint[k]) continue;
                int candidate = waypoints[k];
                auto res = dijkstra_path(current, candidate);
                double d = res.first;
                if (d < best_dist) {
                    best_dist = d;
                    chosen_idx = (int)k;
                    best_path = res;
                }
            }
            if (chosen_idx == -1) break;
            // append path from current -> chosen (but avoid duplicating current)
            vector<int> path = best_path.second;
            if (full_routes[vi].empty()) {
                // start from depot
                for (int node : path) full_routes[vi].push_back(node);
            }
            else {
                // avoid duplicating the joining node
                for (int p = 1; p < path.size(); ++p) full_routes[vi].push_back(path[p]);
            }
            // mark waypoint used
            used_waypoint[chosen_idx] = true;
            // update current
            current = waypoints[chosen_idx];
            // accumulate delivered demand
            delivered_demand[vi] += nodes[current].demand;
        }
        // return to depot: shortest path from current to depot
        if (full_routes[vi].empty()) {
            // vehicle didn't have any assigned nodes; route is depot->depot
            full_routes[vi].push_back(depot);
            full_routes[vi].push_back(depot);
        }
        else {
            auto resret = dijkstra_path(current, depot);
            vector<int> pathret = resret.second;
            // avoid duplicating the connecting node
            for (int p = 1; p < pathret.size(); ++p) full_routes[vi].push_back(pathret[p]);
        }
        // Ensure route starts at depot; if not, prepend depot
        if (full_routes[vi].empty() || full_routes[vi].front() != depot) {
            full_routes[vi].insert(full_routes[vi].begin(), depot);
        }

        // Evaluate route timeCost & reliability
		pair<double, double> res = calculateRouteTimetimeCostReliability(full_routes[vi]);
        double timeCost = res.first;
        double relprod = res.second;
        route_timeCosts[vi] = timeCost;
        route_reliability[vi] = relprod;
    }

    // --- Output formatting ---
    double total_combined_timeCost = 0.0;
    double sum_route_reliability = 0.0;
    int total_delivered_demand = 0;
    int total_possible_priority = 0;
    int total_delivered_priority = 0;
    for (int i = 1; i < N; ++i) total_possible_priority += nodes[i].priority;


    for (int vi = 0; vi < vehicles.size(); ++vi) {
        cout << "Vehicle " << vehicles[vi].id << " Route : ";
        // print route as "0 -> 1 -> 2 -> 0"
        for (int i = 0; i < full_routes[vi].size(); ++i) {
            cout << full_routes[vi][i];
            if (i + 1 < full_routes[vi].size()) cout << " -> ";
        }
        cout << "\n";
        cout << "Delivered Demand : " << delivered_demand[vi] << " \n";
        cout << "Total timeCost : " << (long long)llround(route_timeCosts[vi]) << "  \n";
        cout << "\n";
        total_combined_timeCost += route_timeCosts[vi];
        sum_route_reliability += route_reliability[vi];
        total_delivered_demand += delivered_demand[vi];
        // delivered priorities: sum priorities of nodes actually assigned to this vehicle
        int sumpri = 0;
        for (int nid : assigned[vi]) sumpri += nodes[nid].priority;
        total_delivered_priority += sumpri;
    }

    double avg_reliability = sum_route_reliability / vehicles.size();

    cout << "Total Combined timeCost : " << (long long)llround(total_combined_timeCost) << "\n";
    cout << "Average Reliability (route product average) : " << avg_reliability << "\n";

    cout << "Priority Satisfaction Score (sum of delivered priorities): " << total_delivered_priority << "\n";
    double pri_percent = 0.0;
    if (total_possible_priority > 0) pri_percent = 100.0 * (double)total_delivered_priority / (double)total_possible_priority;
    cout << "Priority Satisfaction (%) : " << pri_percent << "%\n";

    return 0;
}

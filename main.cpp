#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;

const double INF = numeric_limits<double>::max();

// Structure to represent a node (location)
struct Node {
    int id;
    int demand;
    int priority;
    Node(int i = 0, int d = 0, int p = 0) : id(i), demand(d), priority(p) {}
};

// Structure to represent an edge (road)
struct Edge {
    int u, v;
    double cost;        // travel time
    double reliability; // reliability score (0-1)
    Edge(int from = 0, int to = 0, double c = 0.0, double r = 0.0)
        : u(from), v(to), cost(c), reliability(r) {
    }
};

// Structure for vehicle information
struct Vehicle {
    int id;
    int capacity;
    Vehicle(int i = 0, int c = 0) : id(i), capacity(c) {}
};

// Structure for Dijkstra's algorithm
struct DijkstraNode {
    int id;
    double distance;
    DijkstraNode(int i, double d) : id(i), distance(d) {}

    // Priority queue needs to order by smallest distance first
    bool operator>(const DijkstraNode& other) const {
        return distance > other.distance;
    }
};

class DisasterRoutingSystem {
private:
    vector<Node> nodes;
    vector<Edge> edges;
    vector<Vehicle> vehicles;
    vector<vector<pair<int, double>>> adj; // adjacency list: (neighbor, combined_weight)

    // Parameters for the objective function
    double alpha, beta, gamma;

public:
    DisasterRoutingSystem(double a = 1.0, double b = 1.0, double c = 1.0)
        : alpha(a), beta(b), gamma(c) {
    }

    // Add a node to the graph
    void addNode(int id, int demand, int priority) {
        nodes.push_back(Node(id, demand, priority));
    }

    // Add an edge to the graph
    void addEdge(int u, int v, double cost, double reliability) {
        edges.push_back(Edge(u, v, cost, reliability));
    }

    // Add a vehicle
    void addVehicle(int id, int capacity) {
        vehicles.push_back(Vehicle(id, capacity));
    }

    // Build adjacency list using the formula: ?*cost + ?*(1-reliability)
    void buildGraph() {
        int maxNodeId = 0;
        for (const auto& node : nodes) {
            maxNodeId = max(maxNodeId, node.id);
        }
        for (const auto& edge : edges) {
            maxNodeId = max(maxNodeId, max(edge.u, edge.v));
        }

        adj.resize(maxNodeId + 1);

        for (const auto& edge : edges) {
            double weight = alpha * edge.cost + beta * (1 - edge.reliability);

            // Undirected graph - add both directions
            adj[edge.u].push_back({ edge.v, weight });
            adj[edge.v].push_back({ edge.u, weight });
        }
    }

    // Dijkstra's algorithm to find shortest paths from source to all nodes
    vector<double> dijkstra(int source) {
        int n = adj.size();
        vector<double> dist(n, INF);
        vector<bool> visited(n, false);

        priority_queue<DijkstraNode, vector<DijkstraNode>, greater<DijkstraNode>> pq;

        dist[source] = 0.0;
        pq.push(DijkstraNode(source, 0.0));

        while (!pq.empty()) {
            DijkstraNode current = pq.top();
            pq.pop();

            int u = current.id;

            if (visited[u]) continue;
            visited[u] = true;

            for (const auto& neighbor : adj[u]) {
                int v = neighbor.first;
                double weight = neighbor.second;

                if (!visited[v] && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push(DijkstraNode(v, dist[v]));
                }
            }
        }

        return dist;
    }

    // Find shortest path from source to destination using Dijkstra
    vector<int> findShortestPath(int source, int destination) {
        int n = adj.size();
        vector<double> dist(n, INF);
        vector<int> parent(n, -1);
        vector<bool> visited(n, false);

        priority_queue<DijkstraNode, vector<DijkstraNode>, greater<DijkstraNode>> pq;

        dist[source] = 0.0;
        pq.push(DijkstraNode(source, 0.0));

        while (!pq.empty()) {
            DijkstraNode current = pq.top();
            pq.pop();

            int u = current.id;

            if (u == destination) break;
            if (visited[u]) continue;
            visited[u] = true;

            for (const auto& neighbor : adj[u]) {
                int v = neighbor.first;
                double weight = neighbor.second;

                if (!visited[v] && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push(DijkstraNode(v, dist[v]));
                }
            }
        }

        // Reconstruct path
        vector<int> path;
        if (dist[destination] == INF) {
            return path; // No path exists
        }

        for (int v = destination; v != -1; v = parent[v]) {
            path.push_back(v);
        }
        reverse(path.begin(), path.end());

        return path;
    }

    // Calculate the cost of a given path
    double calculatePathCost(const vector<int>& path) {
        if (path.size() < 2) return 0.0;

        double totalCost = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];

            // Find the edge between u and v
            for (const auto& edge : edges) {
                if ((edge.u == u && edge.v == v) || (edge.u == v && edge.v == u)) {
                    totalCost += edge.cost;
                    break;
                }
            }
        }
        return totalCost;
    }

    // Calculate reliability of a given path
    double calculatePathReliability(const vector<int>& path) {
        if (path.size() < 2) return 1.0;

        double totalReliability = 1.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];

            // Find the edge between u and v
            for (const auto& edge : edges) {
                if ((edge.u == u && edge.v == v) || (edge.u == v && edge.v == u)) {
                    totalReliability *= edge.reliability;
                    break;
                }
            }
        }
        return totalReliability;
    }

    // Simple greedy algorithm for vehicle routing
    void solve() {
        buildGraph();

        cout << "=== Disaster Routing Solution ===" << endl;
        cout << "Using Dijkstra's Algorithm with weights: ?=" << alpha << ", ?=" << beta << endl << endl;

        // Display all pairs shortest paths
        cout << "Shortest Paths from Depot (Node 0):" << endl;
        vector<double> distFromDepot = dijkstra(0);
        for (size_t i = 0; i < distFromDepot.size(); i++) {
            if (distFromDepot[i] < INF) {
                cout << "Node " << i << ": " << distFromDepot[i] << endl;
            }
        }
        cout << endl;

        // Simple vehicle routing - assign nodes to vehicles based on capacity and proximity
        vector<bool> assigned(nodes.size(), false);
        vector<vector<int>> vehicleRoutes(vehicles.size());

        // Mark depot as assigned
        assigned[0] = true;

        // Assign nodes to vehicles using greedy approach
        for (size_t i = 0; i < vehicles.size(); i++) {
            int currentCapacity = vehicles[i].capacity;
            int currentNode = 0; // Start from depot

            while (currentCapacity > 0) {
                // Find the closest unassigned node that fits in capacity
                int closestNode = -1;
                double minDistance = INF;

                for (size_t j = 0; j < nodes.size(); j++) {
                    if (!assigned[j] && nodes[j].id != 0 && nodes[j].demand <= currentCapacity) {
                        if (distFromDepot[nodes[j].id] < minDistance) {
                            minDistance = distFromDepot[nodes[j].id];
                            closestNode = j;
                        }
                    }
                }

                if (closestNode == -1) break; // No more nodes can be assigned

                // Assign this node to current vehicle
                assigned[closestNode] = true;
                currentCapacity -= nodes[closestNode].demand;
                vehicleRoutes[i].push_back(nodes[closestNode].id);
            }

            // Add return to depot
            if (!vehicleRoutes[i].empty()) {
                vehicleRoutes[i].insert(vehicleRoutes[i].begin(), 0); // Start from depot
                vehicleRoutes[i].push_back(0); // Return to depot
            }
        }

        // Display vehicle routes
        double totalCombinedCost = 0.0;
        double totalReliability = 0.0;
        int edgeCount = 0;

        for (size_t i = 0; i < vehicles.size(); i++) {
            if (vehicleRoutes[i].empty()) {
                cout << "Vehicle " << vehicles[i].id << ": No route assigned" << endl;
                continue;
            }

            cout << "Vehicle " << vehicles[i].id << " Route: ";
            for (size_t j = 0; j < vehicleRoutes[i].size(); j++) {
                cout << vehicleRoutes[i][j];
                if (j < vehicleRoutes[i].size() - 1) cout << "->";
            }
            cout << endl;

            // Calculate delivered demand
            int deliveredDemand = 0;
            for (size_t j = 1; j < vehicleRoutes[i].size() - 1; j++) {
                int nodeId = vehicleRoutes[i][j];
                for (const auto& node : nodes) {
                    if (node.id == nodeId) {
                        deliveredDemand += node.demand;
                        break;
                    }
                }
            }
            cout << "Delivered Demand: " << deliveredDemand << endl;

            // Calculate path cost and reliability
            double pathCost = calculatePathCost(vehicleRoutes[i]);
            double pathReliability = calculatePathReliability(vehicleRoutes[i]);

            cout << "Total Cost: " << pathCost << endl;
            cout << "Path Reliability: " << pathReliability << endl << endl;

            totalCombinedCost += pathCost;
            totalReliability += pathReliability;
            edgeCount += (vehicleRoutes[i].size() - 1);
        }

        cout << "Total Combined Cost: " << totalCombinedCost << endl;
        cout << "Average Reliability: " << (totalReliability / vehicles.size()) << endl;
    }
};

int main() {
    // Create the routing system with given weights
    DisasterRoutingSystem system(1.0, 1.0, 1.0); // ?=1, ?=1, ?=1

    // Hardcoded input data from the problem statement

    // Add nodes (locations)
    system.addNode(0, 0, 0);  // Depot
    system.addNode(1, 3, 5);
    system.addNode(2, 2, 3);
    system.addNode(3, 4, 4);
    system.addNode(4, 1, 2);

    // Add edges (roads)
    system.addEdge(0, 1, 4.0, 0.9);
    system.addEdge(0, 2, 6.0, 0.8);
    system.addEdge(1, 2, 2.0, 0.7);
    system.addEdge(1, 3, 5.0, 0.95);
    system.addEdge(2, 3, 3.0, 0.85);
    system.addEdge(3, 4, 4.0, 0.9);

    // Add vehicles
    system.addVehicle(1, 5);
    system.addVehicle(2, 6);

    // Solve the routing problem
    system.solve();

    // Demonstrate path finding between specific nodes
    cout << "=== Path Finding Examples ===" << endl;
    vector<int> path1 = system.findShortestPath(0, 4);
    cout << "Shortest path from 0 to 4: ";
    for (int node : path1) {
        cout << node << " ";
    }
    cout << endl;

    vector<int> path2 = system.findShortestPath(1, 3);
    cout << "Shortest path from 1 to 3: ";
    for (int node : path2) {
        cout << node << " ";
    }
    cout << endl;

    return 0;
}
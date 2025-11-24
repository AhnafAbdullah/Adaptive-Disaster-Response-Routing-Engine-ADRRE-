#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <map>
#include <string>

using namespace std;

const double INF = numeric_limits<double>::max();

// Structure to represent a graph edge
struct Edge {
    int u, v;
    double cost;
    double reliability;

    Edge(int u, int v, double cost, double reliability)
        : u(u), v(v), cost(cost), reliability(reliability) {
    }
};

// Structure to represent a node (location)
struct Node {
    int id;
    double demand;
    int priority;
    double service_time;

    Node(int id, double demand, int priority, double service_time = 0)
        : id(id), demand(demand), priority(priority), service_time(service_time) {
    }
};

// Structure to represent a vehicle
struct Vehicle {
    int id;
    double capacity;
    double max_travel_time;
    vector<int> route;
    double current_load;
    double total_cost;

    Vehicle(int id, double capacity, double max_travel_time = INF)
        : id(id), capacity(capacity), max_travel_time(max_travel_time),
        current_load(0), total_cost(0) {
    }
};

// Graph class to represent the disaster region
class DisasterGraph {
private:
    int V;
    vector<vector<pair<int, double>>> adj; // (neighbor, cost) / u->{v, weight}
    vector<vector<double>> reliability_matrix;
    vector<Node> nodes;

public:
    DisasterGraph(int vertices) : V(vertices) {
        adj.resize(V);
        reliability_matrix.resize(V, vector<double>(V, 0.0));
    }

    void addEdge(int u, int v, double cost, double reliability) {
        adj[u].push_back({ v, cost });
        adj[v].push_back({ u, cost });
        reliability_matrix[u][v] = reliability;
        reliability_matrix[v][u] = reliability;
    }

    void addNode(const Node& node) {
        nodes.push_back(node);
    }

    // Dijkstra's algorithm to find shortest paths from source
    vector<double> dijkstra(int source) {
        vector<double> dist(V, INF);
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

        dist[source] = 0;
        pq.push({ 0, source });

        while (!pq.empty()) {
            double current_dist = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            if (current_dist > dist[u]) continue;

            for (auto& edge : adj[u]) {
                int v = edge.first;
                double weight = edge.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({ dist[v], v });
                }
            }
        }
        return dist;
    }

    // Get reliability between two nodes
    double getReliability(int u, int v) {
        return reliability_matrix[u][v];
    }

    // Get all nodes
    vector<Node>& getNodes() {
        return nodes;
    }

    int getVertexCount() const {
        return V;
    }
};

// Disaster Response System class
class DisasterResponseSystem {
private:
    DisasterGraph graph;
    vector<Vehicle> vehicles;
    double alpha, beta, gamma; // weights for objective function

public:
    DisasterResponseSystem(const DisasterGraph& g, double alpha = 1.0, double beta = 0.5, double gamma = 0.3)
        : graph(g), alpha(alpha), beta(beta), gamma(gamma) {
    }

    void addVehicle(const Vehicle& vehicle) {
        vehicles.push_back(vehicle);
    }

    // Greedy vehicle assignment with priority-based node selection
    void assignRoutes() {
        // Reset vehicles
        for (auto& vehicle : vehicles) {
            vehicle.route.clear();
            vehicle.current_load = 0;
            vehicle.total_cost = 0;
        }

        // Get all nodes except depot (node 0)
        vector<Node> nodes = graph.getNodes();
        vector<Node> deliveryNodes;
        for (const auto& node : nodes) {
            if (node.id != 0 && node.demand > 0) {
                deliveryNodes.push_back(node);
            }
        }

        // Sort nodes by priority (descending)
        sort(deliveryNodes.begin(), deliveryNodes.end(),
            [](const Node& a, const Node& b) {
                return a.priority > b.priority;
            });

        // Precompute distances from depot
        vector<double> depotDistances = graph.dijkstra(0);

        // Assign nodes to vehicles using greedy approach
        for (const auto& node : deliveryNodes) {
            double minAdditionalCost = INF;
            int bestVehicle = -1;
            int bestInsertPosition = -1;

            for (int i = 0; i < vehicles.size(); i++) {
                if (vehicles[i].current_load + node.demand <= vehicles[i].capacity) {
                    // Try to insert this node into the vehicle's route
                    double additionalCost = calculateInsertionCost(vehicles[i], node.id, depotDistances);

                    if (additionalCost < minAdditionalCost) {
                        minAdditionalCost = additionalCost;
                        bestVehicle = i;
                    }
                }
            }

            if (bestVehicle != -1) {
                // Insert node into the best vehicle's route
                insertNodeInRoute(vehicles[bestVehicle], node.id, depotDistances);
                vehicles[bestVehicle].current_load += node.demand;
            }
        }

        // Complete routes by returning to depot
        for (auto& vehicle : vehicles) {
            if (!vehicle.route.empty() && vehicle.route.back() != 0) {
                vehicle.route.push_back(0);
            }
        }
    }

private:
    double calculateInsertionCost(const Vehicle& vehicle, int nodeId, const vector<double>& depotDistances) {
        if (vehicle.route.empty()) {
            // Vehicle at depot, cost is round trip to node
            return 2 * depotDistances[nodeId];
        }

        // For simplicity, calculate cost of inserting at end of route
        int lastNode = vehicle.route.back();
        vector<double> lastNodeDistances = graph.dijkstra(lastNode);

        return lastNodeDistances[nodeId] + depotDistances[nodeId] - depotDistances[lastNode];
    }

    void insertNodeInRoute(Vehicle& vehicle, int nodeId, const vector<double>& depotDistances) {
        if (vehicle.route.empty()) {
            vehicle.route.push_back(0); // Start from depot
        }

        // For simplicity, always insert at the end (before returning to depot)
        vehicle.route.push_back(nodeId);
    }

public:
    // Calculate objective function value
    double calculateObjectiveFunction() {
        double totalCost = 0.0;
        double totalReliabilityPenalty = 0.0;
        double totalIdleCost = 0.0;

        // Calculate total travel cost and reliability penalty
        for (const auto& vehicle : vehicles) {
            if (vehicle.route.size() < 2) continue;

            for (int i = 0; i < vehicle.route.size() - 1; i++) {
                int u = vehicle.route[i];
                int v = vehicle.route[i + 1];

                // Find the edge cost (simplified - would need edge lookup)
                vector<double> dist = graph.dijkstra(u);
                double cost = dist[v];
                totalCost += cost;

                // Reliability penalty (simplified)
                double reliability = graph.getReliability(u, v);
                totalReliabilityPenalty += (1 - reliability);
            }
        }

        // Calculate idle cost (simplified - vehicles with no routes are idle)
        for (const auto& vehicle : vehicles) {
            if (vehicle.route.size() <= 2) { // Only depot or depot->one node->depot
                totalIdleCost += 1.0;
            }
        }

        return alpha * totalCost + beta * totalReliabilityPenalty + gamma * totalIdleCost;
    }

    // Print results
    void printResults() {
        cout << "=== DISASTER RESPONSE ROUTING RESULTS ===" << endl;
        cout << endl;

        double totalCombinedCost = 0.0;
        double totalDeliveredDemand = 0.0;
        double totalReliability = 0.0;
        int edgeCount = 0;

        for (const auto& vehicle : vehicles) {
            cout << "Vehicle " << vehicle.id << " Route: ";
            for (int i = 0; i < vehicle.route.size(); i++) {
                cout << vehicle.route[i];
                if (i < vehicle.route.size() - 1) cout << "->";
            }
            cout << endl;

            cout << "Delivered Demand: " << vehicle.current_load << endl;

            // Calculate vehicle cost
            double vehicleCost = 0.0;
            double vehicleReliability = 0.0;
            int vehicleEdgeCount = 0;

            if (vehicle.route.size() >= 2) {
                for (int i = 0; i < vehicle.route.size() - 1; i++) {
                    int u = vehicle.route[i];
                    int v = vehicle.route[i + 1];

                    vector<double> dist = graph.dijkstra(u);
                    vehicleCost += dist[v];

                    double rel = graph.getReliability(u, v);
                    vehicleReliability += rel;
                    vehicleEdgeCount++;
                }
            }

            cout << "Total Cost: " << vehicleCost << endl;
            if (vehicleEdgeCount > 0) {
                cout << "Average Reliability: " << (vehicleReliability / vehicleEdgeCount) << endl;
            }
            cout << endl;

            totalCombinedCost += vehicleCost;
            totalDeliveredDemand += vehicle.current_load;
            totalReliability += vehicleReliability;
            edgeCount += vehicleEdgeCount;
        }

        cout << "SUMMARY:" << endl;
        cout << "Total Combined Cost: " << totalCombinedCost << endl;
        cout << "Total Delivered Demand: " << totalDeliveredDemand << endl;
        if (edgeCount > 0) {
            cout << "Overall Average Reliability: " << (totalReliability / edgeCount) << endl;
        }
        cout << "Objective Function Value: " << calculateObjectiveFunction() << endl;
    }
};

// Create sample disaster scenario for testing
DisasterGraph createSampleScenario() {
    // Create graph with 5 nodes (0 is depot)
    DisasterGraph graph(5);

    // Add nodes (id, demand, priority)
    graph.addNode(Node(0, 0, 0));  // Depot
    graph.addNode(Node(1, 3, 5));  // High priority
    graph.addNode(Node(2, 2, 3));  // Medium priority
    graph.addNode(Node(3, 4, 4));  // High priority
    graph.addNode(Node(4, 1, 2));  // Low priority

    // Add edges (u, v, cost, reliability)
    graph.addEdge(0, 1, 4, 0.9);
    graph.addEdge(0, 2, 6, 0.8);
    graph.addEdge(1, 2, 2, 0.7);
    graph.addEdge(1, 3, 5, 0.95);
    graph.addEdge(2, 3, 3, 0.85);
    graph.addEdge(3, 4, 4, 0.9);

    return graph;
}

int main() {
    cout << "Disaster Response Routing Algorithm Implementation" << endl;
    cout << "=================================================" << endl;
    cout << endl;

    // Create sample scenario
    DisasterGraph graph = createSampleScenario();

    // Create response system
    DisasterResponseSystem system(graph);

    // Add vehicles
    system.addVehicle(Vehicle(1, 5));  // Vehicle 1 with capacity 5
    system.addVehicle(Vehicle(2, 6));  // Vehicle 2 with capacity 6

    // Assign routes using greedy algorithm
    system.assignRoutes();

    // Print results
    system.printResults();

    // Test with different parameters
    cout << endl;
    cout << "=== TESTING WITH DIFFERENT PARAMETERS ===" << endl;
    cout << endl;

    // Test with higher priority weight
    DisasterResponseSystem system2(graph, 2.0, 0.5, 0.3); // Higher time weight
    system2.addVehicle(Vehicle(1, 5));
    system2.addVehicle(Vehicle(2, 6));
    system2.assignRoutes();
    system2.printResults();

    return 0;
}
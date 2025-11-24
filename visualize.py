import json
import networkx as nx
import matplotlib.pyplot as plt

# Load JSON data exported from C++
with open("viz_data.json") as f:
    data = json.load(f)

# Create a graph
G = nx.Graph()

# Add nodes
for n in data["nodes"]:
    G.add_node(n["id"])

# Add edges with travel time as weight
for e in data["edges"]:
    G.add_edge(e["u"], e["v"], weight=e["time"])

# Use a layout to position nodes nicely
pos = nx.spring_layout(G, seed=42)  # automatically positions nodes

# Draw base graph
plt.figure(figsize=(12, 8))
nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=600)
nx.draw_networkx_labels(G, pos, font_size=10)
nx.draw_networkx_edges(G, pos, width=1, alpha=0.5, edge_color='gray')

# Draw edge labels (travel time)
edge_labels = {(e["u"], e["v"]): f'{e["time"]:.1f}' for e in data["edges"]}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

# Define colors for vehicles
colors = ['red', 'green', 'blue', 'orange', 'purple']

# Draw routes for each vehicle
for idx, r in enumerate(data["routes"]):
    path = r["path"]
    edges_in_path = [(path[i], path[i+1]) for i in range(len(path)-1)]
    nx.draw_networkx_edges(G, pos,
                           edgelist=edges_in_path,
                           width=3,
                           edge_color=colors[idx % len(colors)],
                           style='solid',
                           alpha=0.8,
                           label=f'Vehicle {r["vehicle_id"]}')

# Show legend
plt.legend(loc='upper left')
plt.title("Disaster Response Vehicle Routes")
plt.axis('off')
plt.tight_layout()
plt.show()

# eco-travel-optimizer

Creating a complete Python program for an "eco-travel-optimizer" project involves several key elements. We'll build a simplified version of this, including route optimization using a concept like Dijkstra's algorithm with a focus on minimizing carbon emissions.

For this example, let's assume we have a basic dataset of travel routes along with their corresponding carbon emissions. We'll design a program to find the most eco-friendly route between two destinations.

Here is a simple version of the program:

```python
import heapq

class Graph:
    def __init__(self):
        self.edges = {}  # Represents the adjacency list of the graph
        self.weights = {}  # Represents the weights (carbon emissions) between edges

    def add_edge(self, from_node, to_node, weight):
        # Bidirectional graph (undirected)
        if from_node not in self.edges:
            self.edges[from_node] = []
        if to_node not in self.edges:
            self.edges[to_node] = []

        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        
        # Carbon emissions stored in weights dictionary
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight

def dijkstra(graph, start, end):
    # Handles case where start or end node is missing
    if start not in graph.edges or end not in graph.edges:
        raise ValueError("Start or end node not present in the graph")

    # Priority queue to store (cost, node)
    queue = [(0, start)]
    visited = set()
    min_costs = {start: 0}
    predecessors = {}

    while queue:
        current_cost, current_node = heapq.heappop(queue)

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == end:
            break

        for neighbor in graph.edges[current_node]:
            weight = graph.weights[(current_node, neighbor)]
            cost = current_cost + weight

            # If neighbor not visited or new cost is lower
            if neighbor not in min_costs or cost < min_costs[neighbor]:
                min_costs[neighbor] = cost
                heapq.heappush(queue, (cost, neighbor))
                predecessors[neighbor] = current_node

    if end not in min_costs:
        raise ValueError("No route from start to end")

    # Reconstruct path
    path = []
    step = end
    while step is not None:
        path.append(step)
        step = predecessors.get(step)
    path.reverse()

    return path, min_costs[end]

def main():
    # Create graph representing travel routes and carbon emissions (hypothetical data)
    graph = Graph()
    graph.add_edge("A", "B", 5)  # 5 units of carbon emissions
    graph.add_edge("B", "C", 2)
    graph.add_edge("A", "C", 10)
    graph.add_edge("C", "D", 1)
    graph.add_edge("B", "D", 4)
    
    start_location = "A"
    end_location = "D"

    try:
        eco_friendly_path, emissions = dijkstra(graph, start_location, end_location)
        print("Eco-friendly path:", eco_friendly_path)
        print("Total carbon emissions:", emissions)
    except ValueError as e:
        print("Error:", e)

if __name__ == "__main__":
    main()

```

### Key Components:
1. **Graph Representation**: We use an adjacency list to represent the graph and a dictionary to keep the weights (carbon emissions) between nodes.

2. **Dijkstra's Algorithm**: This algorithm helps find the shortest path between nodes in a graph. Here, it helps find the path with the lowest carbon emissions.

3. **Error Handling**: Basic error handling is implemented to catch issues—like missing nodes—or cases where no route exists.

4. **Example Data**: The graph is initialized with example nodes and edges with associated carbon emissions.

This is a simplified version of an eco-travel optimization program. A real-world application would involve more complex data processing, integration with geographical databases, API fetching for real-time data, advanced optimization techniques, and user-friendly interfacing.
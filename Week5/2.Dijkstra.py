__author__ = 'blkrt'
import networkx as nx
import matplotlib.pyplot as plt

def backtrace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def dijkstra(graph, source, target):
    queue = []
    visited = {}
    distance = {}
    shortest_distance = {}
    parent = {}

    for node in range(len(graph)):
        distance[node] = None
        visited[node] = False
        parent[node] = None
        shortest_distance[node] = float("inf")

    queue.append(source)
    distance[source] = 0
    while len(queue) != 0:
        current = queue.pop(0)
        visited[current] = True
        if current == target:
            path = backtrace(parent, source, target)
            print("Path:", path)
            return path  # Return the path for highlighting
        for neighbor in graph[current]:
            if visited[neighbor] == False:
                distance[neighbor] = distance[current] + 1
                if distance[neighbor] < shortest_distance[neighbor]:
                    shortest_distance[neighbor] = distance[neighbor]
                    parent[neighbor] = current
                    queue.append(neighbor)
    return []  # If path not found

def main():
    # Create a more complex graph with additional nodes and edges
    G = nx.Graph()
    edges = [
        (0, 1, 2), (1, 2, 3), (0, 3, 1), (3, 4, 4), (4, 5, 2), 
        (5, 6, 1), (1, 6, 3), (2, 5, 2), (2, 7, 1), (6, 7, 2),
        (3, 7, 4), (4, 7, 3)
    ]
    G.add_weighted_edges_from(edges)

    # Run Dijkstra from node 0 to node 7 and get the path
    path = dijkstra(G, 0, 5)

    # Highlight the path in the graph
    pos = nx.spring_layout(G)  # Positions for all nodes
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

    # Draw the shortest path in red and thicker lines
    path_edges = list(zip(path, path[1:]))  # Create edges from path nodes
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2.5)

    plt.show()

if __name__ == "__main__":
    main()

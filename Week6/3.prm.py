#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import yaml
import networkx as nx
from geometry_msgs.msg import Point

# Fungsi untuk menghasilkan node acak
def generate_random_nodes(num_nodes, xlim, ylim):
    return np.random.rand(num_nodes, 2) * [xlim[1] - xlim[0], ylim[1] - ylim[0]] + [xlim[0], ylim[0]]

# Fungsi untuk membuat graf dan menambahkan edges
def build_roadmap(nodes, radius):
    G = nx.Graph()
    for i, node in enumerate(nodes):
        G.add_node(i, pos=node)
        for j, other_node in enumerate(nodes[:i]):
            if np.linalg.norm(node - other_node) <= radius:
                G.add_edge(i, j)
    return G

# Fungsi untuk mencari jalur terpendek
def shortest_path(G, start, goal):
    try:
        path = nx.shortest_path(G, source=start, target=goal)
        return path
    except nx.NetworkXNoPath:
        return []

def visualize(G, nodes, path, start, goal):
    plt.figure(figsize=(8, 8))
    pos = nx.get_node_attributes(G, 'pos')
    
    # Visualisasikan seluruh PRM (node dan edge) dengan warna merah
    nx.draw(G, pos, node_size=50, with_labels=False, node_color="skyblue", edge_color="red", width=1)

    # Visualisasikan jalur terpendek (jika ada) dengan warna kuning
    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color="yellow", node_size=100)
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="yellow", width=2)

    # Visualisasikan titik start dan goal
    plt.scatter(nodes[start, 0], nodes[start, 1], c='green', s=200, label='Start', marker='o')
    plt.scatter(nodes[goal, 0], nodes[goal, 1], c='red', s=200, label='Goal', marker='x')

    plt.legend(loc='best')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Probabilistic Roadmap (PRM) with Start and Goal')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    rospy.init_node('prm_node')

    # Load parameters
    config_file = rospy.get_param("~config_file")
    with open(config_file, 'r') as file:
        params = yaml.safe_load(file)
    
    num_nodes = params['num_nodes']
    radius = params['connection_radius']
    xlim = params['xlim']
    ylim = params['ylim']
    start_node = params['start_node']
    goal_node = params['goal_node']

    # Tentukan posisi start dan goal
    nodes = generate_random_nodes(num_nodes, xlim, ylim)
    nodes[start_node] = [xlim[0], ylim[1]]  # Titik start di pojok kiri atas
    nodes[goal_node] = [xlim[1], ylim[0]]  # Titik goal di pojok kanan bawah

    G = build_roadmap(nodes, radius)
    path = shortest_path(G, start_node, goal_node)

    visualize(G, nodes, path, start_node, goal_node)


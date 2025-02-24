import heapq

def dijkstra(graph, start):
    # Initialize distances to all nodes as infinity, except for the start node
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    # Priority queue to hold nodes to visit (distance, node)
    priority_queue = [(0, start)]
    
    # Dictionary to store the previous node for each node in the optimal path
    previous_nodes = {node: None for node in graph}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If this distance is not up-to-date, skip it
        if current_distance > distances[current_node]:
            continue
        
        # Examine neighbors of the current node
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            # If a shorter path to neighbor is found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return distances, previous_nodes

def reconstruct_path(previous_nodes, start, end):
    # Reconstructs the shortest path from start to end using the previous_nodes dictionary
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()  # Reverse the path to start from the start node
    return path

# Example graph represented as an adjacency list
graph_name = {
    'A': (0,0),
    'B': (0, -29),
    'C': (103, 0),
    'D': (-34, 0),
    'E': (-104, 0),
    'F': (-104, 88),
    'G': (-34, 32),
    'H': (-1, 88),
    'I': (32, 88),
    'J': (103, 88),
    'K': (103, 162),
    'L':(40, 162),
    'M':(40, 139),
    'N':(-1, 162),
    'O':(-104, 162),
    'P':(-27, 123),
    'Q':(-1, 123),
    'R':(32, 66),
    'S':(-104, 31),
    'T':(103, -31), 
}
graph = {
    'A': [('C', 103), ('D', 34), ('B', 29)],
    'B': [('A', 29)],
    'C': [('T', 31), ('J', 88), ('A', 103)],
    'D': [('G', 32), ('E', 70), ('A', 34)],
    'E': [('S', 31), ('F', 88), ('D', 70)],
    'F': [('O', 74), ('H', 103), ('E', 88)],
    'G': [('D', 32)],
    'H': [('I', 33), ('R', 40), ('F', 103)],
    'I': [('H', 33), ('R', 22), ('J', 71)],
    'J': [('K', 74), ('C', 88), ('I', 71)],
    'K': [('J', 74), ('L', 63)],
    'L': [('K', 63), ('N', 41), ('M', 23)],
    'M': [('L', 23)],
    'N': [('L', 41), ('Q', 39), ('O', 103)],
    'O': [('F', 74), ('N', 103)],
    'P': [('Q', 26)],
    'Q': [('P', 26), ('N', 39), ('H', 35)],
    'R': [('I', 22)],
    'S': [('E', 31)],
    'T': [('C', 31)]
}

start_node = 'A'
end_node = 'P'

# Run Dijkstra's algorithm
distances, previous_nodes = dijkstra(graph, start_node)

# Retrieve the shortest path from start to end
path = reconstruct_path(previous_nodes, start_node, end_node)

print("Shortest distances:", distances)
print("Shortest path from", start_node, "to", end_node, ":", path)


from constants import GRAPH

import heapq

class PathFinder:
    def __init__(self, graph):
        self.graph = graph
    
    def find_shortest_path(self, node_A, node_B):
        """
        Given nodes A, B, find the shortest path between them, using the lazy version of Dijkstra.
        """
        N = len(self.graph)
        pq = [(0, node_A)]
        best = {node : float('inf') for node in self.graph.keys()}
        prev = {node : None for node in self.graph.keys()}
        visited = set()
        best[node_A] = 0
        
        while pq:
            curr_dist, curr_node = heapq.heappop(pq)
            if best[curr_node] < curr_dist:
                continue
            if curr_node in visited:
                continue
            visited.add(curr_node)
            for child_node in self.graph[curr_node]:
                if child_node not in visited:
                    curr_x, curr_y = curr_node
                    next_x, next_y = child_node
                    edge_dist = abs(next_x - curr_x) + abs(next_y - curr_y)
                    new_dist = curr_dist + edge_dist
                    if new_dist < best[child_node]:
                        best[child_node] = new_dist
                        heapq.heappush(pq, (new_dist, child_node))
                        prev[child_node] = curr_node
        return self.reconstruct_path(prev, node_A, node_B)    

    def reconstruct_path(self, prev, start, end):
        """
        From prev (produced by Dijkstra) reconstruct the path.
        We do this by going in reverse order then reversing the array.
        """
        path = []
        curr_node = end
        while curr_node != start:
            path.append(curr_node)
            curr_node = prev[curr_node]
        path.append(start)
        path.reverse()
        return path      
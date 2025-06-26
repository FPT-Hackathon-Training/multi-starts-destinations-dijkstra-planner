import requests
import heapq

map_url = "https://hackathon.omelet.tech/api/maps/df668aa9-bed0-4207-814a-4ec1ff478405/"


class MultiAGVPlanner:
    def __init__(self, map_url: str):
        self.map_url = map_url
        self.graph_data = None
        self.adjececy_list = {}
        self.start_nodes = []  # 2 node starts
        self.destination_nodes = []  # 2 node destinations

        self.distance_cahe = {}  # Cache distance compute for pre-planning

    def fetch_map_data(self):
        response = requests.get(self.map_url)
        self.graph_data = response.json()

    def build_adjacency_list(self):
        nodes = self.graph_data["nodes"]
        for node in nodes:
            node_id = node["id"]
            self.adjececy_list[node_id] = []

            if node["type"] == "START":
                self.start_nodes.append(node_id)
            elif node["type"] == "DESTINATION":
                self.destination_nodes.append(node_id)
        edges = self.graph_data["edges"]
        for edge in edges:
            source = edge["source"]
            target = edge["target"]
            weight = 1  # Khoảng cách của các nodes là cố định.
            # Update weight của các cạnh để control xe có ưu tiên đi qua cạnh đó hay không.

            # add dirirectional edges
            self.adjececy_list[source].append((target, weight))
            self.adjececy_list[target].append((source, weight))

    def dijkstra(self, start, end):
        cache_key = (start, end)
        if cache_key in self.distance_cahe:
            return self.distance_cahe[cache_key]

        if start not in self.adjececy_list or end not in self.adjececy_list:
            return [], float("inf")

        distances = {node: float("inf") for node in self.adjececy_list}
        distances[start] = 0

        previous = {node: None for node in self.adjececy_list}

        # Priority queue
        pq = [(0, start)]
        visited = set()

        while pq:  # loop until `pq` is empty
            current_distance, current_node = heapq.heappop(pq)
            # IN first iteration, current_distance is 0, current_node is start
            if current_node in visited:
                continue

            visited.add(current_node)

            # Found the destination
            if current_node == end:
                break

            # Check neighbors
            for neighbor, weight in self.adjececy_list[current_node]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        # Reconstruct the shortest path
        path = []  # store from the destination to the start
        current = end

        while current is not None:
            path.append(current)
            current = previous[current]

        path.reverse()
        self.distance_cahe[cache_key] = (path, distances[end])
        return path, distances[end]

    def compute_distance_matrix(self, starts, destinations):
        matrix = {}
        for start in starts:
            for destination in destinations:
                path, distance = self.dijkstra(start, destination)
                print(f"Distance from {start} to {destination}: {distance}")
                matrix[(start, destination)] = distance
        return matrix

    def find_global_optimal_plan(self):
        matrix = self.compute_distance_matrix(self.start_nodes, self.destination_nodes)

        for start in self.start_nodes:
            total_distance = 0
            for destination in self.destination_nodes:
                if (start, destination) in matrix:
                    total_distance += matrix[(start, destination)]
            print(f"Total distance: {total_distance} for start {start}")


if __name__ == "__main__":
    planner = MultiAGVPlanner(map_url)
    planner.fetch_map_data()
    planner.build_adjacency_list()
    planner.find_global_optimal_plan()

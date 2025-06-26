#!/usr/bin/env python3
"""
Multi-AGV Path Planning with Assignment Optimization
Optimally assigns multiple AGVs to destinations to minimize total distance
"""

import requests
import json
import heapq
from typing import Dict, List, Tuple, Optional
from datetime import datetime
import argparse
import sys
from itertools import permutations


class MultiAGVPlanner:
    def __init__(self, map_url: str):
        self.map_url = map_url
        self.graph_data = None
        self.adjacency_list = {}
        self.start_nodes = []
        self.destination_nodes = []
        self.distance_cache = {}  # Cache for dijkstra results

    def fetch_map_data(self) -> bool:
        """Fetch map data from the API"""
        try:
            response = requests.get(self.map_url)
            response.raise_for_status()
            self.graph_data = response.json()
            return True
        except requests.exceptions.RequestException as e:
            print(f"Error fetching map data: {e}")
            return False

    def build_adjacency_list(self):
        """Convert graph data to adjacency list for pathfinding"""
        if not self.graph_data or "edges" not in self.graph_data:
            raise ValueError("No graph data available")

        # Initialize adjacency list and identify start/destination nodes
        nodes = self.graph_data.get("nodes", [])
        for node in nodes:
            node_id = node["id"]
            self.adjacency_list[node_id] = []

            # Identify node types
            if node.get("type") == "START":
                self.start_nodes.append(node_id)
            elif node.get("type") == "DESTINATION":
                self.destination_nodes.append(node_id)

        # Build adjacency list from edges
        edges = self.graph_data["edges"]
        for edge in edges:
            source = edge["source"]
            target = edge["target"]
            weight = edge.get("weight", 1.0)

            # Add bidirectional edges
            self.adjacency_list[source].append((target, weight))
            self.adjacency_list[target].append((source, weight))

    def dijkstra(self, start: str, end: str) -> Tuple[List[str], float]:
        """
        Find shortest path using Dijkstra's algorithm with caching
        """
        # Check cache
        cache_key = (start, end)
        if cache_key in self.distance_cache:
            return self.distance_cache[cache_key]

        if start not in self.adjacency_list or end not in self.adjacency_list:
            return [], float("inf")

        # Distance from start to each node
        distances = {node: float("inf") for node in self.adjacency_list}
        distances[start] = 0

        # Previous node in optimal path
        previous = {node: None for node in self.adjacency_list}

        # Priority queue: (distance, node)
        pq = [(0, start)]
        visited = set()

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            if current_node in visited:
                continue

            visited.add(current_node)

            # Found destination
            if current_node == end:
                break

            # Check neighbors
            for neighbor, weight in self.adjacency_list[current_node]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous[current]

        if path[-1] != start:  # No path found
            result = ([], float("inf"))
        else:
            path.reverse()
            result = (path, distances[end])

        # Cache result
        self.distance_cache[cache_key] = result
        return result

    def compute_distance_matrix(
        self, starts: List[str], destinations: List[str]
    ) -> Dict[Tuple[str, str], Tuple[List[str], float]]:
        """Compute all pairwise distances between starts and destinations"""
        matrix = {}
        for start in starts:
            for dest in destinations:
                path, distance = self.dijkstra(start, dest)
                matrix[(start, dest)] = (path, distance)
        return matrix

    def find_optimal_assignment(self, starts: List[str], destinations: List[str]) -> Tuple[Dict[str, str], float]:
        """
        Find optimal assignment of AGVs to destinations using brute force for small problems
        Returns: (assignment dict, total distance)
        """
        if len(starts) != len(destinations):
            raise ValueError(f"Number of starts ({len(starts)}) must equal destinations ({len(destinations)})")

        # Compute distance matrix
        distance_matrix = self.compute_distance_matrix(starts, destinations)

        # For 2 AGVs, there are only 2 possible assignments
        if len(starts) == 2:
            # Assignment 1: start[0]->dest[0], start[1]->dest[1]
            dist1 = distance_matrix[(starts[0], destinations[0])][1] + distance_matrix[(starts[1], destinations[1])][1]

            # Assignment 2: start[0]->dest[1], start[1]->dest[0]
            dist2 = distance_matrix[(starts[0], destinations[1])][1] + distance_matrix[(starts[1], destinations[0])][1]

            if dist1 <= dist2:
                return {starts[0]: destinations[0], starts[1]: destinations[1]}, dist1
            else:
                return {starts[0]: destinations[1], starts[1]: destinations[0]}, dist2

        # For larger problems, try all permutations
        best_assignment = None
        best_distance = float("inf")

        for perm in permutations(destinations):
            total_distance = 0
            assignment = {}

            for i, start in enumerate(starts):
                dest = perm[i]
                assignment[start] = dest
                total_distance += distance_matrix[(start, dest)][1]

            if total_distance < best_distance:
                best_distance = total_distance
                best_assignment = assignment

        return best_assignment, best_distance

    def generate_multi_agv_plan(self, agv_assignments: Optional[Dict[str, str]] = None) -> Dict:
        """
        Generate movement plan for multiple AGVs
        If agv_assignments is None, will optimize assignment automatically
        """
        if agv_assignments is None:
            # Auto-detect and optimize
            if len(self.start_nodes) != 2 or len(self.destination_nodes) != 2:
                raise ValueError(
                    f"Expected 2 start nodes and 2 destinations, got {len(self.start_nodes)} and {len(self.destination_nodes)}"
                )

            # Find optimal assignment
            assignment, total_distance = self.find_optimal_assignment(self.start_nodes, self.destination_nodes)
            print(f"Optimal assignment found with total distance: {total_distance:.2f}")
        else:
            assignment = agv_assignments
            total_distance = 0

        # Build distance matrix for assigned pairs
        distance_matrix = self.compute_distance_matrix(list(assignment.keys()), list(assignment.values()))

        # Generate plan
        plan = {
            "map_id": self.graph_data.get("id", "unknown") if self.graph_data else "unknown",
            "agv_count": len(assignment),
            "total_distance": 0,
            "agv_plans": [],
            "timestamp": datetime.now().isoformat(),
        }

        # Create individual AGV plans
        agv_id = 1
        for start, dest in assignment.items():
            path, distance = distance_matrix[(start, dest)]

            agv_plan = {
                "agv_id": f"AGV{agv_id}",
                "start_node": start,
                "destination": dest,
                "movements": [{"from": start, "to": dest, "path": path, "distance": distance}],
                "total_distance": distance,
            }

            plan["agv_plans"].append(agv_plan)
            plan["total_distance"] += distance
            agv_id += 1

        return plan


def main():
    parser = argparse.ArgumentParser(description="Multi-AGV Path Planning Tool")
    parser.add_argument(
        "--url",
        default="https://hackathon.omelet.tech/api/maps/df668aa9-bed0-4207-814a-4ec1ff478405/",
        help="Map API URL",
    )
    parser.add_argument("--output", default="multi_agv_plan.json", help="Output file name")
    parser.add_argument("--format", choices=["json", "txt"], default="json", help="Output format")
    parser.add_argument("--submit", action="store_true", help="Submit plan to API after generation")
    parser.add_argument("--nickname", help="Nickname for API submission (required with --submit)")

    args = parser.parse_args()

    # Create planner
    planner = MultiAGVPlanner(args.url)

    # Fetch map data
    print(f"Fetching map data from {args.url}...")
    if not planner.fetch_map_data():
        print("Failed to fetch map data")
        sys.exit(1)

    # Build graph
    print("Building graph...")
    try:
        planner.build_adjacency_list()
    except ValueError as e:
        print(f"Error building graph: {e}")
        sys.exit(1)

    print(f"Found {len(planner.start_nodes)} start nodes: {planner.start_nodes}")
    print(f"Found {len(planner.destination_nodes)} destination nodes: {planner.destination_nodes}")

    # Check if manual assignment provided
    manual_assignment = None
    print("Optimizing AGV assignment automatically...")

    # Generate multi-AGV plan
    try:
        plan = planner.generate_multi_agv_plan(manual_assignment)
    except ValueError as e:
        print(f"Error generating plan: {e}")
        sys.exit(1)

    # Save results
    if args.format == "json":
        with open(args.output, "w") as f:
            json.dump(plan, f, indent=2)
    else:  # txt format
        with open(args.output, "w") as f:
            f.write("Multi-AGV Movement Plan\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Map ID: {plan['map_id']}\n")
            f.write(f"AGV Count: {plan['agv_count']}\n")
            f.write(f"Total Combined Distance: {plan['total_distance']:.2f}\n")
            f.write(f"Generated: {plan['timestamp']}\n\n")

            for agv in plan["agv_plans"]:
                f.write(f"\n{agv['agv_id']}:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Start: {agv['start_node']} → Destination: {agv['destination']}\n")
                f.write(f"Distance: {agv['total_distance']:.2f}\n")
                for movement in agv["movements"]:
                    f.write(f"Path: {' → '.join(movement['path'])}\n")

    print(f"\nMulti-AGV plan saved to {args.output}")
    print(f"Total combined distance: {plan['total_distance']:.2f}")

    # Display summary
    print("\nAssignment Summary:")
    for agv in plan["agv_plans"]:
        print(f"  {agv['agv_id']}: {agv['start_node']} → {agv['destination']} (distance: {agv['total_distance']:.2f})")

    # Submit to API if requested
    if args.submit:
        if not args.nickname:
            print("\nError: --nickname is required when using --submit")
            sys.exit(1)

        print(f"\nSubmitting plan to API with nickname: {args.nickname}")
        submit_multi_agv_plan(plan, args.nickname, args.url)


def submit_multi_agv_plan(plan_data, nickname, base_url):
    """Submit multi-AGV plan to the API"""
    import requests

    # Extract base URL
    base_url = base_url.split("/api/")[0]
    submit_url = f"{base_url}/api/submit-plan/"

    # Convert multi-AGV plan to submission format
    # Combine all movements from all AGVs
    all_movements = []

    for agv in plan_data["agv_plans"]:
        # Add AGV identifier to movements
        for movement in agv["movements"]:
            movement_copy = movement.copy()
            movement_copy["agv_id"] = agv["agv_id"]
            all_movements.append(movement_copy)

    # Use first AGV's start node as the plan start
    start_node = plan_data["agv_plans"][0]["start_node"]

    submission_data = {
        "nickname": nickname,
        "map_id": plan_data["map_id"],
        "plan": {
            "start_node": start_node,
            "movements": all_movements,
            "is_multi_agv": True,
            "agv_count": plan_data["agv_count"],
        },
    }

    try:
        response = requests.post(submit_url, json=submission_data)

        if response.status_code == 201:
            result = response.json()
            print(f"Plan submitted successfully!")
            print(f"Submission ID: {result.get('submission_id')}")
            print(f"Total distance: {result['total_distance']:.2f} (lower is better)")
            print(f"Total movements: {result['total_movements']}")
            print(f"Valid movements: {result['valid_movements']}/{result['total_movements']}")
        else:
            print(f"Submission failed: {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"Error submitting plan: {e}")


if __name__ == "__main__":
    main()

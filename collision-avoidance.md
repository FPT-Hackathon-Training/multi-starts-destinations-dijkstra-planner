# Multi-AGV Path Planning Algorithm Documentation

## Overview

The Multi-AGV Path Planning system optimizes routes for multiple Automated Guided Vehicles (AGVs) to minimize total completion time while preventing collisions. The system considers individual AGV speeds and implements collision avoidance through time-based scheduling.

## Core Components

### 1. Path Finding Algorithm (Dijkstra)

The system uses Dijkstra's algorithm to find the shortest path between any two nodes:

```python
def dijkstra(start, end):
    # Initialize distances to infinity
    # Use priority queue for efficient node selection
    # Track previous nodes for path reconstruction
    # Return (path, distance)
```

**Key features:**

- Caching mechanism to avoid recalculating same paths
- Bidirectional edge support
- Weighted graph traversal

### 2. Assignment Optimization

The system finds the optimal assignment of AGVs to destinations to minimize total completion time.

#### For 2 AGVs (Common Case):

```
Assignment 1: AGV1→Dest1, AGV2→Dest2
Assignment 2: AGV1→Dest2, AGV2→Dest1
```

The algorithm:

1. Calculates travel time for each AGV based on distance and speed
2. Total time = max(AGV1_time, AGV2_time) since AGVs move in parallel
3. Selects assignment with minimum total time

#### For N AGVs:

- Uses permutation to try all possible assignments
- Complexity: O(N!) - suitable for small N

### 3. Speed Optimization

Each AGV can have different speeds:

```python
travel_time = distance / speed
```

The optimization now minimizes time instead of distance:

- Fast AGVs may take longer routes if it results in faster completion
- Slow AGVs are assigned to shorter paths when possible

## Collision Avoidance Mechanism

### 1. Time-Based Scheduling

Each AGV's path is converted to a timed schedule:

```python
[(node1, arrival_time1), (node2, arrival_time2), ...]
```

### 2. Collision Detection

Two types of conflicts are detected:

#### Node Occupation Conflict

When two AGVs occupy the same node at overlapping times:

```python
# AGV1 at node X from time 10.0 to 10.5
# AGV2 at node X from time 10.3 to 10.8
# → Collision detected!
```

#### Time Buffer

- Each AGV occupies a node for 0.5 time units
- This accounts for the physical space needed during traversal

### 3. Collision Resolution

The `adjust_path_timing` function implements collision avoidance:

```python
def adjust_path_timing(path, start_time, agv_id, other_schedules):
    for each node in path:
        1. Calculate arrival time based on speed
        2. Check if node is occupied by another AGV
        3. If conflict found:
           - Add wait time (0.5 units increments)
           - Re-check until no conflict
        4. Update schedule with adjusted time
```

**Key strategies:**

- **First-Come-First-Served**: Earlier scheduled AGVs have priority
- **Incremental Delays**: Wait times added in 0.5 unit increments
- **Maximum Wait Time**: Prevents infinite loops (max 10 units)

### 4. Post-Planning Verification

After all paths are scheduled, the system:

1. Performs final collision check between all AGV pairs
2. Reports any remaining conflicts as warnings
3. Provides detailed collision information (node, times, AGVs involved)

## Example Scenario

Consider 2 AGVs with different speeds:

- AGV1: Speed 2.0 units/sec
- AGV2: Speed 1.0 units/sec

Path intersection at Node C:

```
AGV1: A → B → C → D (distance: 6 units)
AGV2: E → F → C → G (distance: 4 units)

Without collision avoidance:
- AGV1 reaches C at time 1.5
- AGV2 reaches C at time 2.0
→ Near collision!

With collision avoidance:
- AGV1 reaches C at time 1.5-2.0
- AGV2 waits, reaches C at time 2.5-3.0
→ Safe passage!
```

## Usage Examples

### Basic Usage

```bash
python multi_agv_planner.py
```

### With Custom Speeds

```bash
python multi_agv_planner.py --agv-speeds "AGV1:2.0,AGV2:1.5"
```

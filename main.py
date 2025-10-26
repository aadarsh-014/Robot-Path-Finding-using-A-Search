import heapq, random, math, time
import matplotlib.pyplot as plt

N = 20

def generate_grid(n=20, obstacle_prob=0.2):
    grid = [[0 if random.random() > obstacle_prob else 1 for _ in range(n)] for _ in range(n)]
    start, goal = (0, 0), (n-1, n-1)
    grid[start[0]][start[1]] = grid[goal[0]][goal[1]] = 0
    return grid, start, goal

def heuristic(a, b, mode="manhattan"):
    if mode == "manhattan":
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    elif mode == "euclidean":
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    elif mode == "diagonal":
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

def astar(grid, start, goal, htype):
    n = len(grid)
    open_set = [(0, start)]
    g = {start: 0}
    f = {start: heuristic(start, goal, htype)}
    came_from = {}
    visited = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        visited += 1
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return len(path), visited

        for dx, dy in [(0,1),(1,0),(-1,0),(0,-1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < n and 0 <= neighbor[1] < n and not grid[neighbor[0]][neighbor[1]]:
                temp_g = g[current] + 1
                if neighbor not in g or temp_g < g[neighbor]:
                    came_from[neighbor] = current
                    g[neighbor] = temp_g
                    f[neighbor] = temp_g + heuristic(neighbor, goal, htype)
                    heapq.heappush(open_set, (f[neighbor], neighbor))
    return None, visited

def compare():
    heuristics = ["manhattan", "euclidean", "diagonal"]
    results = {h: [] for h in heuristics}

    for h in heuristics:
        for _ in range(10):
            grid, start, goal = generate_grid()
            t1 = time.time()
            path_len, visited = astar(grid, start, goal, h)
            t2 = time.time()
            results[h].append((path_len or 0, visited, t2 - t1))
    for h in heuristics:
        print(f"\nHeuristic: {h}")
        print(f"Avg Path: {sum(p for p,_,_ in results[h])/10:.2f}, Avg Nodes: {sum(v for _,v,_ in results[h])/10:.2f}, Avg Time: {sum(t for _,_,t in results[h])/10:.4f}s")

compare()

import heapq
import time

# Grid kota dengan rintangan dan simbol
city_map = [
    ['.', '.', '.', '.', 'C'],
    ['.', '#', '#', '.', '.'],
    ['R', '.', '.', '#', '.'],
    ['.', '#', '.', '.', '.'],
    ['.', '.', '.', '#', '.']
]

rows = len(city_map)
cols = len(city_map[0])
directions = [(-1,0), (1,0), (0,-1), (0,1)]

def find_position(symbol):
    for i in range(rows):
        for j in range(cols):
            if city_map[i][j] == symbol:
                return (i, j)
    return None

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan

def a_star(start, goal):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        explored_nodes += 1

        if current == goal:
            break

        for d in directions:
            ni, nj = current[0] + d[0], current[1] + d[1]
            neighbor = (ni, nj)

            if 0 <= ni < rows and 0 <= nj < cols:
                if city_map[ni][nj] != '#':
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, goal)
                        heapq.heappush(frontier, (priority, neighbor))
                        came_from[neighbor] = current

    # Rekonstruksi path
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()

    return path, cost_so_far.get(goal, float('inf')), explored_nodes

# Mulai dari R ke C
start = find_position('R')
goal = find_position('C')

# Eksekusi A*
start_time = time.time()
path, cost, nodes = a_star(start, goal)
end_time = time.time()

# Tampilkan hasil
print("Path:", path)
print("Cost:", cost)
print("Nodes explored:", nodes)
print("Time taken (ms):", (end_time - start_time) * 1000)

# Visualisasi
print("\nMap with path:")
for i in range(rows):
    row = ""
    for j in range(cols):
        if (i, j) in path and city_map[i][j] == '.':
            row += 'o '
        else:
            row += city_map[i][j] + ' '
    print(row)

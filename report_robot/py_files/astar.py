def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(maze, start, end):
    start=start[::-1]
    end=end[::-1]
    moves = [(0, 1), (0, -1), (1, 0), (-1, 0),
             (1, 1), (-1, -1), (-1, 1), (1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, end)}
    oheap = []

    def neighbors_available(position, maze, moves):
        neighbors = []
        diagonal=[]
        for move in moves[0:4]:
            x_test = position[0]+move[0]
            y_test = position[1]+move[1]
            maze_test = maze[x_test, y_test]
            if x_test >= 0 and x_test < maze.shape[0]-1:
                if y_test >= 0 and y_test < maze.shape[1]-1:
                    if maze_test != 1:
                        neighbors.append((x_test, y_test))
                        diagonal.append(False)
                        
                        
        for move in moves[4:]:
            x_test = position[0]+move[0]
            y_test = position[1]+move[1]
            maze_test = maze[x_test, y_test]
            if x_test >= 0 and x_test < maze.shape[0]-1:
                if y_test >= 0 and y_test < maze.shape[1]-1:
                    if maze_test != 1:
                        neighbors.append((x_test, y_test))
                        diagonal.append(True)


        return zip(neighbors, diagonal)

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == end:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return [(x[1], x[0]) for x in data]#[::-1]]

        close_set.add(current)
        for neighbor,diag in neighbors_available(current, maze, moves):
            if diag:   
                tentative_g_score = gscore[current] + 1.4
            else:
                tentative_g_score= gscore[current] + 1

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False
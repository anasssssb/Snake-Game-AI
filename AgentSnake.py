import heapq
from collections import deque

# Cell class definition remains the same

class Agent(object):
    def SearchSolution(self, state):
        return []

class AStarAgent(Agent):
    def __init__(self):
        self.name = "AStar"

    def heuristic(self, current, goal):
        """
        Manhattan distance heuristic.
        """
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_valid_neighbors(self, node, state):
        """
        Returns valid neighbors of the current node in the maze.
        """
        maze = state.maze.MAP
        valid_neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        for dr, dc in directions:
            r, c = node[0] + dr, node[1] + dc
            if 0 <= r < state.maze.HEIGHT and 0 <= c < state.maze.WIDTH and maze[r][c] != -1:
                valid_neighbors.append((r, c))
        return valid_neighbors

    def search_solution(self, state):
        start = (state.snake.HeadPosition.Y, state.snake.HeadPosition.X)
        goal = (state.FoodPosition.Y, state.FoodPosition.X)
        open_set = []
        heapq.heappush(open_set, (0, start))  # using heap as a data structure
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current_node = heapq.heappop(open_set)

            if current_node == goal:
                break

            for next_node in self.get_valid_neighbors(current_node, state):
                new_cost = cost_so_far[current_node] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    heapq.heappush(open_set, (priority, next_node))
                    came_from[next_node] = current_node

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        # Converting found path to directions
        directions = []
        for i in range(1, len(path)):
            diff_row = path[i][0] - path[i - 1][0]
            diff_col = path[i][1] - path[i - 1][1]
            if diff_row == 1:
                directions.append(6)  # Down
            elif diff_row == -1:
                directions.append(0)  # Up
            elif diff_col == 1:
                directions.append(3)  # Right
            elif diff_col == -1:
                directions.append(9)  # Left

        return directions

class GreedyBestFirstAgent(Agent):
    def __init__(self):
        self.name = "Greedy"

    def heuristic(self, current, goal):
        """
        Manhattan distance heuristic.
        """
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_valid_neighbors(self, node, state):
        """
        Returns valid neighbors of the current node in the maze.
        """
        maze = state.maze.MAP
        valid_neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        for dr, dc in directions:
            r, c = node[0] + dr, node[1] + dc
            if 0 <= r < state.maze.HEIGHT and 0 <= c < state.maze.WIDTH and maze[r][c] != -1:
                valid_neighbors.append((r, c))
        return valid_neighbors

    def search_solution(self, state):
        start_position = (state.snake.HeadPosition.Y, state.snake.HeadPosition.X)
        goal_position = (state.FoodPosition.Y, state.FoodPosition.X)
        open_set = []  # Heap as data structure
        heapq.heappush(open_set, (self.heuristic(start_position, goal_position), start_position))
        came_from = {}

        while open_set:
            _, current_node = heapq.heappop(open_set)

            if current_node == goal_position:
                break

            for next_node in self.get_valid_neighbors(current_node, state):
                if next_node not in came_from:
                    heapq.heappush(open_set, (self.heuristic(next_node, goal_position), next_node))
                    came_from[next_node] = current_node

        path = []
        current = goal_position
        while current != start_position:
            path.append(current)
            current = came_from[current]
        path.append(start_position)
        path.reverse()

        # Converting found path to directions
        directions = []
        for i in range(1, len(path)):
            diff_row = path[i][0] - path[i - 1][0]
            diff_col = path[i][1] - path[i - 1][1]
            if diff_row == 1:
                directions.append(6)  # Down
            elif diff_row == -1:
                directions.append(0)  # Up
            elif diff_col == 1:
                directions.append(3)  # Right
            elif diff_col == -1:
                directions.append(9)  # Left

        return directions

class DepthFirstAgent(Agent):
    def __init__(self):
        self.name = "DFS"

    def get_neighbors(self, node, maze):
        """
        Returns valid neighbors of the current node in the maze.
        """
        neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        for dr, dc in directions:
            r, c = node[0] + dr, node[1] + dc
            if 0 < r < len(maze) - 1 and 0 < c < len(maze[0]) - 1 and maze[r][c] != -1:
                neighbors.append((r, c))
        return neighbors

    def search_solution(self, state):
        start = (state.snake.HeadPosition.Y, state.snake.HeadPosition.X)
        goal = (state.FoodPosition.Y, state.FoodPosition.X)
        stack = [(start, [start])]  # Stack for DFS traversal
        visited = set()

        while stack:
            current_node, path = stack.pop()

            if current_node == goal:
                return self.convert_path_to_directions(path)

            visited.add(current_node)

            for next_node in self.get_neighbors(current_node, state.maze.MAP):
                if next_node not in visited:
                    stack.append((next_node, path + [next_node]))
                    visited.add(next_node)

        return []

    def convert_path_to_directions(self, path):
        directions = []
        for i in range(1, len(path)):
            diff_row = path[i][0] - path[i - 1][0]
            diff_col = path[i][1] - path[i - 1][1]
            if diff_row == 1:
                directions.append(6)  # Down
            elif diff_row == -1:
                directions.append(0)  # Up
            elif diff_col == 1:
                directions.append(3)  # Right
            elif diff_col == -1:
                directions.append(9)  # Left
        return directions

class AgentSnake(Agent):
    def SearchSolution(self, state, algorithm="A*"):
        if algorithm == "A*":
            solver = AStarAgent()
        elif algorithm == "GreedyBFS":
            solver = GreedyBestFirstAgent()
        elif algorithm == "DFS":
            solver = DepthFirstAgent()
        else:
            print("Unsupported search algorithm. Defaulting to A*.")
            solver = AStarAgent()

        return solver.search_solution(state)

    @staticmethod
    def showAgent():
        print("A Snake Solver By MB")
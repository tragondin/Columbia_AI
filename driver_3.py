from heapq import heappush, heappop

from queue import Queue

import time

import resource

import sys

import math


# The Class that Represents the Puzzle
class PuzzleState(object):
    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0):

        if n * n != len(config) or n < 2:
            raise Exception("the length of config is not correct!")

        self.n = n

        self.cost = cost

        self.parent = parent

        self.action = action

        self.dimension = n

        self.config = config

        self.children = []

        self.number = ""

        self.action_order = ["Initial", "Up", "Down", "Left", "Right"].index(action)

        for i, item in enumerate(self.config):

            if item == 0:
                self.blank_row = int(i / self.n)
                self.blank_col = i % self.n

                break

    # "pretty" representation of the puzzle's state
    def display(self):
        result = ""
        for i in range(self.n):

            line = []

            offset = i * self.n

            for j in range(self.n):
                line.append(self.config[offset + j])
            result = result + str(line) + "\n"
        return result

    # return a string representation of the puzzle's state
    # by simply listing each number one after another with no delimiter
    def to_number(self):
        if self.number == "":
            for i in range(self.n):
                offset = i * self.n
                for j in range(self.n):
                    self.number = self.number + str(self.config[offset + j])
        return self.number

    def move_left(self):

        if self.blank_col == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1)

    def move_right(self):

        if self.blank_col == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1)

    def move_up(self):

        if self.blank_row == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1)

    def move_down(self):
        if self.blank_row == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1)

    def expand(self):

        """expand the node"""

        # add child nodes in order of UDLR

        if len(self.children) == 0:

            up_child = self.move_up()

            if up_child is not None:
                self.children.append(up_child)

            down_child = self.move_down()

            if down_child is not None:
                self.children.append(down_child)

            left_child = self.move_left()

            if left_child is not None:
                self.children.append(left_child)

            right_child = self.move_right()

            if right_child is not None:
                self.children.append(right_child)

        return self.children

    def get_n(self):
        return self.n

    def get_config(self):
        return self.config

    # added this method to help decide between two nodes with the same priority in heapq
    def __lt__(self, other):
        return self.to_number() <= other.to_number()

# Function that appends to output.txt
def write_output(final_state, nodes_expanded, search_depth, max_search_depth, run_time, max_ram):
    # return path is constructed by walking back from the goal all the way to the start
    return_path = []
    state = final_state
    # test if we have reached the root node
    while state.action != "Initial":
        # we started at the end, every node we encounter came before the previous one
        # so we add its action at the beginning of our list
        return_path.insert(0, state.action)
        state = state.parent

    output_txt = "path_to_goal: {0}\ncost_of_path: {1}\nnodes_expanded: {2}\nsearch_depth: {3}\n" \
                 "max_search_depth: {4}\nrunning_time: {5}\nmax_ram_usage: {6}\n".format(
                 str(return_path), str(final_state.cost), str(nodes_expanded), str(search_depth), str(max_search_depth),
                 str(run_time), str(max_ram))
    output_file = open("output.txt", "a+")
    output_file.write(output_txt)
    output_file.close()


# implementation of bsf tree search
def bfs_search(initial_state):
    # initialize various counters and timers
    start_time = time.time()
    # good ole queue
    q = Queue()
    state = initial_state
    # for bfs, it's not possible to find a state that has a lower cost than a previously encountered state
    # so keeping track of states in a set works: unique keys and fast check
    visited = {state.to_number()}
    count = 0
    max_search = 0

    # we'll loop until we've found a state that matches the goal condition
    while not test_goal(state):
        for child in state.expand():
            if child.to_number() not in visited:
                visited.add(child.to_number())
                q.put(child)
                max_search = max(max_search, child.cost)
        # we've expanded state so increase our counter by 1
        count += 1
        # next!!!
        state = q.get()

    # let's get our stats and print our results
    max_ram = resource.getrusage(resource.RUSAGE_SELF)[2] / 2. ** 20
    run_time = time.time() - start_time
    write_output(state, count, state.cost, max_search, run_time, max_ram)


# implementation of dfs tree search
def dfs_search(initial_state):
    # initialize various counters and timers
    start_time = time.time()
    stack = []
    state = initial_state
    # we don't care if a previously visited state has a higher cost than the same state
    # encountered in a different path, so we just use a set (keys are unique, sets are fast)
    visited = {state.to_number()}
    count = 0
    max_search = 0

    # we'll loop until we've found a state that matches the goal condition
    while not test_goal(state):
        # instructions are to go over children in UDLR order,
        # so we reverse the list return by state.expand, so that they are added to the stack
        # in reverse order but are consumed in UDLR order.
        for child in reversed(state.expand()):
            # if a child has not been visited OR
            # add to our set of visited,
            # put in the stack
            if child.to_number() not in visited:
                visited.add(child.to_number())
                stack.append(child)
                max_search = max(max_search, child.cost)
        # next!!
        state = stack.pop()
        # we've expanded state so increase our counter by 1
        count += 1

    # let's get our stats and print our results
    max_ram = resource.getrusage(resource.RUSAGE_SELF)[2] / 2. ** 20
    run_time = time.time() - start_time
    write_output(state, count, state.cost, max_search, run_time, max_ram)


# implementation of A_Star tree search
def a_star_search(initial_state):
    # initialize various counters and timers
    start_time = time.time()
    # we'll use a heapq list for the priority queue
    q = []
    state = initial_state
    # a dictionary of visited states. key is the string representation of the state.
    # we use a dictionary here because we want to not only know if we've visited a state previously
    # but if we did, we want to retrieve the distance + cost of that state
    visited = {state.to_number(): state}
    count = 0
    max_search = 0

    # we'll loop until we've found a state that matches the goal condition
    while not test_goal(state):
        for child in state.expand():
            # if a child has not been visited
            # OR if a child HAS been visited, but its distance to the goal + cost from the root
            # is less than the one we had visited before
            # add/replace in the visited dict, put in the queue with distance + cost as the priority measure
            if child.to_number() not in visited \
                    or distance(visited[child.to_number()]) + visited[child.to_number()].cost > \
                    distance(child) + child.cost:
                visited[child.to_number()] = child
                # we use the action that led to the creation of the child
                # as the second element of the tuple used to establish state priority
                # first one is distance to goal + cost to get to where we are
                heappush(q, (distance(child) + child.cost, child.action_order , child))
                max_search = max(max_search, child.cost)
        count += 1
        # next one to process is on top of the queue:
        # tuple (lowest distance + cost, UDLR, state)
        state = heappop(q)[2]

    # let's get our stats and print our results
    max_ram = resource.getrusage(resource.RUSAGE_SELF)[2] / 2. ** 20
    run_time = time.time() - start_time
    write_output(state, count, state.cost, max_search, run_time, max_ram)


def a_star_search_2(initial_state):
    # initialize various counters and timers
    start_time = time.time()
    # we'll use a heapq list for the priority queue
    visited = {}
    q = []
    heappush(q, (distance(initial_state) + initial_state.cost, initial_state.action_order, initial_state))
    # a dictionary of visited states. key is the string representation of the state.
    # we use a dictionary here because we want to not only know if we've visited a state previously
    # but if we did, we want to retrieve the distance + cost of that state

    count = 0
    max_search = 0

    while len(q) != 0:
        state = heappop(q)[2]
        visited[state.to_number()] = state
        if test_goal(state):
            max_ram = resource.getrusage(resource.RUSAGE_SELF)[2] / 2. ** 20
            run_time = time.time() - start_time
            write_output(state, count, state.cost, max_search, run_time, max_ram)
            return

        for child in state.expand():
            if child.to_number() not in visited \
                    or distance(visited[child.to_number()]) + visited[child.to_number()].cost > \
                    distance(child) + child.cost:
                visited[child.to_number()] = child
                # we use the action that led to the creation of the child
                # as the second element of the tuple used to establish state priority
                # first one is distance to goal + cost to get to where we are
                heappush(q, (distance(child) + child.cost, child.action_order, child))
                max_search = max(max_search, child.cost)
        count += 1
        # next one to process is on top of the queue:
        # tuple (lowest distance + cost, state)
        # state = heappop(q)[1]

    # let's get our stats and print our results



def calculate_total_cost(state):
    return state.cost


# Manhattan distance between state and goal
# for each tile (except blank = 0), sum the abs of the diff of tile's x and goal tile's x
# and abs of tile's y and goal tile's y
def distance(puzzle_state):
    total_distance = 0

    for element in range(1, puzzle_state.get_n() ** 2):
        goal_tile_x = element % puzzle_state.get_n()
        goal_tile_y = element // puzzle_state.get_n()
        tile = puzzle_state.get_config()[element]
        tile_x = tile % puzzle_state.get_n()
        tile_y = tile // puzzle_state.get_n()
        total_distance += abs(tile_x - goal_tile_x) + abs(tile_y - goal_tile_y)

    return total_distance


# to test if a puzzle is the goal, compare a string representation of it to the goal's string representation
# for n=3, goal str is "012345678"
def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    goal = "".join(str(x) for x in range(puzzle_state.get_n() ** 2))
    return puzzle_state.to_number() == goal


# Main Function that reads in Input and Runs corresponding Algorithm

def main():
    sm = sys.argv[1].lower()

    begin_state = sys.argv[2].split(",")

    begin_state = tuple(map(int, begin_state))

    size = int(math.sqrt(len(begin_state)))

    hard_state = PuzzleState(begin_state, size)

    if sm == "bfs":

        bfs_search(hard_state)

    elif sm == "dfs":

        dfs_search(hard_state)

    elif sm == "ast":

        a_star_search(hard_state)

    elif sm == "ast2":

        a_star_search_2(hard_state)
    else:

        print("Enter valid command arguments !")


if __name__ == '__main__':
    main()

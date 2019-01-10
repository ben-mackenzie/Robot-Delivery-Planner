'''
=== Introduction ===

In this problem, you will build a planner that helps a robot
  find the best path through a warehouse filled with boxes
  that it has to pick up and deliver to a dropzone.

Your file must be called `partA.py` and must have a class
  called `DeliveryPlanner`.
This class must have an `__init__` function that takes three 
  arguments: `self`, `warehouse`, and `todo`.
The class must also have a function called `plan_delivery` that 
  takes a single argument, `self`.

=== Input Specifications ===

`warehouse` will be a list of m strings, each with n characters,
  corresponding to the layout of the warehouse. The warehouse is an
  m x n grid. warehouse[i][j] corresponds to the spot in the ith row
  and jth column of the warehouse, where the 0th row is the northern
  end of the warehouse and the 0th column is the western end.

The characters in each string will be one of the following:

'.' (period) : traversable space. The robot may enter from any adjacent space.
'#' (hash) : a wall. The robot cannot enter this space.
'@' (dropzone): the starting point for the robot and the space where all boxes must be delivered.
  The dropzone may be traversed like a '.' space.
[0-9a-zA-Z] (any alphanumeric character) : a box. At most one of each alphanumeric character 
  will be present in the warehouse (meaning there will be at most 62 boxes). A box may not
  be traversed, but if the robot is adjacent to the box, the robot can pick up the box.
  Once the box has been removed, the space functions as a '.' space.

For example, 
  warehouse = ['1#2',
               '.#.',
               '..@']
  is a 3x3 warehouse.
  - The dropzone is at the warehouse cell in row 2, column 2.
  - Box '1' is located in the warehouse cell in row 0, column 0.
  - Box '2' is located in the warehouse cell in row 0, column 2.
  - There are walls in the warehouse cells in row 0, column 1 and row 1, column 1.
  - The remaining five warehouse cells contain empty space.
#
The argument `todo` is a list of alphanumeric characters giving the order in which the 
  boxes must be delivered to the dropzone. For example, if 
  todo = ['1','2']
  is given with the above example `warehouse`, then the robot must first deliver box '1'
  to the dropzone, and then the robot must deliver box '2' to the dropzone.

=== Rules for Movement ===

- Two spaces are considered adjacent if they share an edge or a corner.
- The robot may move horizontally or vertically at a cost of 2 per move.
- The robot may move diagonally at a cost of 3 per move.
- The robot may not move outside the warehouse.
- The warehouse does not "wrap" around.
- As described earlier, the robot may pick up a box that is in an adjacent square.
- The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
- While holding a box, the robot may not pick up another box.
- The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
  of 2 (regardless of the direction in which the robot puts down the box).
- If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
  house.
- The warehouse will be arranged so that it is always possible for the robot to move to the 
  next box on the todo list without having to rearrange any other boxes.

An illegal move will incur a cost of 100, and the robot will not move (the standard costs for a 
  move will not be additionally incurred). Illegal moves include:
- attempting to move to a nonadjacent, nonexistent, or occupied space
- attempting to pick up a nonadjacent or nonexistent box
- attempting to pick up a box while holding one already
- attempting to put down a box on a nonadjacent, nonexistent, or occupied space
- attempting to put down a box while not holding one

=== Output Specifications ===

`plan_delivery` should return a LIST of moves that minimizes the total cost of completing
  the task successfully.
Each move should be a string formatted as follows:

'move {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot moves
  to and '{j}' is replaced by the column-coordinate of the space the robot moves to

'lift {x}', where '{x}' is replaced by the alphanumeric character of the box being picked up

'down {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot puts 
  the box, and '{j}' is replaced by the column-coordinate of the space the robot puts the box

For example, for the values of `warehouse` and `todo` given previously (reproduced below):
  warehouse = ['1#2',
               '.#.',
               '..@']
  todo = ['1','2']
`plan_delivery` might return the following:
  ['move 2 1',
   'move 1 0',
   'lift 1',
   'move 2 1',
   'down 2 2',
   'move 1 2',
   'lift 2',
   'down 2 2']

=== Grading ===

- Your planner will be graded against a set of test cases, each equally weighted.
- If your planner returns a list of moves of total cost that is K times the minimum cost of 
  successfully completing the task, you will receive 1/K of the credit for that test case.
- Otherwise, you will receive no credit for that test case. This could happen for one of several 
  reasons including (but not necessarily limited to):
  - plan_delivery's moves do not deliver the boxes in the correct order.
  - plan_delivery's output is not a list of strings in the prescribed format.
  - plan_delivery does not return an output within the prescribed time limit.
  - Your code raises an exception.

=== Additional Info ===

- You may add additional classes and functions as needed provided they are all in the file `partA.py`.
- Upload partA.py to Project 2 on T-Square in the Assignments section. Do not put it into an 
  archive with other files.
- Your partA.py file must not execute any code when imported.
- Ask any questions about the directions or specifications on Piazza.

-Sorry, you can't put the box down if you are inside the dropzone...so you need to have a special test after generating the route from the pickup point back to the dropzone to make sure that you move out of the dropzone if your path didn't do that for you already.
(or if you are traveling, you should move to the closest square to the drop zone, not all the way back to the dropzone.)

-Here's the simple test case that I used to prove out the situation where the robot is standing on the dropzone where it needs to drop the box:

params = {'test_case': 11,
          'warehouse': ['1@2'],
          'todo': ['1', '2'],
          'min_cost': 20}

-A* does not necessarily have to process the entire map, so if you are just calculating one route, it may be faster than DP depending upon the map.
If you plan on making multiple trips to the same goal, DP will give you a route from "any" location in the warehouse.
Which one is easier to implement depends entirely upon your implementation skills and understandings of the two algorithms.
'''


class DeliveryPlanner:

    def __init__(self, warehouse, todo):
        self.warehouse = warehouse
        self.todo = todo
        self.delta_name = delta_name = ['^', '<', 'v', '>', '^<', 'v<', 'v>', '^>']
        self.delta = [[-1, 0],  # go up
                     [0, -1],  # go left
                     [1, 0],  # go down
                     [0, 1], #go right
                     [-1, -1],  # go up.left
                     [1, -1],  # go down.left
                     [1, 1],  # go down.right
                     [-1, 1]]# go up.right

    def plan_delivery(self):
        xd,yd = self.get_dz_coordinates()
        return_policy = self.get_policy_grid((xd,yd))
        x,y = xd,yd
        moves = []
        next_policy_calculated = False

        for b, box in enumerate(self.todo):
            goal = self.get_box_coordinates(box)
            if next_policy_calculated == True:
                policy_grid = next_policy_grid
            else:
                policy_grid = self.get_policy_grid(goal)
            found = False
            returned = False

            # travel to box
            while not found:
                # generate neighbors and check for end condition (neighbor is adjacent to box)
                cell = policy_grid[x][y]
                if cell != "#":
                    x2 = x + int(policy_grid[x][y][0])
                    y2 = y + int(policy_grid[x][y][1])
                    if x2 >= 0 and x2 < len(self.warehouse) and y2 >= 0 and y2 < len(self.warehouse[0]):
                        # if the next move is box, lift box
                        if x2 == goal[0] and y2 == goal[1]:
                            entry = 'lift ' + str(box)
                            found = True
                            self.warehouse[x2] = self.warehouse[x2][:y2] + "." + self.warehouse[x2][y2+1:]
                        # otherwise, move to the next location
                        else:
                            entry = 'move ' + str(x2) + ' ' + str(y2)
                        # update move list, x, y
                        moves.append(entry)
                        if found == False:
                            x, y = x2, y2

            while not returned:
                if x == xd and y == yd:
                    # robot is holding box on drop zone.  lift box, move to next closest box, drop box
                    next_goal = self.get_box_coordinates(self.todo[b+1])
                    next_policy_grid = self.get_policy_grid(next_goal)
                    x2 = x + int(next_policy_grid[x][y][0])
                    y2 = y + int(next_policy_grid[x][y][1])
                    if x2 == next_goal[0] and y2 == next_goal[1]:
                        # x2, y2 is loc of next box.  move to location of last box
                        x2, y2 = goal[0], goal[1]
                    moves.append('move ' + str(x2) + ' ' + str(y2))
                    moves.append('down ' + str(xd) + ' ' + str(yd))
                    next_policy_calculated = True
                    returned = True
                    x,y = x2, y2
                else:
                    # generate neighbors and check for end condition (neighbor is adjacent to box)
                    cell = return_policy[x][y]
                    if cell != "#":
                        x2 = x + int(return_policy[x][y][0])
                        y2 = y + int(return_policy[x][y][1])
                        if x2 >= 0 and x2 < len(self.warehouse) and y2 >= 0 and y2 < len(self.warehouse[0]):
                            # if the next move is drop zone, put box down
                            if x2 == xd and y2 == yd:
                                entry = 'down ' + str(xd) + ' ' + str(yd)
                                returned = True
                                next_policy_calculated = False
                            # otherwise, move to the next location
                            else:
                                entry = 'move ' + str(x2) + ' ' + str(y2)
                            # update move list, x, y
                            moves.append(entry)
                            if returned == False:
                                x, y = x2, y2
        return moves


    def get_box_coordinates(self, box):
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                if self.warehouse[i][j] == box:
                    return (i, j)

    def get_dz_coordinates(self):
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                cell = self.warehouse[i][j]
                if cell == '@':
                    return (i, j)

    def get_policy_grid(self, goal):
        value = [[99 for i in range(len(self.warehouse[0]))] for i in range(len(self.warehouse))]
        policy = [['#' for row in range(len(self.warehouse[0]))] for col in range(len(self.warehouse))]
        change = True

        while change:
            change = False

            for x in range(len(self.warehouse)):
                for y in range(len(self.warehouse[0])):
                    cell = self.warehouse[x][y]

                    if goal[0] == x and goal[1] == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            policy[x][y] = '*'
                            change = True

                    elif cell != '#':
                        for i in range(len(self.delta)):
                            if i > 3:
                                cost = 3
                            else:
                                cost = 2
                            x2 = x + self.delta[i][0]
                            y2 = y + self.delta[i][1]
                            if x2 >= 0 and x2 < len(self.warehouse) and y2 >= 0 and y2 < len(self.warehouse[0]):
                                ####################
                                if self.warehouse[x2][y2] != "#":
                                    v2 = value[x2][y2] + cost
                                    if v2 < value[x][y]:
                                        change = True
                                        value[x][y] = v2
                                        policy[x][y] = self.delta[i]
        return policy


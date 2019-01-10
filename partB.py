"""
=== Introduction ===

In this problem, you will again build a planner that helps a robot
  find the best path through a warehouse filled with boxes
  that it has to pick up and deliver to a dropzone. Unlike Part A,
  however, in this problem the robot is moving in a continuous world
  (albeit in discrete time steps) and has constraints on the amount
  it can turn its wheels in a given time step.

Your file must be called `partB.py` and must have a class
  called `DeliveryPlanner`.
This class must have an `__init__` function that takes five
  arguments: `self`, `warehouse`, `todo`, `max_distance`, and
  `max_steering`.
The class must also have a function called `plan_delivery` that
  takes a single argument, `self`.

=== Input Specifications ===

`warehouse` will be a list of m strings, each with n characters,
  corresponding to the layout of the warehouse. The warehouse is an
  m x n grid. warehouse[i][j] corresponds to the spot in the ith row
  and jth column of the warehouse, where the 0th row is the northern
  end of the warehouse and the 0th column is the western end.

The characters in each string will be one of the following:

'.' (period) : traversable space.
'' (hash) : a wall. If the robot contacts a wall space, it will crash.
'@' (dropzone): the space where all boxes must be delivered. The dropzone may be traversed like
  a '.' space.

Each space is a 1 x 1 block. The upper-left corner of space warehouse[i][j] is at the point (j,-i) in
  the plane. Spaces outside the warehouse are considered walls; if any part of the robot leaves the
  warehouse, it will be considered to have crashed into the exterior wall of the warehouse.

For example,
  warehouse = ['..',
               '..',
               '..@']
  is a 3x3 warehouse. The dropzone is at space (2,-2) and there are walls at spaces (1,0)
  and (1,-1). The rest of the warehouse is empty space.

The robot is a circle of radius 0.25. The robot begins centered in the dropzone space.
  The robot's initial bearing is 0.

The argument `todo` is a list of points representing the center point of each box.
  todo[0] is the first box which must be delivered, followed by todo[1], and so on.
  Each box is a square of size 0.2 x 0.2. If the robot contacts a box, it will crash.

The arguments `max_distance` and `max_steering` are parameters constraining the movement
  of the robot on a given time step. They are described more below.

=== Rules for Movement ===

- The robot may move any distance between 0 and `max_distance` per time step.
- The robot may set its steering angle anywhere between -`max_steering` and
  `max_steering` per time step. A steering angle of 0 means that the robot will
  move according to its current bearing. A positive angle means the robot will
  turn counterclockwise by `steering_angle` radians; a negative steering_angle
  means the robot will turn clockwise by abs(steering_angle) radians.
- Upon a movement, the robot will change its steering angle instantaneously to the
  amount indicated by the move, and then it will move a distance in a straight line in its
  new bearing according to the amount indicated move.
- The cost per move is 1 plus the amount of distance traversed by the robot on that move.

- The robot may pick up a box whose center point is within 0.5 units of the robot's center point.
- If the robot picks up a box, it incurs a total cost of 2 for that move (this already includes
  the 1-per-move cost incurred by the robot).
- While holding a box, the robot may not pick up another box.
- The robot may put a box down at a total cost of 1.5 for that move. The box must be placed so that:
  - The box is not contacting any walls, the exterior of the warehouse, any other boxes, or the robot
  - The box's center point is within 0.5 units of the robot's center point
- A box is always oriented so that two of its edges are horizontal and the other two are vertical.
- If a box is placed entirely within the '@' space, it is considered delivered and is removed from the
  warehouse.
- The warehouse will be arranged so that it is always possible for the robot to move to the
  next box on the todo list without having to rearrange any other boxes.

- If the robot crashes, it will stop moving and incur a cost of 100*distance, where distance
  is the length it attempted to move that move. (The regular movement cost will not apply.)
- If an illegal move is attempted, the robot will not move, but the standard cost will be incurred.
  Illegal moves include (but are not necessarily limited to):
    - picking up a box that doesn't exist or is too far away
    - picking up a box while already holding one
    - putting down a box too far away or so that it's touching a wall, the warehouse exterior,
      another box, or the robot
    - putting down a box while not holding a box

=== Output Specifications ===

`plan_delivery` should return a LIST of strings, each in one of the following formats.

'move {steering} {distance}', where '{steering}' is a floating-point number between
  -`max_steering` and `max_steering` (inclusive) and '{distance}' is a floating-point
  number between 0 and `max_distance`

'lift {b}', where '{b}' is replaced by the index in the list `todo` of the box being picked up
  (so if you intend to lift box 0, you would return the string 'lift 0')

'down {x} {y}', where '{x}' is replaced by the x-coordinate of the center point of where the box
  will be placed and where '{y}' is replaced by the y-coordinate of that center point
  (for example, 'down 1.5 -2.9' means to place the box held by the robot so that its center point
  is (1.5,-2.9)).

=== Grading ===

- Your planner will be graded against a set of test cases, each equally weighted.
- Each task will have a "baseline" cost. If your set of moves results in the task being completed
  with a total cost of K times the baseline cost, you will receive 1/K of the credit for the
  test case. (Note that if K < 1, this means you earn extra credit!)
- Otherwise, you will receive no credit for that test case. This could happen for one of several
  reasons including (but not necessarily limited to):
  - plan_delivery's moves do not deliver the boxes in the correct order.
  - plan_delivery's output is not a list of strings in the prescribed format.
  - plan_delivery does not return an output within the prescribed time limit.
  - Your code raises an exception.

=== Additional Info ===

- You may add additional classes and functions as needed provided they are all in the file `partB.py`.
- Your partB.py file must not execute any code when it is imported.
- Upload partB.py to Project 2 on T-Square in the Assignments section. Do not put it into an
  archive with other files.
- Ask any questions about the directions or specifications on Piazza.


minheap: https://docs.python.org/2/library/heapq.html

To avoid having the robot turn at each grid cell (each .1 length step, in your case),
you'll have to step through your moves list and keep only the movements that do not have
a line-of-sight with the subsequent movement. This is effectively the Theta* algorithm.
Converting the moves into a distance and steering is just trigonometry at that point.

"""

from math import *
from copy import *
from robot import Robot

class DeliveryPlanner:

    def __init__(self, warehouse, todo, max_distance, max_steering):
        self.todo = todo
        self.factor = 7.
        self.box = 0
        self.boxes = [b for b in range(len(self.todo))]
        self.box_corners = []
        self.todo_expanded = [(t[0] * self.factor, t[1] * self.factor) for t in self.todo]
        self.warehouse = warehouse
        self.drop_zone = self.get_dz_coordinates()
        self.expanded = self.expand_warehouse()
        self.closed = []
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # go up.left
                      [1, -1],  # go down.left
                      [1, 1],  # go down.right
                      [-1, 1]]  # go up.right
        self.bot_radius = 0.25
        self.box_radius = 0.1
        self.circle_to_square_margin = sqrt(2 * self.bot_radius ** 2) - self.bot_radius
        self.threshold = self.bot_radius + self.box_radius + self.circle_to_square_margin
        self.scaled_threshold = self.threshold * self.factor
        self.max_drop_distance = 0.8

    def plan_delivery(self):
        moves = []
        init_bot_pos = self.drop_zone
        init_bot_pos_expanded = [init_bot_pos[0] * self.factor, init_bot_pos[1]*self.factor]
        robot = Robot(x=init_bot_pos[0], y=init_bot_pos[1], max_distance=self.max_distance, max_steering=self.max_steering)


        for box, expanded_box_pos in enumerate(self.todo_expanded):
            self.box = box

            # CREATE A* PATH

            heuristic = self.calculate_heuristic(expanded_box_pos)
            unsmoothed_path = self.unsmooth_path(heuristic, init_bot_pos_expanded, expanded_box_pos)
            consolidated_path = self.consolidate_path(unsmoothed_path)
            consolidated_path = [[c[0]/self.factor, c[1]/self.factor] for c in consolidated_path]

            # TRAVEL TO BOX

            box_found = False
            i = 0
            while box_found == False:
                bot_pos = (robot.x, robot.y)
                box_pos = self.todo[box]
                box_distance, box_bearing = robot.measure_distance_and_bearing_to(box_pos)

                # if box is within pickup range, pick up box and truncate path (make last entry in path the last location)
                if box_distance < 0.5:
                    # Lift
                    move = "lift " + str(self.box)
                    moves.append(move)
                    box_found = True
                    # Create truncated return path
                    return_path = consolidated_path[: i+1]

                # If box is too far to pick up, keep moving toward box.
                else:
                    if i < len(consolidated_path) - 1:
                        next = consolidated_path[i+1]
                    else:
                        next = box_pos
                    distance, steering = robot.measure_distance_and_bearing_to(next)

                    #if distance is greater than max_distance, break up translation into multiple translations
                    if distance > self.max_distance:
                        rem_distance = self.max_distance
                        while distance > self.max_distance:
                            distance = self.max_distance
                            rem_distance -= self.max_distance
                            robot.move(0, distance)
                            move = "move " + str(0) + " " + str(distance)
                            moves.append(move)
                        distance = rem_distance

                    #if robot will run into box with current distance, truncate distance
                    if i <= len(consolidated_path) - 1 and distance > box_distance - self.threshold:
                        distance = box_distance - self.threshold

                    # if turn is larger than max_steering, break up turn into multiple turns
                    if abs(steering) > self.max_steering:
                        rem_steering = steering

                        # while steering exceeds max
                        while abs(rem_steering) > self.max_steering:
                            if rem_steering < 0:
                                steering = -self.max_steering
                                rem_steering += self.max_steering
                            else:
                                steering = self.max_steering
                                rem_steering -= self.max_steering
                            robot.move(steering, 0)
                            move = "move " + str(steering) + " " + str(0)
                            moves.append(move)
                        steering = rem_steering

                    # move normally when steering and distance are below thresholds
                    robot.move(steering, distance)
                    move = "move " + str(steering) + " " + str(distance)
                    moves.append(move)
                    i += 1

            # RETURN TO DROP ZONE

            return_path.reverse()
            dropped = False
            i = 0
            while not dropped and i < len(return_path):
                distance_to_dz, dz_bearing = robot.measure_distance_and_bearing_to(self.drop_zone)
                if distance_to_dz < self.max_drop_distance:
                    # Drop
                    drop_coordinates = self.get_drop_coordinates(robot)
                    move = "down " + str(drop_coordinates[0]) + " " + str(drop_coordinates[1])
                    moves.append(move)
                    dropped = True
                else:
                    next = return_path[i + 1]
                    distance, bearing = robot.measure_distance_and_bearing_to(next)
                    steering = bearing

                    # If distance is greater than max_distance, break up translation into multiple translations
                    if distance > self.max_distance:
                        rem_distance = self.max_distance
                        while distance > self.max_distance:
                            distance = self.max_distance
                            rem_distance -= self.max_distance
                            robot.move(0, distance)
                            move = "move " + str(0) + " " + str(distance)
                            moves.append(move)
                        distance = rem_distance

                    # If turn is larger than max_steering, break up turn into multiple turns
                    if abs(steering) > self.max_steering:
                        rem_steering = steering
                        if steering < 0:
                            while rem_steering < -self.max_steering:
                                steering = -self.max_steering
                                rem_steering += self.max_steering
                        else:
                            while rem_steering > self.max_steering:
                                steering = self.max_steering
                                rem_steering -= self.max_steering
                        robot.move(steering, 0)
                        move = "move " + str(steering) + " " + str(0)
                        moves.append(move)
                        steering = rem_steering

                    # move normally when steering and distance are below thresholds
                    robot.move(steering, distance)
                    move = "move " + str(steering) + " " + str(distance)
                    moves.append(move)
                    i += 1

        return moves

    def unsmooth_path(self, heuristic, init, goal):

        # generate A* grid from params
        init = (int(init[0]), int(init[1]))
        goal = (int(goal[0]), int(goal[1]))
        grid = self.expanded
        closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
        closed[-init[1]][init[0]] = 1  # closed[-init[1] - 5][init[0] + 5] = 1
        a_star_grid = [[99999 for col in range(len(grid[0]))] for row in range(len(grid))]
        action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

        x = init[0]
        y = init[1]
        g = 0
        h = heuristic[-y][x]
        f = g + h
        open = [[f, g, x, y]]

        found = False  # flag that is set when search is complete
        resign = False  # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open) == 0:
                resign = True
                return "Fail"
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[3]
                g = next[1]
                a_star_grid[-y][x] = count
                count += 1

                if x == goal[0] and y == goal[1]:
                    found = True
                else:
                    for i in range(len(self.delta)):
                        if i > 3:
                            cost = 10
                        else:
                            cost = 14
                        x2 = x + self.delta[i][1]
                        y2 = y - self.delta[i][0]
                        if x2 >= 0 and x2 < len(grid[0]) and -y2 >= 0 and -y2 < len(grid):
                            closed_val = closed[-y2][x2]
                            grid_val = grid[-y2][x2]
                            other_boxes = copy(self.boxes)
                            other_boxes.remove(self.box)

                            if closed_val == 0 and grid_val != '#':
                                if grid_val not in other_boxes:
                                    h = heuristic[-y2][x2]
                                    g2 = g + cost
                                    f = g2 + h
                                    open.append([f, g2, x2, y2])
                                    closed[-y2][x2] = 1
                                    action[-y2][x2] = i

        # create path from A* expansion and action grid

        position = goal
        path = []
        g = g2
        while (position[0], position[1]) != init:
            x = position[0]
            y = position[1]
            i = action[-y][x]  # i = action[-y-5][x+5]
            move = [e * -1 for e in self.delta[i]]
            x2 = x + move[1]
            y2 = y - move[0]
            position = [x2, y2]
            path.append([x2, y2])
        path.reverse()
        self.closed = closed

        return path

    def line_of_sight(self, p1, p2):
        clearance = (0.25 + 0.1*sqrt(2)) * self.factor
        x0 = p1[0]
        y0 = p1[1]
        x1 = p2[0]
        y1 = p2[1]
        dy = y1 - y0
        dx = x1 - x0
        f = 0
        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1
        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1
        if dx >= dy:
            while x0 != x1:
                f = f + dy
                if f >= dx:
                    celly = y0 + ((sy - 1) / 2)
                    cellx = x0 + ((sx - 1) / 2)
                    cell = self.closed[-(celly)][cellx]
                    if cell == 0:
                        return False
                    y0 = y0 + sy
                    f = f - dx
                celly = y0 + ((sy - 1) / 2)
                cellx = x0 + ((sx - 1) / 2)
                cell = self.closed[-(celly)][cellx]
                if f != 0 and cell == 0:
                    return False

                cell1x = x0 + ((sx - 1) / 2)
                cell2x = x0 + ((sx - 1) / 2)
                cell2y = (y0 - 1)
                cell1 = self.closed[-y0][cell1x]
                cell2 = self.closed[-cell2y][cell2x]
                if dy == 0 and cell1 == 0 and cell2 == 0:
                    return False
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx
                if f >= dy:
                    celly = (y0 + ((sy - 1) / 2))
                    cellx = x0 + ((sx - 1) / 2)
                    cell = self.closed[-celly][cellx]
                    if cell == '0':
                        return False
                    x0 = x0 + sx
                    f = f - dy
                celly = y0 + ((sy - 1) / 2)
                cellx = x0 + ((sx - 1) / 2)
                cell = self.closed[-celly][cellx]
                if f != 0 and cell == 0:
                    return False
                cell1y = (y0 + ((sx - 1) / 2))
                cell2x = x0 - 1
                cell2y = (y0 + ((sx - 1) / 2))
                cell1 = self.closed[-cell1y][x0]
                cell2 = self.closed[-cell2y][cell2x]
                if dx == 0 and cell1 == 0 and cell2 == 0:
                    return False
                y0 = y0 + sy
        return True

    def proximity_alert(self, p):
        for corner_set in self.box_corners:
            for corner in corner_set:
                distance = self.distance_between(corner, p)
                if distance <= self.scaled_threshold:
                    return True
        return False

    def consolidate_path(self, path):
        i = 0
        p2_index = 1
        consolidated_path = [path[0]]

        # keep looking for line-of-sight paths to consolidate until end of path
        while i < len(path) - 1 and p2_index < len(path):
            same_line = True
            p2_index = i + 1
            p1 = path[i]
            distance = 0

            # build sup-path until line of sight is lost or path ends
            while same_line == True and p2_index < len(path):
                p2 = path[p2_index]
                proximity_alert = self.proximity_alert(p2)
                if self.line_of_sight(p1, p2) and proximity_alert == False:
                    # the position being considered is superfluous.  consider the next position.
                    if p2_index == len(path) - 1:
                    #the last point (@ p2_index) is last point in path
                        consolidated_path.append(path[p2_index])
                        p2_index += 1
                    else:
                        p2_index += 1
                else:
                    # p2 lacks line of sight with p1 or is too close to an obstacle.  End sub-path.
                    same_line = False
                    # Add (path[p2_index -1]) to path
                    next_point = path[p2_index - 1]
                    consolidated_path.append(next_point)
                    # Start new sub-path, disqualifying points too close to obstacles
                    clear_of_obstacles = False
                    at_end = False
                    new_path_start = p2_index
                    while not clear_of_obstacles and not at_end: # and new_path_start < len(path):
                        if new_path_start == len(path) - 1:
                            at_end = True
                            same_line = False
                            i = new_path_start
                        elif self.proximity_alert(path[new_path_start]) == False:
                            i = new_path_start
                            clear_of_obstacles = True
                        else:
                            new_path_start += 1

        return consolidated_path

    def calculate_heuristic(self, goal):
        # creates heuristic bsed on euclidean distance
        heuristic = [[0 for j in range(len(self.expanded[0]))] for i in range(len(self.expanded))]
        for i in range(len(heuristic)):
            for j in range(len(heuristic[0])):
                if i == goal[1] - 1 + 5 and j == -(goal[0] - 1) - 5:
                    flag = True
                elif self.expanded[i][j] == '.' or self.expanded[i][j] == '@':
                    cell = (j + 5, -i - 5)
                    cost = self.distance_between(cell, goal)
                    heuristic[i][j] = cost
                else:
                    heuristic[i][j] = 999
        return heuristic

    def expand_warehouse(self):
        n_rows = len(self.warehouse) * int(self.factor)
        n_columns = len(self.warehouse[0]) * int(self.factor)
        new_grid = [[0 for j in range(n_columns)] for i in range(n_rows)]

        # expansion

        for i in range(n_rows):
            for j in range(n_columns):
                k = i // int(self.factor)
                l = j // int(self.factor)
                new_grid[i][j] = self.warehouse[k][l]

        # pad stuff

        padded_grid = deepcopy(new_grid)
        row1 = 0
        finished = False

        # starting from origin, scan grid for obstacles
        nr = len(self.warehouse)
        nc = len(self.warehouse[0])
        for r in range(nr):
            for c in range(nc):
                cell = self.warehouse[r][c]
                if cell == '#':

                    # find borders
                    adjusted_c = c + 0.5
                    adjusted_r = r + 0.5
                    left = int(adjusted_c - 0.5) * int(self.factor)
                    right = int(adjusted_c + 0.5) * int(self.factor)
                    up =  int(adjusted_r - 0.5) * int(self.factor)
                    down =  int(adjusted_r + 0.5) * int(self.factor)
                    box_borders = [left, right, up, down]

                    #find corners for proximity alert check in Line-of-Sight function
                    upper_left = (left, -up)
                    upper_right = (right, -up)
                    lower_left = (left, -down)
                    lower_right = (right, -down)
                    corners = [upper_left, upper_right, lower_left, lower_right]
                    self.box_corners.append(corners)

                    # pad cells
                    for pc in range(left - 3, right + 3):
                        for pr in range(up - 3, down + 3):
                            if pc >= 0 and pc < len(padded_grid[0]):
                                if pr >= 0 and pr < len(padded_grid):
                                    padded_grid[pr][pc] = '#'

        #pad warehouse border
        for j, row in enumerate(padded_grid):
            # pad top and bottom
            if j < 3 or j > len(row) - 4:
                for c in range(len(new_grid[j])):
                    if new_grid[j][c] != '@':
                        padded_grid[j][c] = '#'
            last_col = len(row)
            #pad left side
            for i in range(0,3):
                if row[i] != '@':
                    row[i] = '#'
            #pad right side
            for i in range(last_col-3, last_col):
                if row[i] != '@':
                    row[i] = '#'

        # locate and pad boxes
        for i, c in enumerate(self.todo_expanded):
            left = floor(c[0] - 1.5)  # round down
            down = floor(c[1] - 1.5)  # round down
            right = ceil(c[0] + 1.5)  # round up
            up = ceil(c[1] + 1.5)  # round up
            for row in range(-int(up), -(int(down)) + 1):
                for col in range(int(left), int(right) + 1):
                    if col >= 0 and pc < len(padded_grid[0]):
                        if row >= 0 and pr < len(padded_grid):
                            padded_grid[row][col] = i

        return padded_grid

    def distance_between(self, p1, p2):
        """Computes distance between point1 and point2. Points are (x, y) pairs."""
        x1, y1 = p1
        x2, y2 = p2
        d = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    def get_dz_coordinates(self):
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                cell = self.warehouse[i][j]
                if cell == '@':
                    return (j + 0.5, -i - 0.5)

    def get_drop_coordinates(self, robot):
        dzx = float(self.drop_zone[0])
        dzy = float(self.drop_zone[1])
        y = dzy - robot.y
        x = dzx - robot.x
        theta = atan2(y, x)
        drop_x = self.drop_zone[0] - (0.3 * cos(theta))
        drop_y = self.drop_zone[1] - (0.3 * sin(theta))
        return (drop_x, drop_y)

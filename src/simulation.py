import csv 
import matplotlib.pyplot as plt
import numpy as np
import time
GRID_SIZE = 20
def draw_grid_streamlit(step, robots):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    visual = np.zeros((GRID_SIZE, GRID_SIZE))

    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):

            if grid[y][x] == 'X':
                visual[y][x] = 1
            elif grid[y][x] == 'P':
                visual[y][x] = 2
            elif grid[y][x] == 'R':
                visual[y][x] = 3
            elif grid[y][x] == 'C':
                visual[y][x] = 4
            elif grid[y][x] == 'S':
                visual[y][x] = 5

    cmap = ListedColormap(['white','black','orange','blue','green','purple','lightblue'])

    for robot in robots:
        for (tx, ty) in robot.trail:
            if visual[ty][tx] == 0:
                visual[ty][tx] = 6

    fig, ax = plt.subplots()
    ax.imshow(visual, cmap=cmap)
    ax.set_title(f"Step {step}")

    return fig

# create empty grid
grid = []

for i in range(GRID_SIZE):
    row = []
    for j in range(GRID_SIZE):
        row.append('.')
    grid.append(row)


# ---------------- GLOBAL ---------------- #
picked = set()
assigned_targets = set()


# ---------------- ROBOT CLASS ---------------- #
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.steps = 0
        self.target = None
        self.carrying = False
        self.path = []
        self.distance = 0
        self.tasks_completed = 0
        self.battery = 100
        self.max_battery = 100
        self.trail = []


# ---------------- HELPERS ---------------- #

def is_free(x, y):
    if x < 0 or y < 0 or x >= GRID_SIZE or y >= GRID_SIZE:
        return False

    # allow robot to step on target (important fix)
    return grid[y][x] != 'X'


def get_nearest_pick(robot, pick_points):
    min_dist = float('inf')
    nearest = None
    
    for p in pick_points:
        if p in picked or p in assigned_targets:
           continue
            
        dist = abs(robot.x - p[0]) + abs(robot.y - p[1])

        # add intelligence
        score = dist + robot.tasks_completed * 2

        if score < min_dist:
          min_dist = score
          nearest = p
        
            
    return nearest     


# ---------------- LOAD DATA ---------------- #

def load_robots():
    robots = []
    with open('../data/robots.csv', 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            robots.append(row)
    return robots


def load_parcels():
    parcels = []
    with open('../data/parcels.csv', 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            parcels.append(row)
    return parcels


def load_layout():
    layout = []
    with open('../data/warehouse_layout.csv', 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            layout.append(row)
    return layout


# ---------------- PLACE OBJECTS ---------------- #

def place_objects():
    robots = load_robots()
    parcels = load_parcels()
    layout = load_layout()

    # clear grid
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            grid[i][j] = '.'

    # layout
    for cell in layout:
        x = int(cell['cell_x'])
        y = int(cell['cell_y'])

        if cell['type'] == 'obstacle':
            grid[y][x] = 'X'
        elif cell['type'] == 'sorting_zone':
            grid[y][x] = 'S'
        elif cell['type'] == 'charging_station':
            grid[y][x] = 'C'

    # parcels
    for parcel in parcels:
        x = int(parcel['x_position'])
        y = int(parcel['y_position'])
        if (x, y) not in picked:
            grid[y][x] = 'P'

    # robots (initial placement only)
    for robot in robots:
        x = int(robot['x_position'])
        y = int(robot['y_position'])
        grid[y][x] = 'R'


# ---------------- PRINT ---------------- #

def print_grid():
    for row in grid:
        print(' '.join(row))


# ---------------- A* ALGORITHM ---------------- #

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal):

    open_list = [start]
    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:

        current = min(open_list, key=lambda x: f_score.get(x, float('inf')))

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        open_list.remove(current)

        x, y = current

        neighbors = [
            (x+1, y),
            (x-1, y),
            (x, y+1),
            (x, y-1)
        ]

        for nx, ny in neighbors:

            if nx < 0 or ny < 0 or nx >= GRID_SIZE or ny >= GRID_SIZE:
                continue

            if grid[ny][nx] == 'X':
                continue

            neighbor = (nx, ny)
            temp_g = g_score[current] + 1

            if temp_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score[neighbor] = temp_g + heuristic(neighbor, goal)

                if neighbor not in open_list:
                    open_list.append(neighbor)

    return []


# ---------------- SIMULATION ---------------- #
def draw_grid(step, robots):

    visual = np.zeros((GRID_SIZE, GRID_SIZE))

    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):

            if grid[y][x] == 'X':
                visual[y][x] = 1
            elif grid[y][x] == 'P':
                visual[y][x] = 2
            elif grid[y][x] == 'R':
                visual[y][x] = 3
            elif grid[y][x] == 'C':
                visual[y][x] = 4
            elif grid[y][x] == 'S':
                visual[y][x] = 5

    # 🎨 colors
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap([
        'white',    # empty
        'black',    # obstacle
        'orange',   # parcel
        'blue',     # robot
        'green',    # charging
        'purple',   # sorting
        'lightblue' # ✅ trail
])
    # 🔵 DRAW TRAILS
    for robot in robots:
        for (tx, ty) in robot.trail:
            if visual[ty][tx] == 0:
                visual[ty][tx] = 6
    plt.imshow(visual, cmap=cmap)

    # 📊 legend (FIXED INSIDE FUNCTION)
    import matplotlib.patches as mpatches
    legend = [
        mpatches.Patch(color='black', label='Obstacle'),
        mpatches.Patch(color='orange', label='Parcel'),
        mpatches.Patch(color='blue', label='Robot'),
        mpatches.Patch(color='green', label='Charging'),
        mpatches.Patch(color='purple', label='Sorting'),
        mpatches.Patch(color='lightblue', label='Trail')  # ✅ FIX
    ]

    plt.legend(handles=legend, loc='upper right')

    plt.title(f"Warehouse Simulation - Step {step}")
    plt.pause(0.2)
    plt.clf()
def simulate_multi_robot():

    robots_data = load_robots()
    parcels = load_parcels()

    robots = []
    for r in robots_data:
        robots.append(Robot(int(r['x_position']), int(r['y_position'])))

    pick_points = [(int(p['x_position']), int(p['y_position'])) for p in parcels]

    charging_station = None
    layout = load_layout()
    for cell in layout:
        if cell['type'] == 'charging_station':
            charging_station = (int(cell['cell_x']), int(cell['cell_y']))

    print("\nSMART MULTI ROBOT SIMULATION\n")

    for step in range(30):

        place_objects()

        robots.sort(key=lambda r: (r.battery, -r.tasks_completed))

        for robot in robots:

            # assign target
            distance_to_charge = abs(robot.x - charging_station[0]) + abs(robot.y - charging_station[1])

            if robot.battery <= distance_to_charge + 5:
              if robot.target != charging_station:
                robot.target = charging_station
                robot.path = []
            if robot.target is None:
                if not robot.carrying:
                    target = get_nearest_pick(robot, pick_points)

                    if target and target not in assigned_targets:
                       robot.target = target
                       assigned_targets.add(target)
                else:
                    robot.target = charging_station
                robot.path = []

            if robot.target and robot.target in picked:
               robot.target = None
               robot.path = []
            # generate path
            if robot.target:
             if not robot.path:
               robot.path = astar((robot.x, robot.y), robot.target)

             if robot.path:
               next_x, next_y = robot.path[0]

               # check occupied positions
               occupied = {(r.x, r.y) for r in robots if r != robot}

               if is_free(next_x, next_y) and (next_x, next_y) not in occupied:
                robot.path.pop(0)
                robot.x, robot.y = next_x, next_y
                robot.trail.append((robot.x, robot.y))

                # keep last 20 positions only
                if len(robot.trail) > 20:
                    robot.trail.pop(0)
                robot.distance += 1
                robot.battery -= 1
               else:
                 import random
    
                 # 30% chance to WAIT (avoid deadlock)
                 if random.random() < 0.3:
                   continue  # robot skips this turn
    
                 # otherwise recalculate path
                 robot.path = astar((robot.x, robot.y), robot.target)


            if robot.target and (robot.x, robot.y) == robot.target:

               if not robot.carrying:
                  if robot.target not in picked:
                     print("Picked item at", robot.target)
                     picked.add(robot.target)
                     assigned_targets.discard(robot.target)
                     robot.carrying = True

                     if robot.target in pick_points:
                         pick_points.remove(robot.target)

               else:
                 print("Delivered to charging station")
                 robot.carrying = False
    
                 # recharge ONLY here
                 robot.battery = robot.max_battery
                 print("Robot recharged")

               robot.target = None
               robot.path = []
               robot.steps += 1
               
               
               robot.tasks_completed += 1
               
            grid[robot.y][robot.x] = 'R'
        
        draw_grid(step, robots)
        time.sleep(0.05)
    print("\nFINAL STATS:\n")
    for i, robot in enumerate(robots):
      print(f"Robot {i+1}:")
      print("Steps:", robot.steps)
      print("Distance:", robot.distance)
      print("Tasks:", robot.tasks_completed)

    
    total_distance = sum(r.distance for r in robots)
    total_tasks = sum(r.tasks_completed for r in robots)

    print("Total Distance:", total_distance)
    print("Total Tasks:", total_tasks)
    efficiency = total_tasks / total_distance if total_distance > 0 else 0
    print("Efficiency:", efficiency)

    total_battery_used = sum(r.max_battery - r.battery for r in robots)
    print("Battery Used:", total_battery_used)

    avg_tasks = total_tasks / len(robots)
    print("Avg Tasks per Robot:", avg_tasks)


# ---------------- RUN ---------------- #

if __name__ == "__main__":

    place_objects()
    print("Initial Warehouse:\n")
    print_grid()

    plt.ion()   # start animation

    simulate_multi_robot()

    plt.ioff()  # stop animation
    plt.show()  # keep final window open
def simulate_multi_robot_streamlit():
    frames = []

    robots_data = load_robots()
    parcels = load_parcels()

    robots = []
    for r in robots_data:
        robots.append(Robot(int(r['x_position']), int(r['y_position'])))

    pick_points = [(int(p['x_position']), int(p['y_position'])) for p in parcels]

    charging_station = None
    layout = load_layout()
    for cell in layout:
        if cell['type'] == 'charging_station':
            charging_station = (int(cell['cell_x']), int(cell['cell_y']))

    for step in range(30):

        place_objects()

        robots.sort(key=lambda r: (r.battery, -r.tasks_completed))

        for robot in robots:

            distance_to_charge = abs(robot.x - charging_station[0]) + abs(robot.y - charging_station[1])

            if robot.battery <= distance_to_charge + 5:
                robot.target = charging_station
                robot.path = []

            if robot.target is None:
                if not robot.carrying:
                    target = get_nearest_pick(robot, pick_points)
                    if target:
                        robot.target = target
                        assigned_targets.add(target)
                else:
                    robot.target = charging_station
                robot.path = []

            if robot.target and robot.target in picked:
                robot.target = None
                robot.path = []

            if robot.target:
                if not robot.path:
                    robot.path = astar((robot.x, robot.y), robot.target)

                if robot.path:
                    next_x, next_y = robot.path[0]

                    occupied = {(r.x, r.y) for r in robots if r != robot}

                    if is_free(next_x, next_y) and (next_x, next_y) not in occupied:
                        robot.path.pop(0)
                        robot.x, robot.y = next_x, next_y
                        robot.trail.append((robot.x, robot.y))

                        if len(robot.trail) > 20:
                            robot.trail.pop(0)

                        robot.distance += 1
                        robot.battery -= 1

            if robot.target and (robot.x, robot.y) == robot.target:

                if not robot.carrying:
                    picked.add(robot.target)
                    robot.carrying = True
                    if robot.target in pick_points:
                        pick_points.remove(robot.target)
                else:
                    robot.carrying = False
                    robot.battery = robot.max_battery

                robot.target = None
                robot.path = []
                robot.tasks_completed += 1

        fig = draw_grid_streamlit(step, robots)
        frames.append(fig)

    return frames
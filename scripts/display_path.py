import argparse
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.widgets import Slider
from parse import parse
import numpy as np

parser = argparse.ArgumentParser(description='Display a boundary and path planning.')

parser.add_argument("boundary", help="path to the boundary text file", type=str)
parser.add_argument("path_planning", help="path to the path planning text file", type=str)
args = parser.parse_args()

fig, ax = plt.subplots()

plt.subplots_adjust(bottom=0.25)

MAX_ARRAY_SIZE = 10000

def extractPath(file, closed=False, progression=100):
    points = [0] * MAX_ARRAY_SIZE
    codes = [0] * MAX_ARRAY_SIZE

    with open(file) as file_handler:
        lines = file_handler.readlines()
        n_lines = len(lines)

        if (n_lines == 0):
            raise Exception(file + " file is empty")

        idx = 0

        for line in lines:
            [x, y] = parse('({}, {})', line.strip())
            points[idx] = (x, y)

            if (idx == 0):
                codes[idx] = Path.MOVETO
            else:
                codes[idx] = Path.LINETO
            idx += 1

    if (closed):
        codes[idx] = Path.LINETO
        points[idx] = points[0]
        idx += 1

        codes[idx] = Path.CLOSEPOLY
        points[idx] = points[0]
        idx += 1

    last_idx = idx - 1

    # Reduce unused array positions
    points = points[0: last_idx]
    codes = codes[0: last_idx]

    last_idx_with_prog = max(1, int(last_idx*progression/100))

    return Path(points[0:last_idx_with_prog], codes[0:last_idx_with_prog])

# Draw boundary
boundary_path = extractPath(args.boundary, True)
boundary_patch = patches.PathPatch(boundary_path, facecolor='none', edgecolor='red', linewidth=1.5)
ax.add_patch(boundary_patch)

# Draw path planning
planner_path = extractPath(args.path_planning, False)
planner_patch = patches.PathPatch(planner_path, facecolor='none')
ax.add_patch(planner_patch)

# The function to be called anytime a slider's value changes
def update(val):
    if (len(ax.patches) > 0):
        ax.patches.pop()

    # Draw path planning
    planner_path = extractPath(args.path_planning, False, prog_slider.val)
    planner_patch = patches.PathPatch(planner_path, facecolor='none')
    ax.add_patch(planner_patch)

# Make a horizontal slider to control the progression.
axprog = plt.axes([0.25, 0.1, 0.65, 0.03])
prog_slider = Slider(
    ax=axprog,
    label='Progression [%]',
    valmin=1,
    valmax=100,
    valinit=100,
)

prog_slider.on_changed(update)

ax.set_title('Map boundary (red) and path planning (black)')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.margins(0.2, 0.2)
ax.set_aspect('equal', 'box')
plt.show()
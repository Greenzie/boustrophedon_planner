import argparse
import tkinter as tk

parser = argparse.ArgumentParser(description='Graphical interface to create a map boundary.')
parser.add_argument("file", help="path to the boundary text file", type=str)
args = parser.parse_args()

WINDOW_SIZE = 500
ORIGIN_X = WINDOW_SIZE / 2
ORIGIN_Y = WINDOW_SIZE / 2
POINT_RADIUS = 3
PIXEL_TO_M_RATIO = 0.1

root = tk.Tk()
canvas = tk.Canvas(root, width=WINDOW_SIZE, height=WINDOW_SIZE)

def convert_pixels_to_distance(x_pixel, y_pixel):
    x_m = (x_pixel - ORIGIN_X) * PIXEL_TO_M_RATIO
    y_m = (WINDOW_SIZE - y_pixel - ORIGIN_Y) * PIXEL_TO_M_RATIO
    return [x_m, y_m]

def callback(event):
    [x_m, y_m] = convert_pixels_to_distance(event.x, event.y)
    print ("Clicked at x={:.2f}, y={:.2f}".format(x_m, y_m))
    create_circle(event.x, event.y, POINT_RADIUS, canvas)

    with open(args.file, 'a') as file_handler:
        line = "({:.2f}, {:.2f})".format(x_m, y_m)
        file_handler.write(line + '\n')

def create_circle(x, y, r, canvasName, color="black"): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvasName.create_oval(x0, y0, x1, y1, fill=color)

try:
    open(args.file, 'w').close()
except IOError:
    raise Exception('Unable to open file ' + args.file)

# Display origin position in red
create_circle(ORIGIN_X, ORIGIN_Y, POINT_RADIUS, canvas, color="red")

canvas.bind("<Button-1>", callback)
canvas.pack()
root.mainloop()


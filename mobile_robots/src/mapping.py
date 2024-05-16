import matplotlib.pyplot as plt

grid_size = 20
cell_size = 0.1

clicked_points = []

def onclick(event):
    x = event.xdata
    y = event.ydata
    
    if x is not None and y is not None and 0 <= x < grid_size and 0 <= y < grid_size:
        x = round(x, 1)
        y = round(y, 1)
        
        clicked_points.append((x, y))
        
        print(f"Clicked point: ({x}, {y})")
        
        ax.plot(x, y, 'ro')
        plt.draw()

fig, ax = plt.subplots()

ax.set_aspect('equal')

for y in range(0, grid_size + 1, 2):
    ax.plot([0, grid_size], [y, y], color='black', linewidth=2)

for x in range(0, grid_size + 1, 2):
    ax.plot([x, x], [0, grid_size], color='black', linewidth=2)

ax.set_xlim(0, grid_size)
ax.set_ylim(0, grid_size)

ax.axis('off')

cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()

print('[', end=' ')
for i in range(len(clicked_points)):
    if i != 0:
        print(',', end=' ')
    x, y = clicked_points[i]
    x = round(x * 2) / 2  
    y = round(y * 2) / 2  
    x /= 5  
    y /= 5  
    print(f'[{x}, {y}]', end=' ')
print(']')


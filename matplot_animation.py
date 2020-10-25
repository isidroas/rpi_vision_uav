import csv 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

x = []
y = []
with open('build/log.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for i,row in enumerate(plots):
        if i!=0:
            x.append(float(row[-1]))
            y.append(float(row[0]))

DT=0.0033


x=np.array(x)
y=np.array(y)

def draw_animation(x, y):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots()
    xdata, ydata = [], []
    ln, = plt.plot([], [], "r")

    def init():
        margin = 2
        ax.set_xlim(min(x) - margin, max(x) + margin)
        ax.set_ylim(np.nanmin(y) - margin, np.nanmax(y) + margin)
        return (ln,)

    def update(frame):
        xdata.append(x[frame])
        ydata.append(y[frame])
        ln.set_data(xdata, ydata)
        #c = np.cos(theta[frame])
        #s = np.sin(theta[frame])
        #rot_mat = np.array([[c, -s], [s, c]])
        #p1 = [-0.5, 0]
        #p2 = [0.5, 0]
        #p1_rot = rot_mat @ p1
        #p2_rot = rot_mat @ p2
        #ln2.set_data(
        #    [p1_rot[0], p2_rot[0]] + x[frame], [p1_rot[1], p2_rot[1]] + y[frame]
        #)
        return ln, 

    ani = FuncAnimation(
        fig,
        update,
        frames=len(x),
        init_func=init,
        blit=True,
        interval=DT * 1e3,
        repeat=False,
    )
    plt.show()

draw_animation(x, y)

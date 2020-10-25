import csv 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

px = []
py = []
pz = []
t = []

with open('build/log.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for i,row in enumerate(plots):
        if i!=0:
            px.append(float(row[0]))
            py.append(float(row[1]))
            pz.append(float(row[2]))
            t.append(float(row[-1]))


DT=0.025

t=np.linspace(0,len(px)*0.025,len(px))

def draw_animation(x, y1, y2, y3, label1, label2, label3):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots()
    xdata, y1data, y2data, y3data = [], [], [], []
    ln1, = plt.plot([], [], "tab:red", label= label1)
    ln2, = plt.plot([], [], "tab:green", label=label2)
    ln3, = plt.plot([], [], "tab:blue", label=label3)

    def init():
        margin = 1
        ax.set_xlim(min(x) - margin, max(x) + margin)
        total_min=min(np.nanmin(y1), np.nanmin(y2), np.nanmin(y3))
        total_max=max(np.nanmax(y1), np.nanmax(y2), np.nanmax(y3))
        ax.set_ylim(total_min - margin, total_max + margin)
        ax.legend()
        return (ln1,ln2,ln3)

    def update(frame):
        xdata.append(x[frame])

        y1data.append(y1[frame])
        ln1.set_data(xdata, y1data)

        y2data.append(y2[frame])
        ln2.set_data(xdata, y2data)

        y3data.append(y3[frame])
        ln3.set_data(xdata, y3data)
        
        return ln1,ln2,ln3

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

draw_animation(t, px, py, pz, '$P_x$', '$P_y$', '$P_z$')

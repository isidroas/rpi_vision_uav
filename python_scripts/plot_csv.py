#!/usr/bin/python3
# -*- coding: utf-8 -*-
import csv 
import matplotlib.pyplot as plt
import sys

log_csv='../results/single_marker/log.csv'
log_csv=sys.argv[1]

px = []
py = []
pz = []
roll= []
pitch= []
yaw= []
t = []
with open(log_csv ,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for i,row in enumerate(plots):
        if i!=0:
            px.append(float(row[0]))
            py.append(float(row[1]))
            pz.append(float(row[2]))
            roll.append(float(row[3]))
            pitch.append(float(row[4]))
            yaw.append(float(row[5]))
            t.append(float(row[6]))


fig, ax = plt.subplots()
ax.set_title("Position")
ax.plot(px, label="px")
ax.plot(py, label="py")
ax.plot(pz, label="pz")
#plt.xlabel("$P_x$ (m)")
plt.ylabel("$P$ (m)")
ax.legend()

fig, ax = plt.subplots()
ax.set_title("Orientation")
ax.plot(roll, label="roll")
ax.plot(pitch, label="pitch")
ax.plot(yaw, label="yaw")
#plt.xlabel("$P_x$ (m)")
plt.ylabel("Angle (rad)")
ax.legend()

plt.show()

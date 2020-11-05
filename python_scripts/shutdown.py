from datetime import datetime
import os
import shutil
import yaml

import csv 
import numpy as np
import matplotlib.pyplot as plt

IMAGE_EXTENSION=".png"

print("Este es el script de apagado")

# read parameters
params=None
with open('../vision_params.yml') as file:
    # remove first line since it contains invalid syntax
    file.readline()
    params = yaml.load(file.read(), Loader=yaml.FullLoader)

results_path='../results/latest/'

if (not params["write_images"] or not params["log_file"]):
    quit(0)

if (params["log_file"]):
    px = []
    py = []
    pz = []
    roll= []
    pitch= []
    yaw= []
    t = []
    with open(results_path+'log.csv' ,'r') as csvfile:
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
    plt.savefig(results_path + "position" + IMAGE_EXTENSION)
    
    fig, ax = plt.subplots()
    ax.set_title("Orientation")
    ax.plot(roll, label="roll")
    ax.plot(pitch, label="pitch")
    ax.plot(yaw, label="yaw")
    #plt.xlabel("$P_x$ (m)")
    plt.ylabel("Angle (rad)")
    ax.legend()
    plt.savefig(results_path + "eul" + IMAGE_EXTENSION)

shutil.copy('../detector_params.yml',results_path)
shutil.copy('../vision_params.yml',results_path)

if params["generate_video_from_images"]:
    # TODO: adjust fps
    write_images=params["write_images"]
    if write_images:
        os.system(" ffmpeg -r 40 -f image2 -s 640x480 -i ../results/latest/images/image%d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p ../results/latest/ar_result.mp4")
    


timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
os.rename('../results/latest','../results/'+timestamp)


quit(0)

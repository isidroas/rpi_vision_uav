from datetime import datetime
import os
from time import sleep
import yaml
from pdb import set_trace

OUTPUT_FORMAT = "h264"

params=None

# read parameters
with open('vision_params.yml') as file:
    # remove first line since it contains invalid syntax
    file.readline()
    params = yaml.load(file.read(), Loader=yaml.FullLoader)

FPS=params["fps"]
WIDTH=params["frame_width"]
HEIGHT=params["frame_height"]
EXPOSURE=params["exposure_time"]

## Configure camera
# exposure time
if EXPOSURE==0:
    cmd = "v4l2-ctl -d /dev/video0 -c auto_exposure=0"
else:
    cmd = f"v4l2-ctl -c exposure_time_absolute={EXPOSURE} -c auto_exposure=1"
os.system(cmd)

# fps
if FPS!=0:
    cmd = "v4l2-ctl -d /dev/video0 -p {FPS}"

# Set pixelformat = 'BGR3' since opencv make this change
cmd=f"v4l2-ctl --set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=10"
os.system(cmd)

os.system("v4l2-ctl -V")

# Record Video
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_file="videos/"+ timestamp+"." + OUTPUT_FORMAT
# h264_omx es el acelerador hardware de la rpi
#cmd = f"ffmpeg -f v4l2 -video_size {WIDTH}x{HEIGHT} -i /dev/video0 -c:v h264_omx "

# Elimino el acelerador ya que me crea archivos en negro
cmd = f"ffmpeg -f v4l2 -video_size {WIDTH}x{HEIGHT} -i /dev/video0 "
# El siguiente parámetro es para que se parezca a opencv, pero no se puede reproducir luego
#cmd = cmd + "-input_format bgr24 " 
if FPS!=0:
    cmd = cmd + f"-framerate {FPS} "
cmd = cmd + output_file
print(cmd)
os.system(cmd)

# Wait to print in screen since ffmpeg continue printing
sleep(2)

# Verifica los parámetros de la cámara
os.system("v4l2-ctl -V")

# TODO: convert to mp4
print("Written in " + output_file)

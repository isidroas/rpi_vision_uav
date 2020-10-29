from datetime import datetime
import os
from time import sleep

# Configure camera
cmd = "v4l2-ctl -c exposure_time_absolute=50 -c auto_exposure=1"
os.system(cmd)


# Record Video
print(datetime.now())
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_file="videos/"+ timestamp+".h264"

cmd = "ffmpeg -f v4l2 -framerate 30 -video_size 640x480 -i /dev/video0 " + output_file
os.system(cmd)

sleep(3)

# todo convert to mp4


print(timestamp)

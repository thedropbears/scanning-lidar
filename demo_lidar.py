from lidar import LIDARLite
import time

lidar = LIDARLite()

lidar.start()
time.sleep(5)
lidar.terminate()
lidar.join()

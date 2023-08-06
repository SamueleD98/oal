
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import sys

log = open("/home/samuele/graal_ws/oal/cmake-build-debug/WPlog.txt", "r")
#log = open("log.txt", "r")
Lines = log.readlines()
data = {}

#file = log.read()
for line in Lines:
    line = line.strip()
    #print(line)
    Values = line.split("_")
    #print(Values)
    key = Values[0]
    if key == "Start":
        startPos = (float(Values[1]), float(Values[2]))
    elif key == "Goal":
        goalPos = (float(Values[1]), float(Values[2]))
    elif key == "Time":
        timestamp = Values[1]
        data[timestamp] = []
    elif key == "Waypoint":
        data[timestamp].append({"wp": (float(Values[1]), float(Values[2]))})
    elif key == "Obs":
        id = Values[1]
    elif key == "Pose":
        pose = (float(Values[1]), float(Values[2]))
    elif key == "Heading":
        theta = float(Values[1])
    elif key == "Safety":
        safety = float(Values[1])
    elif key == "Max":
        max = float(Values[1])
    elif key == "Dimx":
        dimx = float(Values[1])
    elif key == "Dimy":
        dimy = float(Values[1])
    elif key == "-":
        data[timestamp].append({"obs": id, "pose": pose, "heading": theta, "safety": safety, "max": max, "dimx": dimx, "dimy": dimy})


bk = data
fig = plt.figure(figsize=(10, 5))

timeinstants = sorted(data.keys())
wps = []
count = 0
for timestamp in timeinstants:
  data = bk
  count = count + 1
  ax = fig.add_subplot(1, len(data.keys()), count)
  ax.set_title('Time: '+timestamp)
  # plt.xlim(7, 13)
  # plt.ylim(0, 5)

  wp = data[timestamp][0]
  data[timestamp].remove(wp)
  pol_data = data[timestamp]

  ax.scatter(startPos[0], startPos[1], color='black', marker='o', s=50)
  ax.text(startPos[0]-0.6, startPos[1], "Start", ha='center', va='center', fontsize=7)
  ax.scatter(goalPos[0], goalPos[1], color='green', marker='x', s=50)
  ax.text(goalPos[0]-0.6, goalPos[1], "Goal", ha='center', va='center', fontsize=7)

  wp_pos = wp["wp"]
  wps.append(wp_pos)
  trace_x = []
  trace_y = []
  for wp in wps:
      trace_x.append(wp[0])
      trace_y.append(wp[1])

  ax.plot(trace_x, trace_y, 'r.:')
  ax.scatter(wp_pos[0], wp_pos[1], color='red', marker='o')


  # Add polygons and their names to the plot
  for polygon_data in pol_data:
      #print(polygon_data)
      heading = float(polygon_data['heading'])
      dim_x = float(polygon_data['dimx'])
      dim_y = float(polygon_data['dimy'])
      safety = float(polygon_data['safety'])
      max = float(polygon_data['max'])
      pose = (float(polygon_data['pose'][0]),float(polygon_data['pose'][1]))
      sdim_x = dim_x*safety
      mdim_x = dim_x*max
      sdim_y = dim_y*safety
      mdim_y = dim_y*max

      vx1 = (pose[0]+dim_x/2*math.cos(heading)+dim_y/2*math.sin(heading),
             pose[1]-dim_y/2*math.cos(heading)+dim_x/2*math.sin(heading))
      vx2 = (pose[0]+dim_x/2*math.cos(heading)-dim_y/2*math.sin(heading),
             pose[1]+dim_y/2*math.cos(heading)+dim_x/2*math.sin(heading))
      vx3 = (pose[0]-dim_x/2*math.cos(heading)+dim_y/2*math.sin(heading),
             pose[1]-dim_y/2*math.cos(heading)-dim_x/2*math.sin(heading))
      vx4 = (pose[0]-dim_x/2*math.cos(heading)-dim_y/2*math.sin(heading),
             pose[1]+dim_y/2*math.cos(heading)-dim_x/2*math.sin(heading))
      verts = [vx1, vx2, vx3, vx4]

      vx1 = (pose[0]+sdim_x/2*math.cos(heading)+sdim_y/2*math.sin(heading),
             pose[1]-sdim_y/2*math.cos(heading)+sdim_x/2*math.sin(heading))
      vx2 = (pose[0]+sdim_x/2*math.cos(heading)-sdim_y/2*math.sin(heading),
             pose[1]+sdim_y/2*math.cos(heading)+sdim_x/2*math.sin(heading))
      vx3 = (pose[0]-sdim_x/2*math.cos(heading)+sdim_y/2*math.sin(heading),
             pose[1]-sdim_y/2*math.cos(heading)-sdim_x/2*math.sin(heading))
      vx4 = (pose[0]-sdim_x/2*math.cos(heading)-sdim_y/2*math.sin(heading),
             pose[1]+sdim_y/2*math.cos(heading)-sdim_x/2*math.sin(heading))
      sverts = [vx1, vx2, vx3, vx4]

      vx1 = (pose[0]+mdim_x/2*math.cos(heading)+mdim_y/2*math.sin(heading),
             pose[1]-mdim_y/2*math.cos(heading)+mdim_x/2*math.sin(heading))
      vx2 = (pose[0]+mdim_x/2*math.cos(heading)-mdim_y/2*math.sin(heading),
             pose[1]+mdim_y/2*math.cos(heading)+mdim_x/2*math.sin(heading))
      vx3 = (pose[0]-mdim_x/2*math.cos(heading)+mdim_y/2*math.sin(heading),
             pose[1]-mdim_y/2*math.cos(heading)-mdim_x/2*math.sin(heading))
      vx4 = (pose[0]-mdim_x/2*math.cos(heading)-mdim_y/2*math.sin(heading),
             pose[1]+mdim_y/2*math.cos(heading)-mdim_x/2*math.sin(heading))
      mverts = [vx1, vx2, vx3, vx4]


      #verts = polygon_data['vxs']
      center = np.mean(verts, axis=0)
      sorted_points = sorted(verts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
      poly = Polygon(sorted_points, facecolor='k', edgecolor='k')
      ax.add_patch(poly)
      centroid_x = sum(x for x, y in verts) / len(verts)
      centroid_y = sum(y for x, y in verts) / len(verts)

      #verts = polygon_data['vxs']
      center = np.mean(sverts, axis=0)
      sorted_points = sorted(sverts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
      poly = Polygon(sorted_points, facecolor='None', edgecolor='r')
      ax.add_patch(poly)
      #verts = polygon_data['vxs']
      center = np.mean(mverts, axis=0)
      sorted_points = sorted(mverts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
      poly = Polygon(sorted_points, facecolor='None', edgecolor='g')
      ax.add_patch(poly)


      plt.arrow(centroid_x, centroid_y, 0.3*math.cos(heading), 0.3*math.sin(heading), head_width=0.1, head_length=0.1, color='blue')

      ax.text(centroid_x-mdim_x/2-0.2, centroid_y, polygon_data['obs'], ha='center', va='center', fontsize=10)
      ax.axis('equal')
      #ax.axis('square')
      ax.set_xlim(8, 12)
      ax.set_aspect('equal')


plt.show()


sys.exit()

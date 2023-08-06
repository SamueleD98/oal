
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import sys
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import itertools




log = open("/home/samuele/graal_ws/oal/cmake-build-debug/CKlog.txt", "r")
#log = open("CKlog.txt", "r")
Lines = log.readlines()


plots = []
plot = []
for line in Lines:
  line = line.strip()
  Values = line.split("_")
  if Values[0] == "---":
    plots.append(plot)

    plot = []
  else:
    plot.append(line)


for plot in plots:
  if plot == []:
    continue
  #file = log.read()
  isToPlot = False

  data = {}
  for line in plot:
      #line = line.strip()
      #print(line)
      Values = line.split("_")
      #print(Values)
      key = Values[0]
      if key == "Start":
          startPos = (float(Values[1]), float(Values[2]), float(Values[3]))
      elif key == "Goal":
          goalPos = (float(Values[1]), float(Values[2]), float(Values[3]))
      elif key == "Obs":
          id = Values[1]
          data[id] = []
          vxs = []
          isToPlot = False
      elif key == "Direction":
          dir = (float(Values[1]), float(Values[2]), float(Values[3]))

      elif key == "Vx":
          vx = (float(Values[1]), float(Values[2]))
          vxs.append(vx)
      elif key == "-":
          data[id].append({"dir":dir, "vxs":vxs })
      elif key == "PlotIt":
          isToPlot = True

  if(not isToPlot):
      continue
  bk = data
  fig = plt.figure(figsize=(10, 5))

  #ax = fig.add_subplot(1, len(plots), 1)
  ax = fig.add_subplot(projection="3d")
  #ax.set_title('Time: '+timestamp)

  x, y, z = [startPos[0], goalPos[0]], [startPos[1], goalPos[1]], [startPos[2], goalPos[2]]
  ax.scatter(x, y, z, c='red')#, s=100)
  ax.plot(x, y, z, color='black')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Time')
  obs = data.keys()

  for ob in obs:
    obstacle = data[ob][0]
    dir = obstacle['dir']
    vxs = obstacle['vxs']

    vxs3d_start = []
    vxs3d_goal = []
    for vx in vxs:
      # start time
      vx_=[vx[0], vx[1], 0]
      vxs3d_start.append(vx_)
      # goal time
      vx_next=[vx[0]+dir[0]*goalPos[2] , vx[1]+dir[1]*goalPos[2], dir[2]*goalPos[2]]
      vxs3d_goal.append(vx_next)
    last = vxs3d_start[3]
    vxs3d_start[3] = vxs3d_start[2]
    vxs3d_start[2] = last
    last = vxs3d_goal[3]
    vxs3d_goal[3] = vxs3d_goal[2]
    vxs3d_goal[2] = last
    vertices = vxs3d_start + vxs3d_goal
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[3], vertices[7], vertices[4]],
        [vertices[4], vertices[5], vertices[6], vertices[7]]
    ]

    ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.6))

  plt.show()

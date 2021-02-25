#!/usr/bin/env python
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import PIL
from PIL import Image
import rospy
from std_msgs.msg import Float64MultiArray
from swarmie_msgs.msg import Waypoint
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point

# from progressbar import Progressbar

# parameter
N_SAMPLE = 100  # number of sample_points
N_KNN = 15  # number of edge from one sampled point
MAX_EDGE_LEN = 1000.0  # [m] Maximum edge length
ox=[]
oy=[]

show_animation = False


class Node:
  """
  Node class for dijkstra search
  """

  def __init__(self, x, y, cost, parent_index):
    self.x = x
    self.y = y
    self.cost = cost
    self.parent_index = parent_index

  def __str__(self):
    return str(self.x) + "," + str(self.y) + "," + \
           str(self.cost) + "," + str(self.parent_index)


def prm_planning(sx, sy, gx, gy, ox, oy, rr):
  print('generating obstacle tree')
  obstacle_kd_tree = cKDTree(np.vstack((ox, oy)).T)

  print('generating samples')
  sample_x, sample_y = sample_points(sx, sy, gx, gy,
                                     rr, ox, oy, obstacle_kd_tree)
  if show_animation:
    plt.plot(sample_x, sample_y, ".b")

  print('generating roadmap')
  road_map = generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree)

  print('running dijkstra')
  rx, ry = dijkstra_planning(
    sx, sy, gx, gy, road_map, sample_x, sample_y)

  return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
  x = sx
  y = sy
  dx = gx - sx
  dy = gy - sy
  yaw = math.atan2(gy - sy, gx - sx)
  d = math.hypot(dx, dy)

  if d >= MAX_EDGE_LEN:
    return True

  D = rr
  n_step = int(d / D)

  for i in range(n_step):
    dist, _ = obstacle_kd_tree.query([x, y])
    if dist <= rr:
      return True  # collision
    x += D * math.cos(yaw)
    y += D * math.sin(yaw)

  # goal point check
  dist, _ = obstacle_kd_tree.query([gx, gy])
  if dist <= rr:
    return True  # collision

  return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
  """
  Road map generation
  sample_x: [m] x positions of sampled points
  sample_y: [m] y positions of sampled points
  rr: Robot Radius[m]
  obstacle_kd_tree: KDTree object of obstacles
  """

  road_map = []
  n_sample = len(sample_x)
  sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)

  for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

    dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
    edge_id = []

    for ii in range(1, len(indexes)):
      nx = sample_x[indexes[ii]]
      ny = sample_y[indexes[ii]]

      if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
        edge_id.append(indexes[ii])

      if len(edge_id) >= N_KNN:
        break

    road_map.append(edge_id)

  #  plot_road_map(road_map, sample_x, sample_y)

  return road_map


def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
  """
  s_x: start x position [m]
  s_y: start y position [m]
  gx: goal x position [m]
  gy: goal y position [m]
  ox: x position list of Obstacles [m]
  oy: y position list of Obstacles [m]
  rr: robot radius [m]
  road_map: ??? [m]
  sample_x: ??? [m]
  sample_y: ??? [m]
  @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
  """

  start_node = Node(sx, sy, 0.0, -1)
  goal_node = Node(gx, gy, 0.0, -1)

  open_set, closed_set = dict(), dict()
  open_set[len(road_map) - 2] = start_node

  path_found = True

  while True:
    if not open_set:
      print("Cannot find path")
      path_found = False
      break

    c_id = min(open_set, key=lambda o: open_set[o].cost)
    current = open_set[c_id]

    # show graph
    if show_animation and len(closed_set.keys()) % 2 == 0:
      # for stopping simulation with the esc key.
      plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
      plt.plot(current.x, current.y, "xg", markersize=20)
      plt.pause(0.001)

    if c_id == (len(road_map) - 1):
      print("goal is found!")
      goal_node.parent_index = current.parent_index
      goal_node.cost = current.cost
      break

    # Remove the item from the open set
    del open_set[c_id]
    # Add it to the closed set
    closed_set[c_id] = current

    # expand search grid based on motion model
    for i in range(len(road_map[c_id])):
      n_id = road_map[c_id][i]
      dx = sample_x[n_id] - current.x
      dy = sample_y[n_id] - current.y
      d = math.hypot(dx, dy)
      node = Node(sample_x[n_id], sample_y[n_id],
                  current.cost + d, c_id)

      if n_id in closed_set:
        continue
      # Otherwise if it is already in the open set
      if n_id in open_set:
        if open_set[n_id].cost > node.cost:
          open_set[n_id].cost = node.cost
          open_set[n_id].parent_index = c_id
      else:
        open_set[n_id] = node

  if path_found is False:
    return [], []

  # generate final course
  rx, ry = [goal_node.x], [goal_node.y]
  parent_index = goal_node.parent_index
  while parent_index != -1:
    n = closed_set[parent_index]
    rx.append(n.x)
    ry.append(n.y)
    parent_index = n.parent_index

  return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

  for i, _ in enumerate(road_map):
    for ii in range(len(road_map[i])):
      ind = road_map[i][ii]

      # plt.plot([sample_x[i], sample_x[ind]],
      #        [sample_y[i], sample_y[ind]], "-k", markersize = 20)


def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree):
  max_x = max(ox)
  max_y = max(oy)
  min_x = min(ox)
  min_y = min(oy)

  sample_x, sample_y = [], []

  while len(sample_x) <= N_SAMPLE:
    tx = (random.random() * (max_x - min_x)) + min_x
    ty = (random.random() * (max_y - min_y)) + min_y

    dist, index = obstacle_kd_tree.query([tx, ty])

    if dist >= rr:
      sample_x.append(tx)
      sample_y.append(ty)

  sample_x.append(sx)
  sample_y.append(sy)
  sample_x.append(gx)
  sample_y.append(gy)

  return sample_x, sample_y


def robot_to_PRM(Rx, Ry):
  Px = 1500 + Rx * 100
  Py = abs(Ry * 100 - 1500)

  return Px, Py


def PRM_to_robot(Px, Py):
  Rx = (Px - 1500) / 100
  Ry = (1500-Py)/100

  return Rx, Ry

def generate_obstacle():
  image = PIL.Image.open(
    filter(lambda x: "navigation" in x, rospkg.get_ros_package_path().split(':'))[0] + '/src/occupancytest.jpg')
  g_image = image.convert("L")
  g_array = np.asarray(g_image)
  #print('generating obstacle map')
  for i in range(0, len(g_array[0])):
    for j in range(0, len(g_array[1])):
      if g_array[i][j] == 0:
        ox.append(float(j))
        oy.append(float(i))
   
  return ox,oy

	

def PlanPRM(Rxs, Rys, Rxg, Ryg):
  # print(__file__ + " start!!")

  # start and goal position
  print('generated obstacle map')
  sx, sy = robot_to_PRM(Rxs, Rys)

  gx, gy = robot_to_PRM(Rxg, Ryg)
  robot_size = 5  # [m]
  # plt.plot(ox, oy, ".k")
  # if show_animation:
  #   plt.plot(ox, oy, ".k")
  #  plt.plot(sx, sy, "^r", markersize = 10)
  # plt.plot(gx, gy, "^c", markersize = 10)
  # plt.grid(True)
  # plt.axis("equal")

  print('running prm map')
  rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size)

  for i in reversed(range(0, len(rx))):
    # print('PRM:'+ str(rx[i])+','+str(ry[i]))
    Rxd, Ryd = PRM_to_robot(rx[i], ry[i])
    print('Robot:'+ str(Rxd)+','+str(Ryd))
    pub.publish( PoseStamped(pose=Pose(position=Point(Rxd,Ryd,0))))

  assert rx, 'Cannot found path'

  # if show_animation:
  #   plt.plot(rx, ry, "-r")
  #  plt.pause(0.001)
  # plt.show()


def PRMRequestCallback(msg):
  PlanPRM(msg.data[0], msg.data[1], msg.data[2], msg.data[3])


if __name__ == '__main__':
  rospy.init_node("PRM")
  ox,oy = generate_obstacle()
  sub = rospy.Subscriber('/PRM', Float64MultiArray, PRMRequestCallback)
  pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

  rospy.spin()

# COMS W4733 Robotics Lab 2

## Usage
#### 1. Extract compressed 'vgraph' to '~/catkin_ws/src' <ins>OR</ins> run the following two snippets:

> As in Lab 3 repo:
```shell
cd ~/catkin_ws/src
git clone https://github.com/jingxixu/vgraph.git
cd ../
catkin_make
source devel/setup.bash
rospack profile
```
Then, copy needed files:
```shell
$ cp /PATH/TO/{draw_graph.py,move_robot.py,utils.py} ~/catkin_ws/src/vgraph/src
```

#### 2. Bring up Rviz:
```shell
$ roslaunch vgraph launch.launch
```

#### 3. Generate markers:
```shell
$ cd ~/catkin_ws/src/vgraph/src
$ python draw_graph.py
```

#### 4. Finally, move the robot to the goal
```shell
$ python move_robot.py
```

## Methods

move_robot.py:
------
```python
def get_goal_direction(self):
  """
  Make the robot rotate towards the goal point
  and decide the action to adjust the current yaw accordingly
  """
```
```python
def get_distance_between(self, p1, p2):
  """ Calculate the distance between the two points, p1 and p2 """
```
utils.py:
------
```python
def get_waypoints():
	"""
	This function is responsible for part 2, 3, 4
	Return a list of all edges and the list of vertices of the shortest path.
	"""
```
```python
def get_grown_vts(ob, rob, ref):
	"""
	--- Part 2 ---
	Grow the obstacles using reflection algorithm
	(flip the robot around the reference point,
	place it at each vertex of the obstacle, and create a convex hull).
	Assuming the robot to be a 36cm-by-36cm square.
	Reference point is (18, 18)
	"""
```
```python
def connect_vts(obs, start, goal):
	"""
	Create the vgraph by first fully connecting:
	all obstacle vertices + start + goal
	without collision
	"""
```
```python
def check_collision(A, B, hull):
	"""
	Collision checker, for removing edges that collides with obstacles (except endpoints).
	"""
```
```python
def ray_side(A, B, X, Y):
	"""
	Check whether a ray from A to B intersects with the line segment connecting
	X an Y
	Return t where t * AB + A is the point of intersection
	"""
```
```python
def get_shortest_path(graph, start, goal):
	"""
	Return the shortest path from start to goal through the graph
	"""
```
```python
def dijkstra(graph, source):
	"""
	The Dijkstra algorithm
	"""
```
```python
def get_graph(edges):
	"""
	Extract a dict that represents the graph from the list of edges of the graph
	"""
```
```python
def get_neighbors(graph, vt):
	"""
	Get the neighbors of a vertex
	"""
```
```python
def vt2tuple(vt):
	"""
	Convert vt to a tuple
	"""
```
```python
def is_same_vertex(u, v):
	"""
	Check if u and v is the same vertex
	"""
```

## Video
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/cTsVg1yvfSM/0.jpg)](http://www.youtube.com/watch?v=cTsVg1yvfSM)

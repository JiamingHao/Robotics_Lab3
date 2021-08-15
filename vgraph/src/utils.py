import numpy as np
from scipy.spatial import ConvexHull
import create_map

def get_waypoints():
	"""
	This function is responsible for part 2, 3, 4
	Return a list of all edges and the list of vertices of the shortest path.
	"""
	obstacles = create_map.load_obstacles("../data/world_obstacles.txt")
	goal = create_map.load_goal("../data/goal.txt")
	start = [0, 0]
	goal = tuple(create_map.map2img([goal])[0])
	start = tuple(create_map.map2img([start])[0])
	robot = np.array([[0, 0],
					[36, 0],
					[0, 36],
					[36, 36]])
	ref_pt = np.array([18, 18])

	# ======================= part 2. =======================
	grown_obs = []
	for ob in obstacles:
		grown_vts = get_grown_vts(create_map.map2img(ob), robot, ref_pt)
		grown_obs.append(grown_vts)

	# ======================= part 3. =======================
	edges = connect_vts(grown_obs, start, goal)

	# ======================= part 4. =======================
	graph = get_graph(edges)
	shortest_path = get_shortest_path(graph, start, goal)

	return edges, shortest_path

# ======================= part 2. =======================
def get_grown_vts(ob, rob, ref):
	"""
	--- Part 2 ---
	Grow the obstacles using reflection algorithm
	(flip the robot around the reference point,
	place it at each vertex of the obstacle, and create a convex hull).
	Assuming the robot to be a 36cm-by-36cm square.
	Reference point is (18, 18)
	"""
	flipped_rob = -(rob - np.stack([ref] * rob.shape[0]))
	grown_vts = pts = np.zeros((rob.shape[0]*ob.shape[0], 2))
	for idx, vtx in enumerate(ob):
		grown_vts[idx * rob.shape[0]:(idx+1)*rob.shape[0]] = \
			flipped_rob + np.stack([vtx] * rob.shape[0])
	# Get Convex-Hull scipy.spatial.ConvexHull
	grown_vts = grown_vts[ConvexHull(grown_vts).vertices].astype(np.int32)
	return grown_vts

# ======================= part 3. =======================
def connect_vts(obs, start, goal):
	"""
	Create the vgraph by first fully connecting:
	all obstacle vertices + start + goal
	without collision
	"""
	vts = np.concatenate(obs)
	vts = np.concatenate((vts, [start], [goal]), axis = 0)
	edges = []
	for idx in range(len(vts)):
		for idx_next in range(idx + 1, vts.shape[0]):
			vt = vts[idx]
			vt_next = vts[idx_next]
			has_collision = False
			for ob in obs:
				has_collision |= check_collision(vt, vt_next, ob)
			if not has_collision:
				edges.append(np.stack([vt, vt_next], axis = 0).tolist())
	for ob in obs:
		for idx in range(ob.shape[0]):
			vt = ob[idx]
			vt_next = ob[(idx + 1) % ob.shape[0]]
			edges.append(np.stack([vt, vt_next], axis = 0).tolist())

	return edges

def check_collision(A, B, hull):
	"""
	Collision checker, for removing edges that collides with obstacles (except endpoints).
	"""
	tolerance = 1e-5
	t_AB = float('inf')
	t_BA = float('inf')
	for idx in range(hull.shape[0]):
		X = hull[idx]
		Y = hull[(idx + 1) % hull.shape[0]]
		cur_t_AB = ray_side(A, B, X, Y)
		cur_t_BA = ray_side(B, A, X, Y)
		if cur_t_AB is not None and \
				tolerance < cur_t_AB and cur_t_AB < 1 - tolerance:
			# Vector AB intersects with one of the sides
			return True
		if cur_t_AB is not None and 1 <= cur_t_AB and cur_t_AB < t_AB:
			t_AB = cur_t_AB
		if cur_t_BA is not None and 1 <= cur_t_BA and cur_t_BA < t_BA:
			t_BA = cur_t_BA

	if t_AB >= 1 and t_BA >= 1 and t_AB != float('inf') and t_BA != float('inf'):
		# Vector AB is inside the hull
		return True

	return False

def ray_side(A, B, X, Y):
	"""
	Check whether a ray from A to B intersects with the line segment connecting
	X an Y
	Return t where t * AB + A is the point of intersection
	"""
	AB = B - A
	direction = Y - X
	direction = direction / np.linalg.norm(direction)
	normal = np.array([direction[1], -direction[0]])
	d = np.dot(normal, X)
	denom = np.dot(normal, AB)
	if denom != 0:
		dist = d - np.dot(normal, A)
		t = dist / denom
		P = t * AB + A # The point of intersection
		if np.min([X[0], Y[0]]) <= P[0] and \
		   np.max([X[0], Y[0]]) >= P[0] and \
		   np.min([X[1], Y[1]]) <= P[1] and \
		   np.max([X[1], Y[1]]) >= P[1]:
		   return t
		else:
			return None
	else:
		return None

# ======================= part 4. =======================
def get_shortest_path(graph, start, goal):
	"""
	Return the shortest path from start to goal through the graph
	"""
	source = vt2tuple(start)
	dist_prev = dijkstra(graph, source)

	S = []
	u = vt2tuple(goal)
	if dist_prev[u][1] is not None or is_same_vertex(u, source):
		while u is not None:
			S.insert(0, u)
			u = dist_prev[u][1]

	return S

def dijkstra(graph, source):
	"""
	The Dijkstra algorithm
	"""
	Q = []
	dist_prev = {}

	for v in graph.keys():
		# Tuple for each vertex: (dist, prev)
		dist_prev[v] = [float('inf'), None]
		Q.append(v)
	dist_prev[source][0] = 0

	while len(Q) > 0:
		# Get vertex in Q with min dist[u]
		min_dist = float('inf')
		u = None
		for vt in Q:
			if dist_prev[vt][0] < min_dist:
				min_dist = dist_prev[vt][0]
				u = vt

		Q.remove(u)

		for v in get_neighbors(graph, u):
			alt = dist_prev[u][0] + np.linalg.norm(np.array(u) - np.array(v))
			if alt < dist_prev[v][0]:
				dist_prev[v] = [alt, u]

	return dist_prev

def get_graph(edges):
	"""
	Extract a dict that represents the graph from the list of edges of the graph
	"""
	graph = {}
	for edge in edges:
		v1 = vt2tuple(edge[0])
		v2 = vt2tuple(edge[1])
		if v1 in graph:
			graph[v1].append(v2)
		else:
			graph[v1] = [v2]

		if v2 in graph:
			graph[v2].append(v1)
		else:
			graph[v2] = [v1]
	return graph

def get_neighbors(graph, vt):
	"""
	Get the neighbors of a vertex
	"""
	return graph[vt2tuple(vt)]

def vt2tuple(vt):
	"""
	Convert vt to a tuple
	"""
	return (vt[0], vt[1])

def is_same_vertex(u, v):
	"""
	Check if u and v is the same vertex
	"""
	return u[0] == v[0] and u[1] == v[1]

#
# if __name__ == "__main__":
# 	edges, shortest_path = get_waypoints()
# 	map_edges = []
# 	for img_edge in edges:
# 		map_edge = create_map.img2map(img_edge)
# 		map_edges.append(map_edge.tolist())
# 	print(map_edges)
# 	map_shortest_path = create_map.img2map(shortest_path).tolist()

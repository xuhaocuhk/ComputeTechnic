'''
=================
3D wireframe plot
=================

A very basic demonstration of a wireframe plot.
'''

import matplotlib.pyplot as plt
import numpy as np
import sys

MAX_INT = 64564351345
MAX_FLOAT = 456464654.1454

MAX_VERTEX_ID = 0
MAX_EDGE_ID = 0
edge_num = 0
holes = set()

vertexes = np.full((100000, 3), MAX_FLOAT)
edges = np.full((100000, 2), MAX_INT)

for line in sys.stdin:
    chars = line.strip().split(' ')
    #print(chars)
    if line.strip().split(' ')[0] == 'v':
        if int(chars[1]) > MAX_VERTEX_ID:
            MAX_VERTEX_ID = int(chars[1])
        vertexes[int(chars[1])][0] = float(chars[2])
        vertexes[int(chars[1])][1] = float(chars[3])
        vertexes[int(chars[1])][2] = float(chars[4])
    elif line.strip().split(' ')[0] == 'l':
        if int(chars[1])>MAX_EDGE_ID:
            MAX_EDGE_ID = int(chars[1])
        edges[int(chars[1])][0] = int(chars[2])
        edges[int(chars[1])][1] = int(chars[3])
        holes.add(int(chars[2]))
        holes.add(int(chars[3]))
        edge_num += 1

print ("max vertex id:", MAX_VERTEX_ID)
print ("max edge id:", MAX_EDGE_ID)
print ("edge num:", edge_num)
print ("hole num:", len(holes))
print ("holes:", holes)



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot a basic wireframe.
X = vertexes[:,0][0:MAX_VERTEX_ID+1]
Y = vertexes[:,1][0:MAX_VERTEX_ID+1]
Z = vertexes[:,2][0:MAX_VERTEX_ID+1]

ax.plot(X.tolist(),Y.tolist(),Z.tolist(),'ro')

for index in range(MAX_EDGE_ID+1):
	# Not all edges are output
	if edges[index][0] == MAX_INT:
		continue
	x1 = vertexes[edges[index][0]][0]
	x2 = vertexes[edges[index][1]][0]
	y1 = vertexes[edges[index][0]][1]
	y2 = vertexes[edges[index][1]][1]
	z1 = vertexes[edges[index][0]][2]
	z2 = vertexes[edges[index][1]][2]
	_x = [x1, x2]
	_y = [y1, y2]
	_z = [z1, z2]
	label = str(index)
	ax.text( (x1+x2)/2, (y1+y2)/2, (z1+z2)/2, label, None)
	ax.plot(_x,_y,_z)

plt.show()



















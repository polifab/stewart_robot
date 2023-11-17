import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import mpl_toolkits.mplot3d.art3d as art3d

lista = np.array([[-1.21,1.57], [-1.21,-1.57], [1.97,-0.26], [-0.759, -1.83], [-0.759, 1.83], [-0.95772, 0.14656], [0.351935, 0.902], [1.97, 0.26], [-0.957722, -0.14656], [0.351935, -0.90269], [0.605787, -0.756132], [0.605788, 0.75613]])
lista_3d = np.array([[-1.21,1.57, 0.121], [-1.21,-1.57, 0.121], [1.97,-0.26, 0.121], [-0.759, -1.83, 0.121], [-0.759, 1.83, 0.121], [-0.95772, 0.14656, 2.66], [0.351935, 0.902, 2.66 ], [1.97, 0.26, 0.21], [-0.957722, -0.14656, 2.66], [0.351935, -0.90269, 2.66], [0.605787, -0.756132, 2.66], [0.605788, 0.75613, 2.66]])

# print(lista[:,0])
# plt.scatter(lista[:,0], lista[:,1])

# ax = plt.gca()
# circle2 = plt.Circle((0, 0), 2.05, color='b', fill=False)
# ax.add_patch(circle2)
# plt.axis("equal")

# plt.show()


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(lista_3d[:,0], lista_3d[:,1], lista_3d[:,2])
p = Circle((0, 0), 2.05, fill=False)
p2 = Circle((0, 0), 1.05, fill=False)
ax.add_patch(p)
art3d.pathpatch_2d_to_3d(p, z=0, zdir="z")
ax.add_patch(p2)
art3d.pathpatch_2d_to_3d(p2, z=2.69, zdir="z")
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(-0.1, 3)

plt.show()
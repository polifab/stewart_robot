import matplotlib.pyplot as plt
import numpy as np

lista = np.array([[-1.21,1.57], [-1.21,-1.57], [1.97,-0.26], [-0.759, -1.83]])
print(lista[:,0])
plt.scatter(-0.759, 1.83)
plt.scatter(1.74840375, -0.47626371)
plt.scatter(1.97, 0.26)
plt.scatter(lista[:,0], lista[:,1])
#array([ 1.74840375, -0.47626371,  1.98013631,  1.        ])
ax = plt.gca()
circle2 = plt.Circle((0, 0), 2.05, color='b', fill=False)
ax.add_patch(circle2)
plt.axis("equal")
# plt.plot([-1.21,1.57], '*')
# plt.plot([-1.21,-1.57], '*')
# plt.plot([1.97,-0.26], '*')
# plt.plot([1.97, 0.26], '*')
# plt.plot([-0.759, 1.83], '*')
# plt.plot([-0.759, -1.83], '*')

plt.show()

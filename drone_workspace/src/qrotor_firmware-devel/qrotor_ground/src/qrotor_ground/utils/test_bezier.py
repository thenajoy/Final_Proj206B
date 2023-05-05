from bezier2 import Bezier
import numpy as np
import matplotlib.pyplot as plt

b  = Bezier()
alpha = np.array([[0,1,1,1,1,0],
				[1,2,2,2,2,1]])
s = np.linspace(0,1,10)
y = b.bezier(alpha, s)
dy = b.dbezier(alpha, s)
# pritn(s)
# print(y)
plt.figure()
plt.plot(s,y[0,:])
plt.plot(s,y[1,:])
plt.plot(s, dy[0,:])
plt.plot(s, dy[1,:])
plt.show()
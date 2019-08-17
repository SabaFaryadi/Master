import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

# Our 2-dimensional distribution will be over variables X and Y
N = 40
X = np.linspace(0,1000, N)
Y = np.linspace(0, 300, N)
X, Y = np.meshgrid(X, Y)

# Mean vector and covariance matrix
mu = np.array([770,180])
Sigma = np.array([[2000., -1000], [-1000, 2000.]])

# Pack X and Y into a single 3-dimensional array
pos = np.empty(X.shape + (2,))
pos[:, :, 0] = X
pos[:, :, 1] = Y

mu2 = np.array([220,118])
Sigma2 = np.array([[2000., -1000], [-1000, 2000.]])


def multivariate_gaussian(pos, mu, Sigma):
    """Return the multivariate Gaussian distribution on array pos."""

    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

    return np.exp(-fac / 2) / N

# The distribution on the variables X, Y packed into pos.
Z = multivariate_gaussian(pos, mu, Sigma)
Z2 = multivariate_gaussian(pos, mu2, Sigma2)


# plot using subplots
fig = plt.figure()
'''ax1 = fig.add_subplot(1,1,1,projection='3d')

ax1.plot_surface(X, Y, Z+Z2, rstride=3, cstride=3, linewidth=1, antialiased=True,
                cmap=cm.viridis)
ax1.view_init(55,-70)
ax1.set_xticks([])
ax1.set_yticks([])
ax1.set_zticks([])
ax1.set_xlabel(r'$x$')
ax1.set_ylabel(r'$y$')'''
ax2 = fig.add_subplot(1,1,1,projection='3d')
ax2.contourf(X, Y, (Z+Z2), zdir='z', offset=0, cmap=cm.viridis)
#ax2.contourf(X, Y,Z2, zdir='z', offset=0, cmap=cm.viridis)
ax2.view_init(90,270)

ax2.grid(False)
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_zticks([])
ax2.set_xlabel(r'$x_1$')
ax2.set_ylabel(r'$x_2$')

plt.show()
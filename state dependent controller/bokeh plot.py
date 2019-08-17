'''from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D


def CordinateGenerator(w,l):
    # l= The number of horizontal nodes
    # w= The number of vertical nodes
    a=60 #distance between each pair of horizontal nodes
    b=60 #distance between each pair of vertical nodes
    p=w*l
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    Rx=0
    Ry=0
    n=0

    for i in range(0, w):
        for j in range(0, l):
            if j == 0:
                Rx = Rx
            else:
                Rx = Rx + a
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Rx =0
        Ry = (i + 1) * (b)
    return x, y
def MatrixMaker(w,l,d):
    # M, our matrix. Set to a size to accomidate p the total number of points
    p = w*l
    m = np.zeros([p, p], dtype=int)

    # create links between vertical bidirectional paths

    n = 0
    for j in range(0,w):
        for i in range(0, l-1):

            m[n, n + 1] =m[n+ 1, n ] = d
            n = n + 1
        n = n + 1
    k=0
    for j in range(0,w-1):
        for i in range(0, l):
            m[k,k+l]=m[k+l,k]=d
            k=k+1

    return m
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

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.distances[(from_node, to_node)] = distance
if __name__ == '__main__':
    X, Y = CordinateGenerator(6, 15)
    N = 2000
    X = np.linspace(0, 840, N)
    Y = np.linspace(0, 300, N)
    X, Y = np.meshgrid(X, Y)

    # Mean vector and covariance matrix
    mu = np.array([660, 180])
    Sigma = np.array([[2500., -125], [-125, 2500.]])

    # Pack X and Y into a single 3-dimensional array
    pos = np.empty(X.shape + (2,))
    pos[:, :, 0] = X
    pos[:, :, 1] = Y

    mu2 = np.array([120, 120])
    Sigma2 = np.array([[3500., -175], [-175, 3500.]])
    Z = multivariate_gaussian(pos, mu, Sigma)
    Z2 = multivariate_gaussian(pos, mu2, Sigma2)
    #fig = plt.figure()
    #ax2 = fig.add_subplot(1, 1, 1, projection='3d')
    plt.contourf(X, Y, (Z - Z2),colors=['#F0F0F0','#D3D3D3','#C0C0C0','#A9A9A9','#808080','#696969','#474747','#2B2B2B'], zdir='z', offset=0)
    # ax2.contourf(X, Y,Z2, zdir='z', offset=0, cmap=cm.viridis)
    #plt.view_init(90, 270)

    plt.grid(False)
    #plt.set_xticks([])
    #plt.set_yticks([])
    #plt.set_zticks([])
    #plt.savefig('Gplot.png',bbox_inches ='tight')

    plt.show()

    img = plt.imread('Gplot.png')
    fig, ax = plt.subplots()
    ax.imshow(img, extent=[0,840, 0, 300])

    graph = Graph()
    points=MatrixMaker(6,15,1)
    X,Y=CordinateGenerator(6,15)
    plt.plot(X, Y, 'co', markersize=10)
    # plt.scatter(X,Y)
    rowNumber = 0
    columnNumber = 0
    priorityValue = dict()
    for row in points:
        graph.add_node(rowNumber)
        rowNumber = rowNumber + 1
    n = rowNumber
    print(n)
    for i in range(0, n):
        for j in range(0, n):
            if points[i, j] != 0:
                graph.add_edge(i, j, points[i, j])
    for (i,j)in graph.distances:
        plt.plot([X[i],X[j]],[Y[i],Y[j]],color='r')
    i=0
    for p in range(0, len(X)):
        plt.annotate(i, xy=(X[p], Y[p]), color='k', size=10)
        i = i + 1
    print(graph.distances)
    plt.xlabel('X Position',size=12)
    plt.ylabel('Y Position', size=12)
    plt.show()'''

from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=False)
import numpy as np
import matplotlib.pyplot as plt


# Example data
t = np.arange(0.0, 1.0 + 0.01, 0.01)
s = np.cos(4 * np.pi * t) + 2

plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.plot(t, s)

plt.xlabel(r'\textbf{time} (s)')
plt.ylabel(r'\textit{voltage} (mV)',fontsize=16)
plt.title(r"\TeX\ is Number "
          r"$\displaystyle\sum_{n=1}^\infty\frac{-e^{i\pi}}{2^n}$!",
          fontsize=16, color='gray')
# Make room for the ridiculously large title.
plt.subplots_adjust(top=0.8)

plt.savefig('tex_demo')
plt.show()


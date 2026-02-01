from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

def plot_mesh(vertices, faces):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.axis('equal')
    ax.set_xlim(vertices[:, 0].min(), vertices[:, 0].max())
    ax.set_ylim(vertices[:, 1].min(), vertices[:, 1].max())
    ax.set_zlim(vertices[:, 2].min(), vertices[:, 2].max())
    
    tuple_list = list(zip(vertices[:, 0], vertices[:, 1], vertices[:, 2]))
    poly3d = [[tuple_list[faces[ix][iy]] for iy in range(len(faces[0]))] for ix in range(len(faces))]
    ax.add_collection3d(Poly3DCollection(poly3d, facecolors=(0.5, 0.5, 0.5, 0.8), linewidth=1, edgecolors=(0, 0, 0)))
    
    plt.show()

def plot_per_vertex(vertices, faces, per_vertex):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.axis('equal')
    ax.set_xlim(vertices[:, 0].min(), vertices[:, 0].max())
    ax.set_ylim(vertices[:, 1].min(), vertices[:, 1].max())
    ax.set_zlim(vertices[:, 2].min(), vertices[:, 2].max())
    
    tuple_list = list(zip(vertices[:, 0], vertices[:, 1], vertices[:, 2]))
    poly3d = [[tuple_list[faces[ix][iy]] for iy in range(len(faces[0]))] for ix in range(len(faces))]
    ax.add_collection3d(Poly3DCollection(poly3d, facecolors=(0.5, 0.5, 0.5, 0.8), linewidth=1, edgecolors=(0, 0, 0)))

    #ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], per_vertex)

    plt.show()

def highlight_edges(vertices, faces, edges, colors=(1.0, 0.0, 0.0)):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.axis('equal')
    ax.set_xlim(vertices[:, 0].min(), vertices[:, 0].max())
    ax.set_ylim(vertices[:, 1].min(), vertices[:, 1].max())
    ax.set_zlim(vertices[:, 2].min(), vertices[:, 2].max())
    
    tuple_list = list(zip(vertices[:, 0], vertices[:, 1], vertices[:, 2]))
    poly3d = [[tuple_list[faces[ix][iy]] for iy in range(len(faces[0]))] for ix in range(len(faces))]
    ax.add_collection3d(Poly3DCollection(poly3d, facecolors=(0.5, 0.5, 0.5, 0.8), linewidth=0.5, edgecolors=(0, 0, 0, 0.1)))
    
    segments = vertices[edges]
    ax.add_collection3d(Line3DCollection(segments, linewidth=2, colors=colors))
    
    plt.show()

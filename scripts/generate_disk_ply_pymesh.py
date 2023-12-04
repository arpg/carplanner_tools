import math 
import numpy as np
import pymesh2 as pymesh

offset = [0.0, 0.0, 0.0]
radius = 1.0
ang_res = 360.0/8*math.pi/180.0

vertices = []
faces = []
for ii in range(int(2*math.pi/ang_res)):
    vertices += [[0.0, 0.0, 0.0]]
    vertices += [[radius*math.cos(ii), radius*math.sin(ii), 0,0]]
    vertices += [[radius*math.cos(ii+1), radius*math.sin(ii+1), 0,0]]
    faces += [[ii, ii+1, ii+2]]

vertices = np.array(vertices)
faces = np.array(faces)

mesh = pymesh.form_mesh(vertices, faces)

print(mesh)

mesh, info = pymesh.remove_isolated_vertices(mesh)
mesh, info = pymesh.remove_duplicated_vertices(mesh, tol)

print(mesh)
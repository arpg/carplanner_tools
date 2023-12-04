import math
import numpy as np
from plyfile import PlyData, PlyElement

# Read .ply file
# input_file = "/home/arpg/mocha_ws/src/mochagui/labLoop.ply"
# plydata = PlyData.read(input_file)

offset = [0.0, 0.0, 0.0]
radius = 1.0
num_faces = 8
ang_res = 360.0/num_faces*math.pi/180.0




vertices = np.array([None]*(num_faces+1))
faces = np.array([None]*(num_faces))
vertices[0] = (0.0, 0.0, 0.0)
# plydata['vertex'][1] = np.array([radius*math.cos(ii*ang_res), radius*math.sin(ii*ang_res), 0.0])
for ii in range(int(2*math.pi/ang_res)):
    # plydata['vertex'][0] = np.array([0.0, 0.0, 0.0])
    vertices[ii+1] = (radius*math.cos(ii*ang_res), radius*math.sin(ii*ang_res), 0.0)
    # plydata['vertex'][ii+2] = np.array([radius*math.cos((ii+1)*ang_res), radius*math.sin((ii+1)*ang_res), 0.0])
    faces[ii] = (0, (ii+1)%8, (ii+2)%8)

print(vertices)
exit

el = PlyElement.describe(vertices, 'vertices') #, np.array(faces), 'faces')
plydata = PlyData([el])

print(plydata)
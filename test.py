import numpy as np
from SherControl_Scripts_clean import *
import math as math

N = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

nx = N[:, 0]
ny = N[:, 1]
nz = N[:, 2]

rb = np.zeros([1, 3])

fv = np.array([0, -0.05, 0])

rt  = np.array([0, -0.2, 0]) + fv + rb

rtb = np.array([0, -0.2, 0]) + rb

L1 = -0.15

p0 = rb.transpose()
p1 = rt.transpose()
pb = rtb.transpose()

qPC = ik_point2point(p0, p1, pb, nx, ny, nz, L1, L1)

print qPC * 180/PI

#!/usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

"""
Created on Sat Oct 21 18:48:45 2017

@author: mustang
"""

import numpy as np
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

PI = math.pi


# %% Morphed Oscillator


def LowerControl(parameters, X0, dt, q):  # size 28x3

    # 48x1
    # 1x1

    # # Initializing
    # dX0:= [dw
    #        w
    #        dPHI,
    #        dfa, dfh, dfk,
    #        d2fa, d2fh, d2fk,
    #        dRa, dRh, dRk]

    dX = np.zeros([48, 1])

    # Posture control and reflex

    Epos = parameters[12:16, :]  # size: 4x3

    ErfxA = parameters[16:20, 0]  # size: 4x1
    Erfxh = parameters[16:20, 1]  # size: 4x1
    ErfxK = parameters[16:20, 2]  # size: 4x1

    # # Frequency and phase

    w = parameters[20:24, 0]  # size: 4x1
    PHI = parameters[24:28, 0]  # size: 4x1

    # # calculation
    # Frequency

    dX[0:4, 0] = 10 * (w - X0[0:4, 0])  # size: 4x1
    dX[4:8, 0] = X0[0:4, 0]   # size: 4x1

    # Phase

    dX[8:12, 0] = 10 * (PHI - X0[8:12, 0])  # size: 4x1

    dRa = 5 * Epos[:, 0] + 0 * ErfxA  # size: 4x1
    dRh = 5 * Epos[:, 1] + 0 * Erfxh  # size: 4x1
    dRk = 5 * Epos[:, 2] + 0 * ErfxK  # size: 4x1

    dX[36:40, 0] = dRa  # size: 4x1
    dX[40:44, 0] = dRh  # size: 4x1
    dX[44:48, 0] = dRk  # size: 4x1

    # # Integration

    X = dX * dt + X0  # size: 48x1

    # # Angle

    TaoA = X[36:40, 0]  # size: 4x1
    TaoH = X[40:44, 0]  # size: 4x1
    TaoK = X[44:48, 0]  # size: 4x1

    return [TaoA, TaoH, TaoK, X, dX]


# %% Higher Level control

def MidControl(X0, q, dq, F, C, Acc, PHI, dt, Gb, Gl, omega, vs, vf, theta, Phi, w, Phihk, qc, Cq, T):
    # State vector initial values # size : 16x3
    # Joint angle                # size: 3x4
    # Contact forces             # size: 4x1
    # pizo-contact for stumble    # size: 4x1
    # Acceleration of CM       # size: 3x1
    # Orientation of the body  # size: 3x1
    # time step                  # size: 1x1
    # Geometry of the body      # size: 3x4
    # Geometry of the legs      # size: 3x8
    # Initial state vector      # size: ???
    # Turning velocity       # size: 1x1
    # forward velocity          # size: 1x1
    # phase                  # size: 4x1
    # Phase difference       # size: 4x1
    # Actual frequency      # size: 4x1
    # Hip- knee Phase difference # size: 4x1

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% POSTURE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # few constants
    F_threshold = 80  # size: 1x1
    # %%%%%%%%%%%%%%%%%%%%%%% State variable %%%%%%%%%%%%%%%%%%%%%%%%%%%
    zai_LER0 = X0[4:8, 0]
    Ts0 = X0[4:8, 1]
    zai_LRR_h0 = X0[8:12, 1]
    zai_LRR_k0 = X0[8:12, 2]
    # ---------------------------- Input ----------------------------

    roll_b = PHI[0]  # size: 1x1
    pitch_b = PHI[1]  # size: 1x1
    yaw_b = PHI[2]  # size: 1x1

    theta = theta % (2 * PI)
    # ---------------------------- Rotation matrix ----------------------------

    R = Rotation_matrix(roll_b, 0, pitch_b)

    # ---------------------------- Body orientation and state of the end points ----------------------------
    Fc= np.zeros([4,1])
    phi_y0 = np.array([PHI[0], PHI[1], 0])
    rb = body_orientation(Gb, phi_y0)  # size: 3x4
    rl = leg_positions(Gl, q, rb)

    for i in range(0,4):
        if F[i] > 0.01 * F_threshold:
            Fc[i] = 100

    N = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    n = np.dot(R, N)

    nx = n[:, 0]
    ny = n[:, 1]
    nz = n[:, 2]

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Selection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    PhiT = np.zeros([4, 1])

    if abs(vf) < 0.005:
        # canter

        PhiT = np.array([2, 0, 2, 0]) * 0 * PI / 2  # size: 4x1

    if 0.005 <= abs(vf) < 0.1:
        # trot

        PhiT = np.array([0, 2, 2, 0]) * 0 * PI / 2  # size: 4x1

    if abs(vf) >= 0.1:
        # gallop

        PhiT = np.array([0, 0.5, 2.5, 2]) * 0 * PI / 2  # size: 4x1

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Frequency determination %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    w_des = 15 * (-1.7 * vf ** 2 + 4.7 * abs(vf)) * np.ones([4, 1])  # frequency of oscillation
    # size: 4x1
    wd = w_des

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% VMC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Determining velocity%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # % ------------------ Posture control - ---------------------------------
    rt = np.zeros([3, 4])
    rtb = np.zeros([3, 4])

    # inserting learning function
    fv = np.zeros([3, 4])
    alphaAA = np.zeros([3, 4])

    for i in range(0, 4):
        if theta[i] <PI/2 :
           xv=0.05
           yv= 0
        elif PI/2<=  theta[i] <PI :
           xv=0
           yv= 0.05
        elif PI<=  theta[i] <3*PI/2 :
           xv=-0.05
           yv= 0
        else:
           xv=0
           yv= -0.05

        rv = np.array([xv, yv, 0])
        Rv = np.dot(R, rv)

        if F[i] < 0.1 * F_threshold:
            if i <= 1:
                alphaAA[0, i] = np.sign(vf) * 2 * (-vs + omega) * (sin(0.5 * (theta[i] + Phi[i] + PI) % (2 * PI))) ** 2
            else:
                alphaAA[0, i] = np.sign(vf) * 2 * (-vs - omega) * (sin(0.5 * (theta[i] + Phi[i] + PI) % (2 * PI))) ** 2

        fv[:, i] = np.array([Rv[0], Rv[1], 0])

    # target at theta_0 its is posture control rest of the time tracking of trajectories

    for i in range(0, 4):
        rt[:, i]  = np.array([-0.0, -0.25, 0]) + fv[:, i] + rb[:, i]
        rtb[:, i] = np.array([-0.0, -0.25, 0]) + rb[:, i]

    # ---------------------Virtual Model Control---------------------------

    Epos = np.zeros([3, 4])  # size: 3x4

    for i in range(0, 4):

        # ----------------- Target position - -------------------------
        p0 = rb[:, i]
        p1 = rt[:, i]
        pb = rtb[:, i]

        qPC = ik_point2point(p0, p1, pb, nx, ny, nz, Gl[1, i], Gl[1, 4 + i])
	#if i==0:
	#    print qPC*180/PI
        # p0 = vector location of the base wrt the cm
        # p1 = vector location of the target point wrt to
        #  the cm
        # nx, ny, nz = rotated basis vectors

        Epos[:, i] =  1.5 * (qPC + alphaAA[:, i] - q[:, i])

    # % ------------------------Complete reflex---------------------------------
    rfxa = 0.0  # size: 4x1
    rfxh = 0.0  # size: 4x1
    rfxk = 0.0  # size: 4x1

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    parameters = np.zeros([28, 3])  # size: 28x3
    for i in range(0, 4):
        parameters[i + 12, :] = Epos[:, i].transpose()  # size: 1x3
        parameters[i + 16, 0] = 0  # size: 1x1
        parameters[i + 16, 1] = 0  # size: 1x1
        parameters[i + 16, 2] = 0  # size: 1x1
        parameters[i + 20, 0] = wd[i]  # size: 1x1
        parameters[i + 24, 0] = PhiT[i]  # size: 1x1

    # %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% State variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    X = np.zeros([12, 3])  # size: 16x1

    return [parameters, X]  # size: 28x3, 16x1


def HigherControl(k, gP, v0):
    if k == ord('W') or gP == 11:
        v = v0 + 0.001
    elif k == ord('S') or gP == 8:
        v = v0 - 0.001
    else:
        v = v0

    if k == ord('A') or gP == 10:
        omega = 0.25
    elif k == ord('D') or gP == 9:
        omega = -0.25
    else:
        omega = 0

    if k == ord('E') or gP == 5:
        vs = 0.25
    elif k == ord('Q') or gP == 4:
        vs = -0.25
    else:
        vs = 0

    return [v, vs, omega]


# %% Rotation Matrix

def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0,
                                                           sin(roll), cos(roll)]])
    Ry = np.array([[cos(yaw), 0, sin(yaw)], [0, 1, 0], [-sin(yaw), 0,
                                                        cos(yaw)]])
    Rp = np.array([[cos(pitch), -sin(pitch), 0], [sin(pitch),
                                                  cos(pitch), 0], [0, 0, 1]])

    R = np.dot(Rr, np.dot(Ry, Rp))
    return R


# %% Body Orientation

def body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = np.zeros([3, 4])
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])

    for i in range(0, 4):
        rb[:, i] = np.dot(R, Gb[:, i])

    return rb


# %% Leg position

def leg_positions(Gl, q, rB):
    # Abduction positions are from 1 to 4
    # Hip positions are from 5 to 8
    # Knee positions are from 9 to 12

    rl = np.zeros([3, 8])
    for i in range(0, 4):
        Rh = np.dot(Rotx(q[0, i]), Rotz(q[1, i]))
        Rk = Rotz(q[2, i])
        rl[:, i] = Rh.dot(Gl[:, i]) + rB[:, i]
        rl[:, i + 4] = np.dot(Rh, Rk).dot(Gl[:, i + 4]) + rl[:, i]
    return rl


# %% Jacobian


def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, np.dot(Rh,
                                                            Rk)).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n]) + np.dot(Ra, np.dot(dRh,
                                                           Rk)).dot(Gl[:, n + 4])
    J2 = np.dot(Ra, np.dot(Rh, dRk)).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J


# %% Rotation matrix about a single axis

def Rotx(x):
    R = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    return R


def Roty(x):
    R = np.array([[cos(x), 0, sin(x)], [0, 1, 0], [-sin(x), 0, cos(x)]])
    return R


def Rotz(x):
    R = np.array([[cos(x), -sin(x), 0], [sin(x), cos(x), 0], [0, 0, 1]])
    return R


def dRotx(x):
    R = np.array([[0, 0, 0], [0, -sin(x), -cos(x)], [0, cos(x),
                                                     -sin(x)]])
    return R


def dRoty(x):
    R = np.array([[-sin(x), 0, cos(x)], [0, 0, 0], [-cos(x), 0,
                                                    -sin(x)]])
    return R


def dRotz(x):
    R = np.array([[-sin(x), -cos(x), 0], [cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R


# %% ---------------------Foot trajectory-----------------------------

def foot_trajectory(n, Gb, r_off, Phi):
    X = np.loadtxt('x_data.txt')
    Y = np.loadtxt('y_data.txt')

    r = np.array([X[n], 0, Y[n]])  # X and Y are data loaded from .txt file

    R = Rotz(Phi).dot(r) + r_off

    return R


# %% -----------------------Inverse Kinematics --------------------------

def Inverse_Kinamatics(Lh, Lk, r):
    l3 = np.dot(r, r) ** 0.5
    l1 = Lh[1]
    l2 = Lk[1]

    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    q2 = math.acos(v1 / v2)
    q1 = math.asin(l2 * sin(math.pi - q2) / l3)

    return q1, q2


# %% ------------------- Data update -------------------------

# %% ------------------------ Kernel --------------------------
# Theta: 50x1 vector containing phases fron 0 to 360
# Q: the  resulting data.
#       All the modification takes place only in Q.

def Regression_kernel_C(Theta, sigmaf, lf):
    N = len(Theta)

    K = np.zeros([N, N])

    for i in range(0, N):

        for j in range(0, N):
            v1 = np.exp(-(Theta[j] - Theta[i]) ** 2 / lf[i] ** 2)
            K[i, j] = sigmaf[i] ** 2 * v1
            if i == j:
                K[i, j] = K[i, j] + 1e-6

    C = inv(K)

    return C


def Regression_kernel_Kt(Theta, sigma_f, lf, theta):
    N = len(Theta)
    Kt = np.zeros([N, 1])
    for i in range(0, N):
        Kt[i] = sigma_f[i] ** 2 * np.exp(-(theta - Theta[i]) ** 2 / lf[i] ** 2)
    return Kt


def Regression_kernel_dKt(Theta, sigmaf, lf, theta, w):
    N = len(Theta)
    Kt = np.zeros([N, 1])
    for i in range(0, N):
        Kt[i] = - 2 * w * (theta - Theta[i]) * sigmaf[i] ** 2 * np.exp(-(theta - Theta[i]) ** 2 / (lf[i] ** 2)) / lf[
            i] ** 2

    return Kt


# %% -------------------Regression dataset---------------------


def Regression_GP(Kq, Cq, Q):
    kq = Kq.transpose()
    q = np.dot(kq, np.dot(Cq, Q))  # mean of the value
    return q


def ik_point2point(p0, p1, pb, nx, ny, nz, l1, l2):
    r = p1 - p0
    rb = pb - p0
    rn = np.array([np.dot(nx, rb), np.dot(ny, rb), np.dot(nz, rb)])
    rnv = np.array([np.dot(nx, r), np.dot(ny, r), np.dot(nz, r)])

    beta = math.atan2(rnv[0], abs(rnv[1]))  # Angular rotation

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1

    alpha = math.atan2(rn[2], abs(rn[1]))  # Abduction angle
    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)

    Q = np.array([-alpha, theta, phi])

    return Q


def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

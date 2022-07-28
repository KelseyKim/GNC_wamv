#!/usr/bin/env python2

import rospy
import numpy as np
from math import pi, cos, sin, atan2

class KalmanFilter:

    def __init__(self):
        # Declare time step
        self.dt = float()

        # Declare gps offsets
        self.xp_port = float()
        self.yp_port = float()
        self.xp_starboard = float()
        self.yp_starboard = float()

        # Declare added mass parameters
        self.m = float()
        self.Iz = float()
        self.xg = float()
        self.Xudot = float()
        self.Yvdot = float()
        self.Yrdot = float()
        self.Nrdot = float()

        # Declare damping matrix parameters
        self.Xu = float()
        self.Yv = float()
        self.Yr = float()
        self.Nv = float()
        self.Nr = float()

        # Declare control input force
        self.u = np.zeros((3,1), dtype=float)

        # Declare Kalman Filter prediction step variables
        self.X_k0 = np.zeros((8,1), dtype=float)           # Previous post-measurement estimate
        self.X_predict = np.zeros((8,1), dtype=float)      # Current pre-measurement estimate
        self.X_k = np.zeros((8,1), dtype=float)            # Current post-measurement estimate

        self.Q = np.zeros((8,8), dtype=float)              # Process noise covariance

        self.P_k0 = 0.1*np.eye(8)                          # Previous post-measurement covariance
        self.P_predict = np.zeros((8,8), dtype=float)      # Current pre-measurement covariance
        self.P_k = np.zeros((8,8), dtype=float)            # Current post-measurement covariance

        self.A = np.zeros((8,8), dtype=float)              # A matrix
        self.B = np.zeros((8,3), dtype=float)              # B matrix

        self.MinvD = np.zeros((3,3), dtype=float)           # Variable to hold inv(M)*D
        self.Minv =  np.zeros((3,3), dtype=float)           # Variable to hold inv(M)

        # Declare Kalman Filter post-measurement variables
        self.H = np.zeros((8,8), dtype=float)              # Measurement model
        self.R = np.zeros((8,8), dtype=float)              # Measurement covariance matrix
        self.K = np.zeros((8,8), dtype=float)              # Kalman gain
        self.z = np.zeros((8,1), dtype=float)              # Measurements
        self.I = np.eye(8, dtype=float)                  # Identity Matrix

    def set_dynamic_model(self):
        # Create mass matrix
        self.M = np.array([
        [self.m-self.Xudot, 0, 0],
        [0, self.m-self.Yvdot, self.m*self.xg-self.Yrdot],
        [0, self.m*self.xg-self.Yrdot, self.Iz-self.Nrdot]
        ])

        # Create damping matrix
        self.D = np.array(
        [[self.Xu, 0, 0],
        [0, self.Yv, self.Yr],
        [0, self.Nv, self.Nr]])


    def set_kalman_matrices(self):
        self.MinvD = np.matmul(np.linalg.inv(self.M), self.D).astype(float)
        self.Minv = np.linalg.inv(self.M).astype(float)

        self.B = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [self.Minv[0,0], self.Minv[0,1], self.Minv[0,2]],
        [self.Minv[1,0], self.Minv[1,1], self.Minv[1,2]]
        ])

        self.H = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1]
        ])


    def update_state(self):

        self.A = np.array([
            [1, 0, self.dt*(-sin(self.X_k0[2,0])*self.X_k0[3,0] - cos(self.X_k0[2,0])*self.X_k0[4,0]), cos(self.X_k0[2,0])*self.dt, -sin(self.X_k0[2,0])*self.dt, 0, 0, 0],
            [0, 1, self.dt*(cos(self.X_k0[2,0])*self.X_k0[3,0] - sin(self.X_k0[2,0])*self.X_k0[4,0]), sin(self.X_k0[2,0])*self.dt, cos(self.X_k0[2,0])*self.dt, 0, 0, 0],
            [0, 0, 1, 0, 0, self.dt, 0, 0],
            [0, 0, 0, 1, 0, 0, self.dt, 0],
            [0, 0, 0, 0, 1, 0, 0, self.dt],
            [0, 0, 0, -self.dt*self.MinvD[2,0], -self.dt*self.MinvD[2,1], 1 - self.dt*self.MinvD[2,2], 0, 0],
            [0, 0, 0, -self.MinvD[0,0], -self.MinvD[0,1], -self.MinvD[0,2], 0, 0],
            [0, 0, 0, -self.MinvD[1,0], -self.MinvD[1,1], -self.MinvD[1,2], 0, 0]
            ])

        # Without Jacobian
            # [1, 0, 0, cos(self.X_k0[2,0])*self.dt, -sin(self.X_k0[2,0])*self.dt, 0, 0, 0],
            # [0, 1, 0, sin(self.X_k0[2,0])*self.dt, cos(self.X_k0[2,0])*self.dt, 0, 0, 0],
            # [0, 0, 1, 0, 0, self.dt, 0, 0],
            # [0, 0, 0, 1, 0, 0, self.dt, 0],
            # [0, 0, 0, 0, 1, 0, 0, self.dt],
            # [0, 0, 0, -self.dt*self.MinvD[2,0], -self.dt*self.MinvD[2,1], 1 - self.dt*self.MinvD[2,2], 0, 0],
            # [0, 0, 0, -self.MinvD[0,0], -self.MinvD[0,1], -self.MinvD[0,2], 0, 0],
            # [0, 0, 0, -self.MinvD[1,0], -self.MinvD[1,1], -self.MinvD[1,2], 0, 0]
            # ])

            # [1, 0, 0, cos(self.X_k0[2,0])*self.dt, -sin(self.X_k0[2,0])*self.dt, 0, 0, 0],
            # [0, 1, 0, sin(self.X_k0[2,0])*self.dt, cos(self.X_k0[2,0])*self.dt, 0, 0, 0],
            # [0, 0, 1, 0, 0, self.dt, 0, 0],
            # [0, 0, 0, 1, 0, 0, self.dt, 0],
            # [0, 0, 0, 0, 1, 0, 0, self.dt],
            # [0, 0, 0, 0, 0, 1, 0, 0],
            # [0, 0, 0, 0, 0, 0, 1, 0],
            # [0, 0, 0, 0, 0, 0, 0, 1]
            # ])

        # Prediction steps
        self.X_predict = np.matmul(self.A, self.X_k0) + np.matmul(self.B, self.u)
        self.P_predict = np.matmul(self.A, np.matmul(self.P_k0, self.A.T)) + self.Q

        if self.X_predict[2,0] > pi:
            self.X_predict[2,0] = self.X_predict[2,0] - 2*pi
        if self.X_predict[2,0] < -pi:
            self.X_predict[2,0] = self.X_predict[2,0] + 2*pi

        # Measurement update steps
        # Try X_predict if previous state doesn't work
        self.H = np.array([
        [1, 0, -self.xp_port*sin(self.X_k0[2,0]) - self.yp_port*cos(self.X_k0[2,0]), 0, 0, 0, 0, 0],
        [0, 1, self.xp_port*cos(self.X_k0[2,0]) - self.yp_port*sin(self.X_k0[2,0]), 0, 0, 0, 0, 0],
        [1, 0, -self.xp_starboard*sin(self.X_k0[2,0]) - self.yp_starboard*cos(self.X_k0[2,0]), 0, 0, 0, 0, 0],
        [0, 1, self.xp_starboard*cos(self.X_k0[2,0]) - self.yp_starboard*sin(self.X_k0[2,0]), 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1]
        ])

        # [1, 0, 0, 0, 0, 0, 0, 0],
        # [0, 1, 0, 0, 0, 0, 0, 0],
        # [1, 0, 0, 0, 0, 0, 0, 0],
        # [0, 1, 0, 0, 0, 0, 0, 0],
        # [0, 0, 1, 0, 0, 0, 0, 0],
        # [0, 0, 0, 0, 0, 1, 0, 0],
        # [0, 0, 0, 0, 0, 0, 1, 0],
        # [0, 0, 0, 0, 0, 0, 0, 1]
        # ])

        HPH_T_inv = np.linalg.inv(np.matmul(self.H, np.matmul(self.P_predict, self.H.T)) + self.R)
        self.K = np.matmul(self.P_predict, np.matmul(self.H.T, HPH_T_inv))

        self.X_k = self.X_predict + np.matmul(self.K, self.z - np.matmul(self.H, self.X_predict))
        self.P_k = np.matmul(self.I - np.matmul(self.K, self.H), self.P_predict)

        # Wrap yaw between +-pi
        if self.X_k[2,0] > pi:
            self.X_k[2,0] = self.X_k[2,0] - 2*pi
        if self.X_k[2,0] < -pi:
            self.X_k[2,0] = self.X_k[2,0] + 2*pi

        # Update "previous" variables
        self.X_k0 = self.X_k
        self.P_k0 = self.P_k

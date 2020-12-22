#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import math
import sympy
from sympy.utilities.codegen import codegen

def convertToCpp(c_code):
  ret = re.sub(r"\[([0-9]+)\]", lambda m: "(" + str(int(m.group()[1:-1]) // 6) + ", " + str(int(m.group()[1:-1]) % 6) + ")", c_code)
  ret = re.sub(r"X_x", "xw[0]", ret)
  ret = re.sub(r"X_y", "xw[1]", ret)
  ret = re.sub(r"X_z", "xw[2]", ret)
  return ret

def matFromRPY(r, p, y):
  return sympy.Matrix([
    [ sympy.cos(p)*sympy.cos(y),    sympy.sin(p)*sympy.sin(r)*sympy.cos(y) - sympy.cos(r)*sympy.sin(y),   sympy.sin(p)*sympy.cos(r)*sympy.cos(y) + sympy.sin(r)*sympy.sin(y)  ],
    [ sympy.cos(p)*sympy.sin(y),    sympy.sin(p)*sympy.sin(r)*sympy.sin(y) + sympy.cos(r)*sympy.cos(y),   sympy.sin(p)*sympy.cos(r)*sympy.sin(y) - sympy.sin(r)*sympy.cos(y)  ],
    [ -sympy.sin(p),          sympy.cos(p)*sympy.sin(r),                          sympy.cos(p)*sympy.cos(r)                         ],
  ])

def square(x):
  return x*x

def diffScalarByVector(f, x):
  """
  @param f: scalar
  @param x: Mx1 vector
  @return Mx1 vector.
  """
  return sympy.Matrix([ [f.diff(i)] for i in x ])

def diffVectorByVector(f, x):
  """
  @param f: Nx1 vector
  @param x: Mx1 vector
  @return NxM matrix.
  """
  return sympy.Matrix([ [f_i.diff( j ) for j in x] for f_i in f ])

def everything():

  ######################################################################
  # initialize symbols
  ######################################################################

  fu, fv, u0, v0  = sympy.symbols('fu fv u0 v0')

  z_u, z_v  = sympy.symbols('z_u z_v')

  mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw = sympy.symbols('mu_x mu_y mu_z mu_roll mu_pitch mu_yaw')

  X_x, X_y, X_z = sympy.symbols('X_x X_y X_z')

  ######################################################################
  # Initialize vectors and matrices
  ######################################################################

  # 2x1 vector representing a measurement
  z = sympy.Matrix([[z_u],[z_v]])

  # 6x1 vector representing the refined camera pose parameters
  mu = sympy.Matrix([[mu_x], [mu_y], [mu_z], [mu_roll], [mu_pitch], [mu_yaw]])

  # 3x3 rotation matrix from world to the camera frame
  R = matFromRPY(mu_roll, mu_pitch, mu_yaw)

  # 3x1 translation vector from world to the camera frame
  t = sympy.Matrix([[mu_x],[mu_y],[mu_z]])

  # 3x1 vector representing a 3D point in world coordinates
  XW = sympy.Matrix([[X_x],[X_y],[X_z]])

  # 3x1 vector representing the same 3D point in camera coordinates
  XC = R * XW + t

  ######################################################################
  # Compute error function
  ######################################################################

  error = square( z_u - fu * XC[0,0] / XC[2,0] + u0 ) + square( z_v - fv * XC[1,0] / XC[2,0] + v0 )

  ######################################################################
  # Differentiate
  ######################################################################

  d1_mu = diffScalarByVector(error, mu)

  #d1 = error.diff( mu_x )#.diff( mu )
  d2_mu = diffVectorByVector( d1_mu , mu )

  #d2 = error.diff( z )#.diff( mu )
  d2_z = diffVectorByVector( d1_mu, z ).transpose()

  # 6x2 = 6x6 x 6x2
  dA_dz = - d2_mu.transpose() * d2_z

  # 6x6 = 6x2 x 2x2 x 2x6
  # cov = 

  ######################################################################
  # Printing
  ######################################################################

  sympy.init_printing(use_unicode=True)

  #print(d2_mu.shape, d2_z.shape)
  #print(dA_dz.shape)

  #print( len(dA_dz) )
  #for elem in dA_dz:
  #~ for elem in dA_dz:
    #~ print( elem )

  [(c_name, c_code), (h_name, c_header)] = codegen(("dAdz", dA_dz), "C", "derivada", header=False, empty=False)
  print(c_code)

def dg1_du():

  ######################################################################
  # initialize symbols
  ######################################################################

  fu, fv, u0, v0  = sympy.symbols('fu fv u0 v0')

  z_u, z_v  = sympy.symbols('z_u z_v')

  mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw = sympy.symbols('mu_x mu_y mu_z mu_roll mu_pitch mu_yaw')

  X_x, X_y, X_z = sympy.symbols('X_x X_y X_z')

  XC_x, XC_y, XC_z = sympy.symbols('XC_x XC_y XC_z')

  ######################################################################
  # Initialize vectors and matrices
  ######################################################################

  # 2x1 vector representing a measurement
  z = sympy.Matrix([[z_u],[z_v]])

  # 6x1 vector representing the refined camera pose parameters
  mu = sympy.Matrix([[mu_x], [mu_y], [mu_z], [mu_roll], [mu_pitch], [mu_yaw]])

  # 3x3 rotation matrix from world to the camera frame
  R = matFromRPY(mu_roll, mu_pitch, mu_yaw)

  # 3x1 translation vector from world to the camera frame
  t = sympy.Matrix([[mu_x],[mu_y],[mu_z]])

  # 3x1 vector representing a 3D point in world coordinates
  XW = sympy.Matrix([[X_x],[X_y],[X_z]])

  # 3x1 vector representing the same 3D point in camera coordinates
  XC = R * XW + t

  ######################################################################
  # Compute function
  ######################################################################

  g1 = z_u - fu * ( XC[0] / XC[2] ) + u0

  dg1_dXC = diffScalarByVector( X_x / X_z, XW )

  print("dg1_dXC:", dg1_dXC.shape)
  for elem in dg1_dXC:
    print(elem)

  # 3x6 matrix
  dXC_du = diffVectorByVector( XC, mu )

  print("dXC_du:", dXC_du.shape)
  for elem in dXC_du:
    print(elem)

  dg1_du = -2 * fu * sympy.Matrix([1/XC_z],[0],[XC_x/(X_z*X_z)]) * dXC_du

def dXcdmu():

  ######################################################################
  # initialize symbols
  ######################################################################

  mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw = sympy.symbols('mu_x mu_y mu_z mu_roll mu_pitch mu_yaw')

  X_x, X_y, X_z = sympy.symbols('X_x X_y X_z')

  ######################################################################
  # Initialize vectors and matrices
  ######################################################################

  # 6x1 vector representing the refined camera pose parameters
  mu = sympy.Matrix([[mu_x], [mu_y], [mu_z], [mu_roll], [mu_pitch], [mu_yaw]])

  # 3x3 rotation matrix from world to the camera frame
  # matFromRPY gives the orientation matrix for the rpy angles,
  # so we need to transpose it to get the rotation matrix.
  R = matFromRPY(mu_roll, mu_pitch, mu_yaw).transpose()

  # 3x1 translation vector from world to the camera frame
  # mu actually holds the position vector, so we need to transform it.
  t = -R * sympy.Matrix([[mu_x],[mu_y],[mu_z]])

  # 3x1 vector representing a 3D point in world coordinates
  XW = sympy.Matrix([[X_x],[X_y],[X_z]])

  # 3x1 vector representing the same 3D point in camera coordinates
  XC = R * XW + t

  ######################################################################
  # Compute function
  ######################################################################

  # 3x6 matrix
  dXC_dmu = diffVectorByVector( XC, mu )

  ######################################################################
  # Print results
  ######################################################################

  sympy.init_printing(use_unicode=True)

  [(c_name, c_code), (h_name, c_header)] = codegen(("dXCdmu", dXC_dmu), "C", "derivada", header=False, empty=False)

  cpp_code = convertToCpp( c_code )

  print(cpp_code)

def dXC2dmu2():

  ######################################################################
  # initialize symbols
  ######################################################################

  mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw = sympy.symbols('mu_x mu_y mu_z mu_roll mu_pitch mu_yaw')

  X_x, X_y, X_z = sympy.symbols('X_x X_y X_z')

  ######################################################################
  # Initialize vectors and matrices
  ######################################################################

  # 6x1 vector representing the refined camera pose parameters
  mu = sympy.Matrix([[mu_x], [mu_y], [mu_z], [mu_roll], [mu_pitch], [mu_yaw]])

  # 3x3 rotation matrix from world to the camera frame
  # matFromRPY gives the orientation matrix for the rpy angles,
  # so we need to transpose it to get the rotation matrix.
  R = matFromRPY(mu_roll, mu_pitch, mu_yaw).transpose()

  # 3x1 translation vector from world to the camera frame
  # mu actually holds the position vector, so we need to transform it.
  t = -R * sympy.Matrix([[mu_x],[mu_y],[mu_z]])

  # 3x1 vector representing a 3D point in world coordinates
  XW = sympy.Matrix([[X_x],[X_y],[X_z]])

  # 3x1 vector representing the same 3D point in camera coordinates
  XC = R * XW + t

  ######################################################################
  # Compute function
  ######################################################################

  # 3x6 matrix
  dXC_dmu = diffVectorByVector( XC, mu )

  #~ print(dXC_dmu.shape)
  #~ print(dXC_dmu[0,:].shape)
  #~ print(dXC_dmu[1,:].shape)
  #~ print(dXC_dmu[2,:].shape)

  dXC2_dmu2_0 = diffVectorByVector( dXC_dmu[0,:], mu )
  dXC2_dmu2_1 = diffVectorByVector( dXC_dmu[1,:], mu )
  dXC2_dmu2_2 = diffVectorByVector( dXC_dmu[2,:], mu )

  #~ print(dXC2_dmu2_0.shape)
  #~ print(dXC2_dmu2_1.shape)
  #~ print(dXC2_dmu2_2.shape)

  #~ for i in range(6):
    #~ for j in range(6):
      #~ print(i, j, dXC2_dmu2_2[i, j])

  ######################################################################
  # Print results
  ######################################################################

  sympy.init_printing(use_unicode=True)

  [(c_name, c_code), (h_name, c_header)] = codegen(("dXC2dmu2_0", dXC2_dmu2_0), "C", "derivada", header=False, empty=False)
  print(convertToCpp(c_code))

  [(c_name, c_code), (h_name, c_header)] = codegen(("dXC2dmu2_1", dXC2_dmu2_1), "C", "derivada", header=False, empty=False)
  print(convertToCpp(c_code))

  [(c_name, c_code), (h_name, c_header)] = codegen(("dXC2dmu2_2", dXC2_dmu2_2), "C", "derivada", header=False, empty=False)
  print(convertToCpp(c_code))

if __name__ == "__main__":

  #~ dXcdmu()

  dXC2dmu2()

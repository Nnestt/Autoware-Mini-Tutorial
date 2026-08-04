#Acceleration induced by centrifugal force in curves:

# acf - centrifugal acceleration
# v - speed in the curve
# r - radius of the curve

from math import sqrt


acf = v**2 / r

v = sqrt(acf * r)

# Calculate radius and curvature of the curve that passes points (1,4.5), (3,6), (7, 4.5).
# What is the maximum speed this curve can be driven if maximum centrifugal acceleration allowed is 1 m/s2?
# dij = sqrt((xi - xj)**2 + (yi - yj)**2)

# Given points P1(x1,y1), P2(x2,y2), P3(x3,y3).
# Determine the distances between the points:
s = (d12 + d23 + d13)/2
A = sqrt(s(s-d12)(s-d23)(s-d13))

# Calculate the area of the triangle formed by the three points using Heron’s formula


# Apply the Menger Curvature formula
k = 4 * A / (d12 * d23 * d13)

#points
d12 = (1,4.5)
d23 = (3,6)
d13 = (7, 4.5)
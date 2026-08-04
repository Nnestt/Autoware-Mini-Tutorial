import math

# v - final speed
# v0 - initial speed
# a - acceleration
# s - distance


def final_speed(v0, a, s):
    return math.sqrt(v0**2 + 2*a*s)

def initial_speed(v, a, s):
    return math.sqrt(v**2 - 2*a*s)

def convert_kmh_to_ms(v): # Convert km/h to m/s
    return v / 3.6


# 1.What should be our current speed if we want to follow a leading car with speed 40 km/h keeping the same lead distance as it is currently?
# 2.What should be our current speed if we detected an obstacle at 25 meters ahead driving at speed 10 km/h and we want to decelerate at 1 m/s2, keep 4m safety distance and 1.5s reaction time?
# 3.What must be the detection range to stop with 1 m/s2 deceleration from 40 km/h? What about 70 or 90 km/h?

# so - obstacle distance
# ss - safety buffer
# sr - reaction buffer

# sr = v * tr

# tr - reaction time

def stopping_distance(v0, a):
    return v0**2 / (2*a)

# constants shared by Q2 and Q3
a = 1       # m/s^2 deceleration
tr = 1.5    # s reaction time
ss = 4      # m safety buffer

# 1. Match leading car's speed to keep the same distance
v1 = convert_kmh_to_ms(40)
print(f"Q1: current speed = {v1:.2f} m/s ({v1*3.6:.2f} km/h)")

# 2. Solve for our current speed v.
# Available braking distance = so - ss - sr, where sr = v*tr (unknown v)
# vo^2 = v^2 - 2*a*(so - ss - v*tr)  ->  v^2 + 2*a*tr*v - (2*a*(so-ss) + vo^2) = 0
so = 25 # distance to obstacle
vo = convert_kmh_to_ms(10)
B = 2*a*tr
C = -(2*a*(so - ss) + vo**2)
v2 = (-B + math.sqrt(B**2 - 4*C)) / 2
print(f"Q2: current speed = {v2:.2f} m/s ({v2*3.6:.2f} km/h)")

# 3. Detection range = reaction distance + braking distance + safety buffer
for kmh in (40, 70, 90):
    v0 = convert_kmh_to_ms(kmh)
    sr = v0 * tr
    detection_range = sr + stopping_distance(v0, a) + ss
    print(f"Q3: from {kmh} km/h -> detection range = {detection_range:.2f} m")

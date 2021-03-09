"""Practical 7 M2A Pod 4"""
# Solve the pendulum using numerical approximation
# Copyright (c) 2008 David M. Harrison
# - updated Mar.3 2021 by Jason Harlow
# from pylab import *
import numpy as np

# The initial angle in degrees:
thetadegmax = 45
# Convert this to radians:
thetamax = (np.pi / 180) * thetadegmax

# The initial angle:
theta = thetamax
# The initial angular velocity:
omega = 0  # released from rest

# Set g and the length of the pendulum
g = 9.80  # m/s^2
L = 1.00  # m

# The time step in seconds:
dt = 0.01

# Initialize the time:
t = 0.

# We want the pendulum to cross theta = 0 two times going in the negative
# direction, and then stop the simulation.  That way we can look at the
# times and figure out the period.  Initialize this counter:
crosscount = 0

# Below we will want to store the old value of the time.
# Set it the "impossible" value of -1 initially.
t_old = -1.

# In case you'd like to use Excel or something to make a little graph of
# angle versus time, let's output a comma separated file as well:
file = open('pendulum_output_default.csv', 'w')

# Run the while-loop for 2.25 full oscillations, or 5 crossings of theta=0:
while crosscount < 3:

    thetadeg = theta * 180 / np.pi
    print("t = ", t, ", theta = ", thetadeg)
    file.write(str(t))
    file.write(',')
    file.write(str(thetadeg))
    file.write('\n')

    # The angular acceleration, i.e. the second derivative of the
    # angle with respect to time.s
    alpha = -(g / L) * np.sin(theta)

    # The new value of the angular velocity
    omega = omega + alpha * dt

    # The change in the angle of the pendulum
    d_theta = omega * dt

    # A rough and ready way to estimate the period of the oscillation.
    # It the angle is positive and adding d_theta to it will make
    # it negative, then it is going through the vertical
    # from right to left.
    if theta > 0 and theta + d_theta < 0:

        crosscount = crosscount + 1

        # If t_old is > 0, then this is not the first cycle of
        # the oscillation. The difference between t and t_old
        # is the period within the resolution of the time step dt
        # and rounding errors. Print the period.

        if t_old > 0:
            print("Initial Angle = ", thetadegmax, "degrees, Period =", t - t_old, "s")

        # Store the current value of the time in t_old
        t_old = t

    # Update the value of the angle
    theta = theta + d_theta

    # Update the time
    t = t + dt

file.close()

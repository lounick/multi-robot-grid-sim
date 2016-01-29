from __future__ import division

timestep = 1 #Timestep in seconds
max_time = 3600 #Maximum simulation time in seconds
time = 0

vehicles = []

while(time < max_time):
    print(time)
    time += timestep
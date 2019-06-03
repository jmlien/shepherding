# bounding box
bbox=[-25.0, 25.0, -5.0, 20.0, -25.0, 25.0]

#starting to define obstacles
obstacle={
size=1
geo = spiral2.obj
color = (0.9, 0.7, 0.3) 
position = (-2, -5)
angle = -0
angle = 0
}

#define flocks
shepherd={
type = simple
size  = 1
geo   = ../../shepherd/behaviors.py/env/robot2.g
color = (0.1,0.4,0.1)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (0,-5)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (22,0)

# ODE stuff
damping= 1
max_force = 10.0
target_attract=5

# define roadmap parameters here
roadmap_n = 150
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 6
geo   = ../../shepherd/behaviors.py/env/robot2.g
color = (0.1,0.1,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (-5,-10)
dist_dev    = 1
afraid      = 15
separation  = 1
cohesion    = 5
alignment   = 3
obst_repulsion = 20
damping = 1
max_force = 30.0
}


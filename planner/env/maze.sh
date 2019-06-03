# bounding box
bbox=[-23.5, 25.5, -1.0, 20.0, -24.5, 25.5]

#starting to define obstacles
obstacle={
size=1
geo = maze2.obj
color = (0.8, 0.05, 0.3) 
position = (0, -2)
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
dist_center = (20,20)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (-5,-15)

# ODE stuff
damping= 1
max_force = 10.0
target_attract=5

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 3
geo   = ../../shepherd/behaviors.py/env/robot2.g
color = (0.1,0.1,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (19,15)
dist_dev    = 3
afraid      = 15
separation  = 1
cohesion    = 5
alignment   = 3
obst_repulsion = 10
damping = 1
max_force = 30.0
}


# bounding box
bbox=[-45.0, 45.0, -1.0, 20.0, -45.0, 45.0]

#starting to define obstacles
obstacle={
size=2
geo = ../../shepherd/behaviors.py/env/brickwall.obj
color = (0.4, 0.1, 0.05) 
color = (0.2, 0.1, 0.4) 
position = (-16.0, -28.0)
position = (16.0, 18.0)
angle = 10
angle = 150
}

obstacle={
size=1
geo = ../../shepherd/behaviors.py/env/fence.obj
color = (0.45, 0.3, 0.05) 
position = (5, -5)
angle = -47
}


#define flocks
shepherd={
type = simple
size  = 2
geo   = ../../shepherd/behaviors.py/env/stone1_S.obj
color = (0.1,0.4,0.1)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (-10,-30)
dist_dev    = 5
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (10,16)

# ODE stuff
damping= 0.5
max_force = 10.0
target_attract=15

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 20
geo   = ../../shepherd/behaviors.py/env/robot2.g
color = (0.1,0.1,0.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (-10,-30)
dist_dev    = 3
afraid      = 10
separation  = 2
cohesion    = 5
alignment   = 2
obst_repulsion = 10
damping = 1
max_force = 30.0
}


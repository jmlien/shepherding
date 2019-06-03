# bounding box
bbox=[-25.0, 25.0, -1.0, 20.0, -25.0, 25.0]

#starting to define obstacles
obstacle={
size=2
geo = env/objs/fence.obj
color = (0.3, 0.2, 0.05) 
color = (0.1, 0.05, 0.3) 
position = (-6.0, -8.0)
position = (6.0, 10.0)
angle = 10
angle = -10
}

#define flocks
shepherd={
type = deform
size  = 10
geo   = env/objs/robot2.g
color = (0.0,0.0,0.0)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (-20,-20)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (20,20)

# ODE stuff
damping= 1
max_force = 20.01
target_attract=5

# define roadmap parameters here
roadmap_n = 100
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 150
geo   = env/objs/robot2.g
color = (0.4,0.4,.9)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (0,-15)
dist_dev    = 2
afraid      = 15
separation  = 1
cohesion    = 10
alignment   = 10
obst_repulsion = 3
damping = 1
max_force = 30.0
}


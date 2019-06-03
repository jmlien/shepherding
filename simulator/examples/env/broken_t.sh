# bounding box
bbox=[-18.0, 18.0, -1.0, 20.0, -18.0, 18.0]

#starting to define obstacles
obstacle={
size=1
geo = env/objs/broken_t.obj
color = (0.5, 0.2, 0.2) 
position = (4, 0)
angle = 0
}

#define flocks
shepherd={
type = deform
size  = 8
geo   = env/objs/robot2.g
color = (0,0,0)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (-17,17)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (-15,-15)

# ODE stuff
damping= 1
max_force = 10.0
target_attract=15

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 80
geo   = env/objs/robot2.g
color = (0.8,0.6,0)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (-15,12)
dist_dev    = 3
afraid      = 15
separation  = 1
cohesion    = 10
alignment   = 10
obst_repulsion = 10
damping = 1
max_force = 30.0
}



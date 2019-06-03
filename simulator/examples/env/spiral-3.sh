# bounding box
bbox=[-25.0, 25.0, -1.0, 20.0, -25.0, 25.0]

#starting to define obstacles
obstacle={
size=1
geo = env/objs/spiral7-obj.obj
color = (0.8, 0.6, 0.4)
position = (0, 0)
angle = 0
}

#define flocks
shepherd={
type = deform
size  = 1
geo   = env/objs/robot2.g
color = (0.0,0.55,0.0)
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
goal = (0,0)

# ODE stuff
damping= 1
max_force = 20.01
target_attract=15

# define roadmap parameters here
roadmap_n = 280
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 3
geo   = env/objs/robot2.g
color = (0.0,0.0,0.65)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (19,22)
dist_dev    = 3
afraid      = 15
separation  = 1
cohesion    = 5
alignment   = 3
obst_repulsion = 10
damping = 1
max_force = 30.0
}


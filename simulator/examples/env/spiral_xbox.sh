# bounding box
bbox=[-25.0, 25.0, -5.0, 20.0, -25.0, 25.0]

#starting to define obstacles
obstacle={
size=1
geo = env/objs/spiral2.obj
color = (0.9, 0.7, 0.3) 
position = (-2, -4)
angle = 0
}

#define flocks
shepherd={
type = manual
#auto_pilot = deform
size  = 2
geo   = env/objs/robot2.g
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
goal = (0,-5)

# ODE stuff
damping= 1
max_force = 5.0
target_attract=5

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 10
geo   = env/objs/robot2.g
color = (0.1,0.1,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (19,19)
dist_dev    = 3
afraid      = 20
separation  = 1
cohesion    = 10
alignment   = 3
obst_repulsion = 10
damping = 1
max_force = 30.0
}


# bounding box
bbox=[-27.5, 24.5, -1.0, 20.0, -26.0, 26.0]

#starting to define obstacles
obstacle={
size=1
geo = env/objs/filter4-obj.obj
color = (0.7, 0.4, 0.3) 
position = (-1, 0)
angle = 0
}

#define flocks
shepherd={
type = deform
size  = 1
geo   = env/objs/robot2.g
color = (0.1,0.4,0.1)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (24,24)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (-10,-20)

# ODE stuff
damping= 1
max_force = 100.0
target_attract=10

# define roadmap parameters here
roadmap_n = 200
roadmap_k = 10
}

# regular flock
flock={
type=scared
size  = 5
geo   = env/objs/robot2.g
color = (0.1,0.1,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (20,20)
dist_dev    = 3
afraid      = 15
separation  = 1
cohesion    = 5
alignment   = 3
obst_repulsion = 10
damping = 1
max_force = 30.0
}


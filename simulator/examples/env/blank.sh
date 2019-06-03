# bounding box
bbox=[-35.0, 35.0, -1.0, 20.0, -35.0, 35.0]

#starting to define obstacles
obstacle={
size=0
geo   = env/objs/robot2.g
}

#define flocks
shepherd={
type = deform
size  = 4
geo   = env/objs/robot2.g
color = (0.1,0.4,0.1)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (-33,-33)
dist_dev    = 3
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = env/goal/blank.obj	

# ODE stuff
damping= 1
max_force = 20.01
target_attract=2

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 40
geo   = env/objs/robot2.g
color = (0.1,0.1,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (-30,-30)
dist_dev    = 1
afraid      = 15
separation  = 1
cohesion    = 10
alignment   = 10
obst_repulsion = 10
damping = 1
max_force = 30.0
}
	

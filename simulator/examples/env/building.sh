# bounding box
bbox=[-35.0, 35.0, -1.0, 40.0, -35.0, 35.0]

#starting to define obstacles
obstacle={
size=2
geo = env/objs/building.obj
color = (0.6, 0.2, 0.05) 
color = (0.0, 0.2, 0.6) 
position = (-13,-1)
position = (13, 1)
angle =0
angle =180
}

#define flocks
shepherd={
type = deform
size  = 15
geo   = env/objs/robot2.g
color = (0,0,0)
mass  = 0.2
view_radius = 500
view_angle = 360
dist_center = (-32,-20)
dist_dev    = 2
separation  = 0
cohesion    = 0
alignment   = 0
obst_repulsion = 0

#steering destination
goal = (20,20)

# ODE stuff
damping= 1
max_force = 20.01
target_attract=10

# define roadmap parameters here
roadmap_n = 80
roadmap_k = 5
}

# regular flock
flock={
type=scared
size  = 200
geo   = env/objs/robot2.g
color = (0.5,0.5,.5)
mass  = 5
view_radius= 5
view_angle= 360
dist_center = (-25,-25)
dist_dev    = 3
afraid      = 15
separation  = 1
cohesion    = 10
alignment   = 10
obst_repulsion = 10
damping = 1
max_force = 30.0
}


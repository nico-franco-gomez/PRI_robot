import numpy as np
from parcours_create import Parcours

'''
This script creates a quarter sphere of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims
'''

par = Parcours(name='quarter_sphere',base='[2]:straight_sol')
par.set_object_lims([(-183.5,183.5),(-80,80),(0,401)]) # Box

# Set first point to move robot safely without touching object
initial_rot = np.array([-52.35,-87.66,126.68])
par.add_point_SPTP([0,0,800],initial_rot,75)

r = 570 # Radius of quarter sphere
points = []

# Rotation handling – only y
## This is defined for Microflown3D and base2: straight_sol !
deltax = -90
initial_rot = np.array([-52.35,-87.66,126.68])
deltay = np.array([-0.97, 1.5, -177.4]) - initial_rot
rate_y = deltay/deltax

def compute_roty(x):
    rot_y = 90/r * x
    new_vec = initial_rot + rate_y*rot_y
    for ind,i in enumerate(new_vec):
        if i>180:
            new_vec[ind] = 180 - i
        elif i<-180:
            new_vec[ind] = -180 - i
    return initial_rot - rate_y*rot_y

# Second security point
par.add_point_SLIN([-250,0,900],compute_roty(-250),2)

# Define points

theta_vec = np.linspace(np.pi/2-0.1,0,10)
i = 0
theta_pos = True
for theta in theta_vec:
    phi_vec = np.linspace(np.pi/2,3*np.pi/2,len(theta_vec)-i)
    if theta_pos:
        for phi in phi_vec:
            coord_cart = par.spherical2cart((r,phi,theta))
            points.append( ( coord_cart, compute_roty(coord_cart[0]) , 'SLIN') )
    else:
        for phi in np.flip(phi_vec):
            coord_cart = par.spherical2cart((r,phi,theta))
            points.append( ( coord_cart, compute_roty(coord_cart[0]) , 'SLIN') )
    i+=1
    theta_pos = not theta_pos

for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

par.export('real_parcours/')



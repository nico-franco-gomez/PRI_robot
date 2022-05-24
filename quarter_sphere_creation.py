import numpy as np
from parcours import Parcours

'''
This script creates a quarter sphere of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims
'''

par = Parcours(base='[2]:straight_sol')
par.set_object_lims([(-183.5,183.5),(-80,80),(0,401)]) # Box

# Set first point to move robot safely without touching object
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,75,marker=0)

r = 600 # Radius of quarter sphere. MAX:600
points = []

# Rotation handling – only y-axis (B)
## This is defined for Microflown3D and base2: straight_sol !
deltaB_rot = 90/r
rotB = lambda x: -90 + x * deltaB_rot

# Second security point
par.add_point_SLIN([0,r,800],[0,rotB(350),180],2,marker=0)

# Define points

theta_vec = np.linspace(np.pi/2-0.01,0,20) # Theta for secure trials
i = 0
theta_pos = True
for theta in theta_vec:
    phi_vec = np.linspace(np.pi/2,3*np.pi/2,len(theta_vec)-i)
    if theta_pos:
        for phi in phi_vec:
            coord_cart = par.spherical2cart((r,phi,theta))
            points.append( ( coord_cart, [0,rotB(coord_cart[0]),180] , 'SLIN') )
    else:
        for phi in np.flip(phi_vec):
            coord_cart = par.spherical2cart((r,phi,theta))
            points.append( ( coord_cart, [0,rotB(coord_cart[0]),180] , 'SLIN') )
    i+=1
    theta_pos = not theta_pos

for n in points:
    eval(f'''par.add_point_{n[2]}(n[0],n[1])''')

par.name = f'quarter_sphere_r{r}'

par.export('real_parcours/')



import numpy as np
import matplotlib.pyplot as plt
from parcours import Parcours

'''
This script creates a quarter sphere of measuring points for an exemplary
loudspeaker, see dimensions in par.set_object_lims
'''

par = Parcours(base='[2]:straight_sol')
par.set_object_lims([(-183.5,183.5),(-140,140),(0,401)]) # Box
point_dist = 50

# Visualization
visualize = True
if visualize:
    fig,ax = plt.subplots(1,1,subplot_kw={'projection':'3d'})
    # Upper
    xx,yy = np.meshgrid(np.arange(par.object_lims[0][0]/10,par.object_lims[0][1]/10,1), # x
                        np.arange(par.object_lims[1][0]/10,par.object_lims[1][1]/10,1)) # y
    ax.scatter(xx, yy, np.ones_like(xx)*par.object_lims[2][1]/10,
                color='xkcd:green',label='Haut-parleur')
    # Sides 1
    xx,zz = np.meshgrid(np.arange(par.object_lims[0][0]/10,par.object_lims[0][1]/10,1),
                        np.arange(par.object_lims[2][0]/10,par.object_lims[2][1]/10,1))
    ax.scatter(xx,np.ones_like(xx)*par.object_lims[1][0]/10,
                zz,color='xkcd:green')
    ax.scatter(xx,np.ones_like(xx)*par.object_lims[1][1]/10,
                zz,color='xkcd:green')
    # Sides 2
    yy,zz = np.meshgrid(np.arange(par.object_lims[1][0]/10,par.object_lims[1][1]/10,1),
                        np.arange(par.object_lims[2][0]/10,par.object_lims[2][1]/10,1))
    ax.scatter(np.ones_like(yy)*par.object_lims[0][0]/10,
                yy,zz,color='xkcd:green')
    ax.scatter(np.ones_like(yy)*par.object_lims[0][1]/10,
                yy,zz,color='xkcd:green')

    vec = np.arange(0,30,1)
    ax.scatter(vec,np.zeros_like(vec),np.zeros_like(vec),color='xkcd:purple',\
        label='Base')
    ax.scatter(np.zeros_like(vec),np.zeros_like(vec),vec,color='xkcd:purple')
    ax.scatter(np.zeros_like(vec),vec,np.zeros_like(vec),color='xkcd:purple')

# Set first point to move robot safely without touching object
initial_rot = np.array([0,-90,180])
par.add_point_SPTP([0,0,800],initial_rot,75,marker=0)

r = 600 # Radius of quarter sphere.
points = []

# Rotation handling – only y-axis (B)
## This is defined for Microflown3D and base2: straight_sol !
deltaB_rot = 90/r
rotB = lambda x: -90 - x * deltaB_rot

# Second security point
par.add_point_SLIN([0,r,800],[0,rotB(-350),180],2,marker=0)

# Define points

theta_vec = np.arange(np.pi/2-0.01,0,-point_dist/r)
# theta_vec = np.arange(np.pi/3-0.01,0,-point_dist/r) # Theta for secure trials
i = 0
theta_pos = True
for theta in theta_vec:
    # phi_vec = np.arange(np.pi/2,3*np.pi/2,point_dist/r*i)
    phi_vec = np.linspace(np.pi/2,3*np.pi/2,len(theta_vec)*3-i*3)
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

if visualize:
    xx = []
    yy = []
    zz = []
    for n in points:
        xx.append(n[0][0]/10)
        yy.append(n[0][1]/10)
        zz.append(n[0][2]/10)
    ax.scatter(xx,yy,zz,color='xkcd:orange',label='Point de\nmesure')
    ax.set_title(f'Sphère – Radius {r//10:.0f} cm')
    ax.set_xlabel('$x$ [cm]')
    ax.set_ylabel('$y$ [cm]')
    ax.set_zlabel('$z$ [cm]')
    fig.tight_layout()
    ax.legend(loc='center',bbox_to_anchor=(-0.09,0.5))
    # fig.savefig(f'plots/sphere_r{r//10:.0f}cm.pdf')
    # fig.savefig(f'''/Users/nicolas/Documents/Uni/Master – 4. Semester 
    #  INSA/Projet de recherche/Latex/Bilder/parcours/
    # sphere_r{r//10:.0f}cm.pdf'''.replace('\n',''))
    plt.show()


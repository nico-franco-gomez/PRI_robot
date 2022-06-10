import numpy as np
import matplotlib.pyplot as plt
from parcours import Parcours

height = np.arange(0, 601, 100, dtype=np.int16)

for h in height:
    # Create object and set limits
    base_parcours = [1272, -51, -355]  # [2]:straight_sol
    par = Parcours(name=f'points2_{h}', base_coord = base_parcours)
    par.set_object_lims([[-183.5,183.5],[-140,140],[0,400]]) # Box
    point_dist = 100  # mm between measuring points, important for limit frequency

    ## Security points 1
    initial_rot = np.array([0,-90,180])
    # par.add_point_SPTP([0,0,800],initial_rot,20,marker=0)
    nom_dist = -530 # nominal distance for plane location
    yy = np.arange(-500, 501, 100)

    par.add_point_SPTP([nom_dist, yy[0], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})

    par.add_point_SPTP([nom_dist, yy[1], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})
    par.add_point_SPTP([nom_dist, yy[2], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})
    par.add_point_SPTP([nom_dist, yy[3], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})
    par.add_point_SPTP([nom_dist, yy[4], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})
    par.add_point_SPTP([nom_dist, yy[5], h], [0, 0, 180], 20,
                    {'s': '010', 't': '001010'})
    par.add_point_SPTP([nom_dist, yy[6], h], [0, 0, 180], 20,
                    {'s': '010', 't': '100011'})
    par.add_point_SPTP([nom_dist, yy[7], h], [0, 0, 180], 20,
                    {'s': '010', 't': '100011'})
    par.add_point_SPTP([nom_dist, yy[8], h], [0, 0, 180], 20,
                    {'s': '010', 't': '100011'})
    par.add_point_SPTP([nom_dist, yy[9], h], [0, 0, 180], 20,
                    {'s': '010', 't': '100011'})



    par.add_point_SPTP([nom_dist, yy[10], h], [0, 0, 180], 20,
                    {'s': '010', 't': '100011'})

    # Export
    par.export(path='real_parcours/parcours_manually/')
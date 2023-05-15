#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 11 13:26:02 2023

@author: cxiong
"""

import numpy as np
import matplotlib.pyplot as plt



depth_list_for_UC_MP = [0,2,4,8,12,16,20,24]
depth_list_for_Manha = [0,2,4,8,12,18,20,24]

UC_num_checked_node = [1, 6, 23, 286, 2084, 13020, 40254, 127483]
UC_time = [0.00023, 0.000908852, 0.001860142, 0.016914845, 0.082772017, 0.404634714, 1.573977947, 6.754719019]

MP_num_checked_node = [1, 5, 10, 34, 203, 990, 4359, 21248]
MP_time = [0.00021, 0.000503063, 0.001079082, 0.004087925, 0.012373924, 0.053452969, 0.199344873, 0.83634901]

Manha_num_checked_node = [1, 5, 10, 24, 64, 325, 750, 1490]
Manha_time = [0.000171, 0.000950813, 0.001138926, 0.003638983, 0.008463144, 0.027305841, 0.06055975, 0.094711065]



plt.plot(depth_list_for_UC_MP, UC_num_checked_node, label="Uniform Cost Search", color='g')
plt.plot(depth_list_for_UC_MP, MP_num_checked_node, label="A* with Misplaced Tile Heuristic", color='b')
plt.plot(depth_list_for_Manha, Manha_num_checked_node, label="A* with Manhattan Distance Heuristic", color='m')
plt.legend()
plt.xlabel("solution depth")
plt.ylabel("number of Node checked")
plt.title("number of Node checked - solution depth")
plt.ylim([0,50000])
plt.grid()
plt.savefig('Node_checked.tiff')
plt.show()



plt.plot(depth_list_for_UC_MP, UC_time, label="Uniform Cost Search", color='g', linestyle = ":")
plt.plot(depth_list_for_UC_MP, MP_time, label="A* with Misplaced Tile Heuristic", color='b', linestyle = ":")
plt.plot(depth_list_for_Manha, Manha_time, label="A* with Manhattan Distance Heuristic", color='m', linestyle = ":")
plt.legend()
plt.xlabel("solution depth")
plt.ylabel("Searching Time (s)")
plt.title("Searching Time (s) - solution depth")
plt.ylim([0,3])
plt.grid()
plt.savefig('SearchingTime.tiff')
plt.show()

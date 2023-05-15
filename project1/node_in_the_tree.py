#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 11 13:28:20 2023

@author: cxiong
"""
import numpy as np
import heapq

## this class define each node in the search tree, every node can be the 
## state_matrix is a numpy matrix like. np.array([[1,2,3],[4,5,6],[7,0,8]])

##each node is defined as a class,
##each node class has the 2 dimensional state_matrix, refer to the state like np.array([[1,2,3],[4,5,6],[7,0,8]]).
##each node class has the g_n, refers to the g(n) value
##each node class has the h_n, refers to the h(n) value
##each node class has the predecessor, refers to the parent node
##each node class has a set of successor, refers to the son.
## In the heapq, the priority of nodes is ranked by the f(n), lower f(n) with higher priority.
## def __lt__(self, other): is doing the 'less than' ranking. 
class node_in_the_tree:
    def __init__(self, state_matrix, g_n=0, h_n=0, blank_tile_go=None, predecessor=None, successor=None):
        self.state_matrix = state_matrix
        self.g_n = g_n
        self.h_n = h_n
        self.blank_tile_go = blank_tile_go
        self.predecessor = predecessor
        self.successor = successor
        self.f = self.g_n + self.h_n
        
    def __lt__(self, other):
        return self.f < other.f
        
    
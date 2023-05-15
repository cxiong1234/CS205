#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 11 14:52:12 2023

@author: cxiong
"""
import numpy as np
import heapq
from node_in_the_tree import node_in_the_tree   ###class node_in_the_tree is a node in the search tree
import time


##get the location of the blank tile in a certain state matrix
def get_loc_of_blank_tile(state_matrix):
    indices = np.where(state_matrix == 0)
    row_idx = indices[0][0]
    column_idx = indices[1][0]
    return row_idx, column_idx


## for a certain states, get the location of blank, and figure out the possible diretion the blank can go.
## If the blank at border, it can not go further and pass the border
def blank_go_direction_option(state_matrix):
    ##get the row_index and column_index of the blank tile in a certain state
    row_idx, column_idx = get_loc_of_blank_tile(state_matrix)
    
    ## Initialize the direction option list
    option_list = np.array([])
    
    #  border range 0<row_idx<2
    if row_idx > 0:
        option_list = np.append(option_list, "blank_up")
    if row_idx < 2:
        option_list = np.append(option_list, "blank_down")
    
    #  border range 0<column_idx<2
    if column_idx > 0:
        option_list = np.append(option_list, "blank_left")
    if column_idx < 2:
        option_list = np.append(option_list, "blank_right")
    
    return option_list



def update_state_matrix(state_matrix, blank_go_direction):
    
    row_idx, column_idx = get_loc_of_blank_tile(state_matrix)   
    updated_state_matrix = state_matrix.copy()    
    ## if blank goes up, swap 0 element with the element on the top
    if blank_go_direction == "blank_up":
        tmp1 = updated_state_matrix[row_idx][column_idx]
        updated_state_matrix[row_idx][column_idx] = updated_state_matrix[row_idx-1][column_idx]
        updated_state_matrix[row_idx-1][column_idx] = tmp1
    ## if blank goes up, swap 0 element with the element on the bottom
    elif blank_go_direction == "blank_down":
        tmp2 = updated_state_matrix[row_idx][column_idx]
        updated_state_matrix[row_idx][column_idx] = updated_state_matrix[row_idx+1][column_idx]
        updated_state_matrix[row_idx+1][column_idx] = tmp2
    ## if blank goes up, swap 0 element with the element on the left
    elif blank_go_direction == "blank_left":
        tmp3 = updated_state_matrix[row_idx][column_idx]
        updated_state_matrix[row_idx][column_idx] = updated_state_matrix[row_idx][column_idx-1]
        updated_state_matrix[row_idx][column_idx-1] = tmp3
    ## if blank goes up, swap 0 element with the element on the right
    elif blank_go_direction == "blank_right":
        tmp4 = updated_state_matrix[row_idx][column_idx]
        updated_state_matrix[row_idx][column_idx] = updated_state_matrix[row_idx][column_idx+1]
        updated_state_matrix[row_idx][column_idx+1] = tmp4
    
    return updated_state_matrix

## function for calculating the misplaced tile heuristic h(n)
def misplaced_tile_h(state_matrix, goal_matrix):
    dimension = state_matrix.shape[0]
    dist = 0
    for i in range(dimension):
        for j in range(dimension):
            if state_matrix[i][j] == 0: ## doesnt include the blank tile
                continue
            elif state_matrix[i][j] != goal_matrix[i][j]:
                dist += 1    
    return dist


## function for calculating the manhattan distance heuristic h(n)
def cal_manhattan_h(state_matrix, goal_matrix): 
    dimension = state_matrix.shape[0]
    dist = 0
    for i in range(dimension):
        for j in range(dimension):
            if state_matrix[i][j] == 0: ##doesnt include the blank tile
                continue
            indices = np.where(goal_matrix == state_matrix[i][j]) #go through every element in the state_matrix find the corresponding
            goal_row_idx = indices[0][0]
            goal_column_idx = indices[1][0]
            dist = dist + abs(goal_row_idx - i) + abs(goal_column_idx - j)
    
    return dist



## uniform_misplaced_manhattan = 1, we use uniform cost method
## uniform_misplaced_manhattan = 2, we use A* with misplace tile
## uniform_misplaced_manhattan = 3, we used A* with manhattan distance
def implement_A_star(initial_state_matrix, goal_matrix, uniform_misplaced_manhattan):
    is_solu_exist = 0
    nodes_queue_list = []  ##Initialize the node queue which will be checked in the priority of smallest f(n).
    
    match uniform_misplaced_manhattan:
        case 1:
            print("#####Confirmed using uniform cost#####")
            heapq.heappush(nodes_queue_list, node_in_the_tree(initial_state_matrix))
        case 2:
            print("#####Confirmed using A* with Misplaced Tile heuristic#########")
            heapq.heappush(nodes_queue_list, node_in_the_tree(initial_state_matrix, h_n = misplaced_tile_h(initial_state_matrix, goal_matrix)))
        case 3:
            print("#####Confirmed using A* with manhattan heuristic#########")
            heapq.heappush(nodes_queue_list, node_in_the_tree(initial_state_matrix, h_n = cal_manhattan_h(initial_state_matrix, goal_matrix)))
    
    checked_node_states_list = set()
    
    
    ##Begin the loop
    while len(nodes_queue_list)>0:  ## If there are more than one node in the queue, we continue.
        node_ongoing = heapq.heappop(nodes_queue_list)
        checked_node_states_list.add(tuple(map(tuple, node_ongoing.state_matrix))) ##keep track of the checked nodes.
    
        if np.array_equal(goal_matrix, node_ongoing.state_matrix):  ### checking whether the states reach to the goal states.
            is_solu_exist = 1
            blanktile_move_history = []
            while node_ongoing.predecessor != None:  ## get all the previous blank tiles moves.
                blanktile_move_history.insert(0,node_ongoing.blank_tile_go)
                node_ongoing = node_ongoing.predecessor
            return  blanktile_move_history, len(checked_node_states_list), is_solu_exist
        
        ## initialze the differnce options of blank tile move for a certain current node.
        option_list = blank_go_direction_option(node_ongoing.state_matrix)
        
        ## go through all the branch nodes for a certain current node
        for option in option_list:
            updated_state_matrix = update_state_matrix(node_ongoing.state_matrix, option)
            
            
            if tuple(map(tuple, updated_state_matrix)) in checked_node_states_list:
                continue
            
            ##updating the current checking node.
            match uniform_misplaced_manhattan:
                case 1:
                    node_updating = node_in_the_tree(updated_state_matrix, g_n = node_ongoing.g_n+1, h_n = 0, blank_tile_go = option, predecessor = node_ongoing) 
                case 2:
                    node_updating = node_in_the_tree(updated_state_matrix, g_n = node_ongoing.g_n+1, h_n = misplaced_tile_h(node_ongoing.state_matrix, goal_matrix), blank_tile_go = option, predecessor = node_ongoing)
                case 3:
                    node_updating = node_in_the_tree(updated_state_matrix, g_n = node_ongoing.g_n+1, h_n = cal_manhattan_h(node_ongoing.state_matrix, goal_matrix), blank_tile_go = option, predecessor = node_ongoing)
        
            heapq.heappush(nodes_queue_list, node_updating)
    return [], [], 0
        



goal_state = np.array([
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
])

### Doing this to customize the method.
print("###Welcome to the 8-puzzle solver using A star ##############")
method = int(input("please input the method: \n 1 for Uniform Cost;\n 2 for A* with Misplaced Tile;\n 3 for A* with Manhattan Distance\n"))
customize_init_state = int(input("Tell us whether you prefer the default init state or your own customized state:\n 0 for default; \n 1 for customized\n"))

if customize_init_state == 0:
    difficulty =int(input("please input the difficulty you want by default initial states:\n 1 for easy; 2 for medium; 3 for hard; 4 for extremely hard\n"))
    
    match difficulty:
        case 1:
            initial_state = initial_state = np.array([[1, 2, 3], [4, 5, 6], [0, 7, 8]])
        case 2:
            initial_state = initial_state = np.array([[1, 3, 6], [5, 0, 2], [4, 7, 8]])
        case 3:  
            initial_state = initial_state = np.array([[1, 6, 7], [5, 0, 3], [4, 8, 2]])
        case 4:
            initial_state = initial_state = np.array([[0, 7, 2], [4, 6, 1], [3, 5, 8]])

else:
    raw_input = input("please input 9 elements as initial states of 8-puzzle, must be like 1 2 3 4 5 6 7 8 0,\n which present the first row, second row third row.\n")
    list = [int(i) for i in raw_input.split()]

    initial_state = np.array(list).reshape(3,3)


time_1 = time.time()
solution, num_of_checked_nodes, is_solu_exist = implement_A_star(initial_state, goal_state, uniform_misplaced_manhattan=method)
time_2 = time.time()

search_time = time_2 - time_1


## format output for the number of checked nodes and running time and step-by-step solution
if is_solu_exist:
    print("Number of checked nodes:", num_of_checked_nodes)
    print("###########################")
    print(f"The search took {search_time} seconds to run.")
    print("############################")
    print("solution depth:", len(solution))
    print("############################")
    print("Solution is:")
    if solution:
        for i, blank_move in enumerate(solution, start=1):
            print(f"step {i}. {blank_move}")
    else:
        print("Initial states is solution")
    #print(" -> ".join(solution))
else:
    print("No solution found.")


        
        
        
        
        
        
#!/usr/bin/env python
import rospy
from scipy.spatial import Delaunay
import numpy as np

# Kruskal's algorithm reference: http://alrightchiu.github.io/SecondRound/minimum-spanning-treekruskals-algorithm.html

class EMST():
    def __init__(self, laser, neighbor_size=30):
        # params settings
        self.n_size = neighbor_size  # create_graph_m_neighbor: neighbor size

        self.laser = laser
        
        self.complete_graph = []  # complete graph - tuples: [(E,Vi,Vj)...]

        ## test only
        # self.complete_graph = [(11,1,0),
        # (7,2,0),
        # (2,3,0),
        # (6,4,0),
        # (17,2,1),
        # (13,3,1),
        # (18,4,1),
        # (4,3,2),
        # (3,4,2),
        # (9,4,3)]

        self.result_tree = []  # minimun spanning tree - tuples: [(E,Vi,Vj)...] (sorted: edge non-decreasing)
  
        # laser specs
        self.vertices_num = self.laser.cartesian.shape[0]  # numbers of valid laser measurements
        self.arr = [-1] * self.vertices_num  # set represented by array (for kruskal to check whether a cycle is formed)

        # do methods
        #self.create_complete_graph()  # abandoned: not efficient
        # self.create_graph_m_neighbor()
        self.create_graph_delaunay()

        self.kruskals_mst()

    def create_graph_delaunay(self):
        points = self.laser.cartesian
        tri = Delaunay(points)
        for t in tri.simplices:
            for i,j in zip((t[0],t[1],t[2]),(t[1],t[2],t[0])): 
                vi = self.laser.cartesian[i]
                vj = self.laser.cartesian[j]
                euclidean_squared_edge = (vi[0] - vj[0])**2 + (vi[1] - vj[1])**2  # does not calculate squared root to make it faster
                self.complete_graph.append((euclidean_squared_edge, i, j))


    def create_graph_m_neighbor(self):
        ''' 
        Instead of creating a complete graph, we take the advantage of sequence of laser scan. 
        We only create graph between a vertex and its (self.n_size) neighbors.
        0   1   2   3   4
         ..
          .  .
           .   .
        0   1   2   3   4
        '''
        right_size = int(self.n_size/2)  # neighbor size on the right

        # for those i+1+right_size =< self.vertices_num
        for i in range(self.vertices_num - right_size):  
            for j in range(i+1, i+1+right_size):
                vi = self.laser.cartesian[i]
                vj = self.laser.cartesian[j]
                euclidean_squared_edge = (vi[0] - vj[0])**2 + (vi[1] - vj[1])**2  # does not calculate squared root to make it faster 
                self.complete_graph.append((euclidean_squared_edge, i, j))

        # for those i+1+right_size > self.vertices_num
        for i in range(self.vertices_num - right_size, self.vertices_num -1):  # last one ignored: everyone else has already connected to him
            for j in range(i+1, self.vertices_num):
                vi = self.laser.cartesian[i]
                vj = self.laser.cartesian[j]
                euclidean_squared_edge = (vi[0] - vj[0])**2 + (vi[1] - vj[1])**2  # does not calculate squared root to make it faster 
                self.complete_graph.append((euclidean_squared_edge, i, j))


    def create_complete_graph(self):
        '''
        Abandoned: not efficient
        '''
        for i in range(self.vertices_num -1):  # last one ignored: everyone else has already connected to him
            for j in range(i+1, self.vertices_num):
                vi = self.laser.cartesian[i]
                vj = self.laser.cartesian[j]
                euclidean_squared_edge = (vi[0] - vj[0])**2 + (vi[1] - vj[1])**2  # does not calculate squared root to make it faster 
                self.complete_graph.append((euclidean_squared_edge, i, j))


    def find_root(self, element):
        # element is a root
        if self.arr[element] < 0:  
            return element

        # element is not a root: find its root
        temp = element
        while self.arr[temp] >= 0:
            temp = self.arr[temp]

        # now temp is the root of element
        self.arr[element] = temp  # collapsing
        return temp

    def union_sets(self, root_a, root_b):
        rank_a = abs(self.arr[root_a])  # rank = numbers of elements in this set (represented by root root_a)
        rank_b = abs(self.arr[root_b])

        # union by rank 
        if rank_a >= rank_b:
            self.arr[root_a] = self.arr[root_a] + self.arr[root_b] 
            self.arr[root_b] = root_a
            
        else: 
            self.arr[root_b] = self.arr[root_b] + self.arr[root_a] 
            self.arr[root_a] = root_b

        # why we need to find_root: when unify two sets: does not update the root of the children of the set being merged

    def kruskals_mst(self):
        # sort by edge: non-decreasing
        self.complete_graph = sorted(self.complete_graph, key=lambda t:t[0])  # sort according to the first item in the tuple
        
        connected_edges = 0  # numbers of connected edges
        i = 0  # iterator 

        # number of edges needed = (vertice_num - 1)
        while connected_edges < (self.vertices_num - 1):
            e, va, vb = self.complete_graph[i]
            i += 1
            root_a = self.find_root(va)
            root_b = self.find_root(vb)

            if root_a == root_b:  # will form a cycle if connect va and vb: discard this edge
                continue
            
            # connect this edge
            self.result_tree.append((e, va, vb))
            connected_edges += 1
            self.union_sets(root_a, root_b)
            

# test only
# if __name__ == "__main__":
    
#     a = EMST()
#     b = a.result_tree
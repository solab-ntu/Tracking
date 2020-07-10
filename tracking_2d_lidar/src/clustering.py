#!/usr/bin/env python
import numpy as np

class Cluster():
    ''' Clustering based on Efficient Graph-Based Image Segmentation.
    Inputs:
        mst: minimum spanning tree as input graph
        k: k larger the clusters tends to merge together more
        min_size: minimum size of a cluster
    '''
    def __init__(self, mst, k=5.0, min_size=10):
        # EGBIS parameters
        self.k = k
        self.min_size = min_size

        self.vertices_num = mst.vertices_num
        self.input_graph = mst.result_tree  # [(e, v1, v2),...]

        # set
        self.set_arr = [-1] * self.vertices_num  # array representing set: positive number: its root; negative number: a root, abs() is the size 
        self.roots = []  # list of roots, same order as clusters
        self.clusters = []  # list of clusters. cluster: list of measurement index of laser.cartesian
        self.int_diff = [0] * self.vertices_num  # internal difference of a set (max diff in that set) 
        

        # class methods
        self.segmentation()


    def find_root(self, element):
        # element is a root
        if self.set_arr[element] < 0:  
            return element

        # element is not a root: find its root
        temp = element
        while self.set_arr[temp] >= 0:
            temp = self.set_arr[temp]

        # now temp is the root of element
        self.set_arr[element] = temp  # collapsing
        return temp

    def update_roots(self, element):
        '''
        Should only update everyone's root after the clustering is done.
        '''
        # element is a root
        if self.set_arr[element] < 0: 
            if abs(self.set_arr[element]) >= self.min_size:  # a cluster should be larger than min_size
                self.clusters.append([element]) 
                self.roots.append(element)
            return 

        # element is not a root: find its root
        temp = element
        while self.set_arr[temp] >= 0:
            temp = self.set_arr[temp]
        self.set_arr[element] = temp  # now temp is the root of element (collapsing)
        
        return 

    def gather_clusters(self, element):
        for cluster in self.clusters:
            if cluster[0] == self.set_arr[element]:  # cluster[0] is the root of this cluster
                cluster.append(element)

    def union_sets(self, root_a, root_b, e):
        rank_a = abs(self.set_arr[root_a])  # rank = numbers of elements in this set (represented by root root_a)
        rank_b = abs(self.set_arr[root_b])

        # union by rank 
        if rank_a >= rank_b:
            self.set_arr[root_a] = self.set_arr[root_a] + self.set_arr[root_b] 
            self.set_arr[root_b] = root_a

            self.int_diff[root_a] = e
            
        else: 
            self.set_arr[root_b] = self.set_arr[root_b] + self.set_arr[root_a] 
            self.set_arr[root_a] = root_b

            self.int_diff[root_b] = e


    def segmentation(self):
        for edge in self.input_graph:  # mst is already sorted by edge (non-decreasing)
            e, va, vb = edge
            
            # root represent a set
            root_a = self.find_root(va)
            root_b = self.find_root(vb)

            int_diff_a = self.int_diff[root_a]
            int_diff_b = self.int_diff[root_b]

            # r_a = self.k/float(abs(self.set_arr[root_a]))  # k/set_size
            # r_b = self.k/float(abs(self.set_arr[root_b]))  # k/set_size

            r_a = self.k  # k/set_size
            r_b = self.k  # k/set_size

            if root_a != root_b and e < min((int_diff_a + r_a), (int_diff_b + r_b)):
                # merge
                self.union_sets(root_a, root_b, e)

        # update every vertex's root, because when union_sets we did not update the root of those vertices in the set being merged
        for vertex in range(self.vertices_num):
            self.update_roots(vertex)

        # after update_roots, gather clusters
        for vertex in range(self.vertices_num):
            self.gather_clusters(vertex)

# # test only
# if __name__ == "__main__":
#     mst = EMST()
#     a = EGBIS(mst)
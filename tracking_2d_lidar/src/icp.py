import numpy as np
from sklearn.neighbors import NearestNeighbors

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''
    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    #assert src.shape == dst.shape  # commented by wuch

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001, max_dist=1.0):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
        max_dist: maximum distance allowed between matching points
    Output:
        A_matched: (N_matched)xm numpy array of source points that have matching points in B
        B_matched: (N_matched)xm numpy array of destination points that are matched by A. (A_matched[i] will match B_matched[i])
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    #assert A.shape == B.shape
    assert A.shape[1] == B.shape[1]  # modified by wuch

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))  # source
    dst = np.ones((m+1,B.shape[0]))  # destination
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0
    destination_indices = None

    iteration_count = 0

    for iteration in range(max_iterations):
        iteration_count += 1

        # find the nearest neighbors between the current source and destination points
        distances, dst_indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # remove outliers: process the output of nearest_neighbor, only being used to feed into best_fit_transform (added by wuch)
        count = 0
        for distance in distances:
            if distance <= max_dist*2.0/(iteration + 1.0):
                count += 1

        src_good = np.zeros((m, count))  # good means outliers removed
        dst_indices_good = np.zeros(count, dtype=int)
        src_indices_good = np.zeros(count, dtype=int)

        j = 0
        for i, distance in enumerate(distances):
            if distance <= max_dist*2.0/(iteration + 1.0):
                src_good[0][j] = src[0][i]
                src_good[1][j] = src[1][i]
                dst_indices_good[j] = dst_indices[i]
                src_indices_good[j] = i
                j += 1

            #else:  # this is an outlier
                #dst_indices[i] = -1

        # save for output
        destination_indices = dst_indices  # added by wuch

        dst_good = dst[:m,dst_indices_good]  
        # --- end of remove outliers

        # compute the transformation between the current source and nearest destination points
        #T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,dst_indices].T)
        T,_,_ = best_fit_transform(src_good.T, dst_good.T)  # modified by wuch

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:  # ternimate if error does not change much
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    # Matched A and B (A_matched[i] will match B_matched[i])  (added by wuch)
    #A_matched = A[src_indices_good,:]
    #B_matched = B[dst_indices_good,:]
    
    # --- test only (easy to plot: icp point-to-point)
    #return src_indices_good, dst_indices_good, T, distances, iteration

    # --- real work
    return destination_indices, T

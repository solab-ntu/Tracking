
### Pseudo code 
```python
def coarse_level_association(self, laser_now, laser_prev, clusters_now, clusters_prev, icp_max_dist):
        # cluster_now is not the first cluster
        if (laser_prev and clusters_prev) is not None:
            # ICP
            dst_indices, T = icp()
            matched_indices = match_clusters(dst_indices)

            # coarse-level association
            track_indices_new = []

            for each cluster:

                # coarse-level associated clusters (clusters that matched to previous)
                if matched:
                    track_i = self.x.matched_track_indices[match_i]
                    track_indices_new.append(track_i)  # save this track index

                    # match to a cluster that is static background
                    if matched to static background:
                        # to do: append to xb
                        pass
                    
                    # match to a cluster that is a track
                    elif matched to dynamic track:
                        # tentative
                        if still tentative:
                            xt_counter += 1

                        # check mature
                        if just become mature:
                            xt_counter = -1
                    
                            # to do: merge test with static background
                                # if merge succeeded:
                                    # remove_xt_xp(self, indices_new)
                                    # append xb
                                    # track_indices_new[-1] = -1  # static background
                            
                            # to do: merge test with dynamic tracks  # i think this can be ignored for now
                                # for every dynamic tracks:
                                    # if merge succeeded:
                                        # track_indices_new[-1] = the matched track's index   
                                    # if failed: 
                                        # do nothing because i am myself
                        # dynamic track fine-level data association

                # coarse-level unassociated clusters append tentative track
                elif not matched:
                    # append xt, xp
                    self.append_xt_xp(cluster, laser_now)
                    
                    # append xt_counter
                    self.x.xt_counter.append(1)

                    # append track_indices
                    last_index = len(self.x.xt) - 1
                    track_indices_new.append(last_index)

                    # append id
                    max_id = max(self.x.xt_id)
                    self.x.xt_id.append(max_id + 1)

            # to do: static back ground fine-level data association

            # remove xt, xp who is not matched by any cluster_now. update indices
            track_indices_new = self.remove_xt_xp(indices_new=track_indices_new)

            # update matched_track_indices
            self.x.matched_track_indices = track_indices_new

        # the first time to recieve a cluster
        else:  
            for i, cluster in enumerate(clusters_now.clusters):
                # init xt, xp
                self.append_xt_xp(cluster, laser_now)

                # init xt_counter
                self.x.xt_counter.append(1)

                # init track indices
                self.x.matched_track_indices.append(i)

                # init id
                self.x.xt_id.append(i)
```
import numpy as np

def remove_xt_xp(indices_new, xt):
        ''' Remove those tracks that their indices are "not" in indices_new.
        Input:
            indices_new: new track indices that cluster_now corresponds to.
        Output:
            indices_updated: since there are tracks removed, the indices need to be updated. e.g. [12,1,5,9,7] to [5,1,2,4,3]
        '''
        print('input')
        print(indices_new)
        # get the complementary indices 
        sorted_i = sorted(indices_new)
        total_i = np.arange(len(xt))
        mask = np.zeros(total_i.shape,dtype=bool)
        mask[sorted_i] = True
        rest_i = list(total_i[~mask])

        print('rest')
        print(rest_i)
        
        # delete tracks that their indices are not in indices_new
        for i in reversed(rest_i):  # delete from the back so that the index is correct
            del xt[i]

        print('xt')
        print(xt)

        # update indices_new
        # indices_updated = [0]*len(indices_new)
        # sorted_index = sorted(range(len(indices_new)), key=lambda k: indices_new[k])  # e.g. [2, 3, 1, 4, 5] to [2, 0, 1, 3, 4]
        # for i, s in enumerate(sorted_index):
        #     indices_updated[s] = i
        _index = 0
        for i in range(max(indices_new)+1):
            someone_matched_flag = False
            for _i, i_new in enumerate(indices_new):
                if i_new == i:
                    indices_new[_i] = _index
                    someone_matched_flag = True
            if someone_matched_flag:
                _index += 1


        print('output')
        print(indices_new)

if __name__ == "__main__":
    xt = range(15)
    remove_xt_xp([12,1,5,9,7,5],xt)
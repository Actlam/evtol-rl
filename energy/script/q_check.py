import pickle
import pprint
import numpy as np

with open('/home/lamcat/catkin_ws/src/energy/script/tmp_variables/q-table.binary', 'rb') as f:
    hoge = pickle.load(f)
    Q = hoge[0]
    # print(Q)
    for i in Q:
        nums = np.array(i)
        print(np.round(nums, 2))
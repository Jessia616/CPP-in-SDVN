import numpy as np

relist = [[1,2,3],[3,1,2],[2,3,1]]
re = np.array(relist)
for i in range(len(re[1, :])):
    d = re[:, i]
    max = np.sort(d)
    ind = np.argsort(d)
    print(max)
    print(ind)


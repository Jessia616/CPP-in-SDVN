import numpy as np
import numpy.linalg as la
import time

X = np.array([range(0, 500), range(500, 1000)])
m, n = X.shape

# 方法1：标准方法使用两层循环计算Dij
# 时间复杂度O(n*n)
t = time.time()
D = np.zeros([n, n])
for i in range(n):
    for j in range(i + 1, n):
        D[i, j] = la.norm(X[:, i] - X[:, j]) ** 2
        D[j, i] = D[i, j]
print(time.time() - t)

# 方法2：利用矩阵内积dot计算Dij
# 时间复杂度O(n*n)*O(m)
t = time.time()
D = np.zeros([n, n])
for i in range(n):
    for j in range(i + 1, n):
        d = X[:, i] - X[:, j]
        D[i, j] = np.dot(d, d)
        D[j, i] = D[i, j]
print(time.time() - t)

# 方法3：避免循环内点积运算，减少dot调用次数
# 时间复杂度O(n*n)
t = time.time()
G = np.dot(X.T, X)
D = np.zeros([n, n])
for i in range(n):
    for j in range(i + 1, n):
        D[i, j] = G[i, i] - G[i, j] * 2 + G[j, j]
        D[j, i] = D[i, j]
print(time.time() - t)

# 方法4：利用重复操作替代外部循环
t = time.time()
G = np.dot(X.T, X)
# 把G对角线元素拎出来，列不变，行复制n遍。
H = np.tile(np.diag(G), (n, 1))
D = H + H.T - G * 2
print(time.time() - t)
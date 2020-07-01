rect_list = []
draw_list = []


class Stack(object):
    """栈"""

    def __init__(self):
        self.items = []

    def is_empty(self):
        """判断是否为空"""
        return self.items == []

    def push(self, item):
        """加入元素"""
        self.items.append(item)

    def pop(self):
        """弹出元素"""
        return self.items.pop()

    def peek(self):
        """返回栈顶元素"""
        return self.items[self.items.__len__() - 1]

    def size(self):
        """返回栈的大小"""
        return int(self.items.__len__())


def getstack(adj, i, n, s):
    for j in range(n):
        if adj[i][j] == 1:
            s.push(j)
    return s.size()

n = 5
adj = [[0 for i in range(n)] for i in range(n)]
adj[0][1] = 1
adj[1][2] = 1
adj[0][2] = 1
adj[1][4] = 1
adj[3][4] = 1
adj[4][3] = 1
m = []
re = []
s = Stack()
count = 0
for i in range(n):
    if m.count(i) >= 1:
        continue
    else:
        m.append(i)
    result = getstack(adj, i, n, s)
    if result == 0:
        re.append([i])
        # print([i])
        count += 1
    else:
        relay = [i]
        while s.size() != 0:
            temp = s.peek()
            s.pop()
            if m.count(temp) == 0:
                relay.append(temp)
                m.append(temp)
                getstack(adj, temp, n, s)
        print(relay)
        for node in relay:
            draw_list.append(node)

        re.append(relay)
        count += 1
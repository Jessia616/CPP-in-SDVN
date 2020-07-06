# 通信节点文件写入

import Init

com_node_list = []
node_num = 496
com_node_list.extend(Init.get_communication_node(node_num-1))

with open('comnode.txt', 'w') as f:
    for i in com_node_list:
        a = ""
        a += str(i[0])
        a += ' '
        a += str(i[1])
        a += '\n'
        f.write(a)
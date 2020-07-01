# 转换移动轨迹文件格式

head = []
tail = []
with open('big_200.tcl', 'r') as f:
    for line in f:
        print(line)
        if line[2] == 'o':
            tail.append(line)
        else:
            head.append(line)

print('1')

with open('tiexi1.tcl', 'w') as f:
    for line in head:
        f.write(line)
    for line in tail:
        f.write(' ')
        f.write(line)



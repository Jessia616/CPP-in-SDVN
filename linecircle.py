
def judge(line_x, line_y, x, y, r):## 判断是否在圆内 (;´Д`) ​​​​
    if ((line_x - x) * (line_x - x) + (line_y - y) * (line_y - y) - r * r <= 0):
        return 1
    return 0


def Judis(line1_x, line1_y, line2_x, line2_y, x, y, r): ## 线段与圆的关系 (*ﾟ∇ﾟ) ​​​​
    if (judge(line1_x,line1_y,x,y,r) and judge(line2_x,line2_y,x,y,r)): ## 都在圆内 不相交
        return 1
    if  ((not judge(line1_x,line1_y,x,y,r)) and (judge(line2_x,line2_y,x,y,r))) or \
        ((judge(line1_x,line1_y,x,y,r)) and (not judge(line2_x,line2_y,x,y,r))): ## 一个圆内一个圆外 相交
        return 1
    A = 0.0
    B = 0.0
    C = 0.0
    dist1 = 0.0
    dist2 = 0.0
    angle1 = 0.0
    angle2 = 0.0
    if (line1_x == line2_x):
        A = 1.0
        B = 0.0
        C = line1_x * -1.0
    else:
        if (line1_y == line2_y):
            A = 0.0
            B = 1.0
            C = line1_y * -1.0
        else:
            A = line1_y - line2_y
            B = line2_x - line1_x
            C = line1_x * line2_y - line1_y * line2_x
    dist1 = A * x + B * y + C
    dist1 *= dist1
    dist2 = (A * A + B * B) * r * r
    if dist1 > dist2:
        return 0
    angle1 = (x - line1_x) * (line2_x - line1_x) + (y - line1_y) * (line2_y -line1_y)
    angle2 = (x - line2_x) * (line1_x - line2_x) + (y - line2_y) * (line1_y - line2_y)
    if (angle1 > 0 and angle2 > 0):
        return 1
    return 0

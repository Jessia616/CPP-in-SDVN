## new 长方形覆盖判断 ，有重叠返回1 (*ﾟ∇ﾟ) \
def isOverlap(minx0,miny0,maxx0,maxy0,minx1,miny1,maxx1,maxy1):
    if (maxx0  > minx1) and (maxx1  > minx0) and  (maxy0 > miny1) and (maxy1 > miny0):
        return 1
    else:
        return 0


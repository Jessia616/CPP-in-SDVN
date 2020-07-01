import Global_Par as Gp
import random

# 距离约接近最大范围 概率越小， 最低概率为90%
def ratio(dis):
   if dis <= Gp.com_dis/4:
       return 1
   else:
       y = -10/((8/9)*Gp.com_dis)*dis + 90 + (10*9)/8
       a = random.uniform(1,100)
       if a <= y:
           return 1
       else:
           return 0
   # return 1


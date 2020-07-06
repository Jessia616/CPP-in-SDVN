import random
a = [2,3,4,5,12,34,7,3,12]

max_list = max(a)
max_index = a.index(max(a))
GlobalMin = a[max_index]
GlobalParams = a[max_index]


print(max_list)
print(max_index)
print(GlobalMin)
print(GlobalParams)

b = sorted(range(len(a)), key=lambda k: a[k])

print(b)

b = [0,0,0]
a = [0,0,1]
for i in range(len(a)):
    b[i] = a[i]
b[1] = 1
print(b)
print(a)

for i in range(10):
    print(random.randint(0, 7))

b = [0,0,0]
a = [0,0,0]

if a==b:
    print(1)
else:
    print(0)

print('\n'.join([''.join([('Love'[(x-y) % len('Love')] if ((x*0.05)**2+(y*0.1)**2-1)**3-(x*0.05)**2*(y*0.1)**3 <= 0else' ') for x in range(-30, 30)]) for y in range(30, -30, -1)]))

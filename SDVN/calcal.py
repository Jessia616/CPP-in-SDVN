sum = 0
for i in range(1,9):
    sum += i*pow(0.3,i)*(1-0.3)
sum += 10*pow(0.3,10)
print(sum)
backoff = 2/4*((2-0.6-pow(0.6,10))/(1-0.6))
print(backoff)
tcont = 3/4*32 + backoff/2*13
print(tcont)
ar = 50
sr = 40
p = ar/sr

tq = p*(1-pow(p,15)-15*pow(p,14)*0)/(sr*(1-p)*(1-pow(p,16)))
print(tq)

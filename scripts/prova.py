#!/usr/bin/env python

import numpy as np

a=np.zeros((3,3),dtype=bool)
a[:2,:2]=True
b=np.where(a)
# b=np.array(zip(b[0],b[1]))
b=zip(b[0],b[1])
ps=[(0,0),(0,1)]
for p in ps:
    print(p)
    if p in b: print('yes')
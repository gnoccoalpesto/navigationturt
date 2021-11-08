#!/usr/bin/env python

import numpy as np

a=[(1,1),(1,2),(1,3),(1,4)]
ps=[(2,1),(1,1),(1,2),(2,3),(1,3)]
# b=[x for x in a if not x in ps]
b=a.remove(ps[0])
print(b)
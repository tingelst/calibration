# -*- coding: utf-8 -*-
"""
Created on Tue Jan 29 08:30:24 2013

@author: Lars
"""

from numpy import *
A = random.randn(100,2)*mat([[2,3,-1],[0,0,2]])
print(A.shape)
A = A + random.randn(100,3)/3.0
u,s,vh = linalg.linalg.svd(A)
v = vh.conj().transpose()
print v[:,-1] 
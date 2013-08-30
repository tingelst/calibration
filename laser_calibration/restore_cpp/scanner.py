import numpy as np

import sys
sys.path.append('./build')
import liblaser_scanner_py

cm = np.load('../cal_data/cm.npy').astype(np.float32)
dc = np.load('../cal_data/dc.npy').astype(np.float32)
rmat = np.load('../cal_data/c2w_rmat.npy').astype(np.float32)
tvec = np.load('../cal_data/c2w_tvec.npy').reshape(-1,1).astype(np.float32)
lplane = np.loadtxt('../cal_data/lplane.txt').reshape(-1,1).astype(np.float32)
lpoint = np.loadtxt('../cal_data/lpoint.txt').reshape(-1,1).astype(np.float32)

#cm = np.load('../cal_data/cm.npy')
#dc = np.load('../cal_data/dc.npy')
#rmat = np.load('../cal_data/c2w_rmat.npy')
#tvec = np.load('../cal_data/c2w_tvec.npy').reshape(-1,1)
#lplane = np.loadtxt('../cal_data/lplane.txt').reshape(-1,1)
#lpoint = np.loadtxt('../cal_data/lpoint.txt').reshape(-1,1)

ll = np.load('/home/lars/devel/calibration/laser_calibration/stepped_calibration_object/cal_cog_2013-05-28.npy')
ll = ll.astype(np.float32).reshape(-1,1)
#x_range =  np.arange(0, 2048)
#line = np.hstack((x_range, ll)).transpose().reshape(-1, 1)

#print(ll.shape)

scanner = liblaser_scanner_py.Scanner(cm, dc, rmat, tvec, lplane, lpoint)
import time
t1 = time.time()
n = 100
for i in range(n):
    p3ds = scanner.get_3d_coords(ll).astype(np.float32)
t2 = time.time()
time_diff = t2 - t1
fps = n / time_diff
print(fps)
print(time_diff)

#print(p3ds.dtype)
print(p3ds[1024])

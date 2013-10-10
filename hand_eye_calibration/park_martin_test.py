import numpy as np
import math3d as m3d

from aim_tool_pose_generator import AimToolPoseGenerator
from park_martin_calibration import ParkMartinCalibrator

def testsetup(N=100, p_noise=0.0005, r_noise=0.0005):
    """Return a synthetic set of flange and sensor poses. Symmetric,
    un-biased input and output noise is added to the positional
    (p_noise [m]) and rotational (r_noise [rad]) parts of the poses."""
    global sif_nom
    # ws = [m3d.Vector(*trpl) for trpl in [(0.0, 0.1, 0.0), (1.1,0,2.5), (-0.1, 1.5, -3.0), (-0.2, -1.0, 0.1), (2.0, 0.1, 1.1 )]]
    # os = [m3d.Orientation(w) for w in ws]
    # ps = [m3d.Vector(*trpl) for trpl in[(0,0,0), (0.1,0.2,0.15), (-0.3, -0.1, -0.3), (0.3, -0.2, 0.05), (0.4,0.0,-0.05)]]
    # // Some object to base transform, at an arbitrary orientation.
    oib=m3d.Transform()
    oib.pos=[2.2, 0.2, 0.8]
    oib.orient.rotateZ(np.pi/3)
    bio = oib.inverse()
    # // Some realistic sensor in flange transform
    sif = m3d.Transform()
    sif.pos.z = 0.4
    sif.pos.y = 0.2
    sif.orient.rotateX(np.pi/7)
    sif_nom = sif
    fis = sif.inverse()
    # // Realistic setup of aim pose generator for Nachi SC15F around home pose
    centre_pos = m3d.Vector([1.12, 0.0, 1.27])
    apg = AimToolPoseGenerator(centre_pos=centre_pos, aim_pos=oib.pos)
    fibs = [apg() for i in range(N)]
    # fibs=[bio*m3d.Transform(os[i],ps[i]) for i in xrange(len(ws))]
    sibs = [fib*sif for fib in fibs]
    sios = [bio*sib for sib in sibs]
    if p_noise != 0.0:
        for sio in sios:
            sio.pos += m3d.Vector(np.random.normal(0.0,p_noise,3))
        for fib in fibs:
            fib.pos += m3d.Vector(np.random.normal(0.0,p_noise,3))
    if r_noise != 0.0:
        for sio in sios:
            w = sio.orient.rotation_vector
            w += m3d.Vector(np.random.normal(0.0,r_noise,3))
            sio.orient=m3d.Orientation(w)
        for fib in fibs:
            w = fib.orient.rotation_vector
            w += m3d.Vector(np.random.normal(0.0,r_noise,3))
            fib.orient=m3d.Orientation(w)
            return np.array(zip(fibs,sios))



if __name__ == '__main__':
    pmc = ParkMartinCalibrator()
    n_pop = 1000
    fspps = testsetup(N=n_pop, p_noise=0.0005, r_noise=0.0005)
    ns_lims = (5,50)
    N = 400
    samples = np.empty((N, 7))
    for i in range(N):
        ns = np.random.randint(*ns_lims)
        #print('sample %d, size=%d' % (i,ns))
        pmc.pose_pairs = fspps[np.random.random_integers(0,n_pop-1,ns)]
        sif = pmc.sensor_in_flange
        samples[i] = [ns, pmc.pos_nai, pmc.pos_low_sing, sif.pos.dist(sif_nom.pos), pmc.orient_nai, pmc.orient_low_sing, sif.orient.ang_dist(sif_nom.orient)]

    np.save('park_martin_test_pose_pairs.npy', pmc.pose_pairs)
    from matplotlib import pyplot as pp
    (n,pnai,pls,perr,onai,ols,oerr)=samples.T


    def err_vs_nai():
        pp.scatter(pnai,perr, s=3, marker='+', color='blue')
        pp.scatter(onai,oerr, s=3, marker='x', color='red')
        pp.xlabel('noise amplification index')
        pp.ylabel('calibration error')
        pp.show()

    def err_vs_ls():
        pp.scatter(pls,perr, s=3, marker='+', color='blue')
        pp.scatter(ols,oerr, s=3, marker='x', color='red')
        pp.ylabel('calibration error')
        pp.xlabel('lowest singular value')
        pp.show()

    def err_vs_n():
        pp.scatter(n,perr, s=3, marker='+', color='blue')
        pp.scatter(n,oerr, s=3, marker='x', color='red')
        pp.xlabel('n_sample')
        pp.ylabel('calibration error')
        pp.show()

    def nai_vs_n():
        pp.scatter(n,pnai, s=3, marker='+', color='blue')
        pp.scatter(n,onai, s=3, marker='x', color='red')
        pp.xlabel('n_sample')
        pp.ylabel('noise amplification index')
        pp.show()

    def ls_vs_n():
        pp.scatter(n,pls, s=3, marker='+', color='blue')
        pp.scatter(n,ols, s=3, marker='x', color='red')
        pp.xlabel('n_sample')
        pp.ylabel('lowest singular value')
        pp.show()

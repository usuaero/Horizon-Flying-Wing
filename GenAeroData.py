from datetime import datetime as dt
from HorizonAircraft import scene, sceneDict, updateControls, updateState, bw, cbar, forcesOptions
# from multiprocessing import Pool, cpu_count
import numpy as np
import matplotlib.pyplot as plt
import ZachsModules as zm
import polyFits as pf
import machupX as mx

pbar2p = 2. * 54. / bw
qbar2q = 2. * 54. / cbar

def horizonForcesMoments(inp):
    # scene = mx.Scene(sceneDict)
    updateState(54., inp[14], inp[15], [inp[11]*pbar2p,
                                        inp[12]*qbar2q,
                                        inp[13]*pbar2p], scene)
    updateControls( [   inp[0],
                        (inp[1]+inp[ 6])/2.,
                        (inp[2]+inp[ 7])/2.,
                        (inp[3]+inp[ 8])/2.,
                        (inp[4]+inp[ 9])/2.,
                        (inp[5]+inp[10])/2.],
                    [   (inp[1]-inp[ 6])/2.,
                        (inp[2]-inp[ 7])/2.,
                        (inp[3]-inp[ 8])/2.,
                        (inp[4]-inp[ 9])/2.,
                        (inp[5]-inp[10])/2.],
                    scene)
    try:
        x = scene.solve_forces(**forcesOptions)['Horizon']['total']
        fm = [ x['Cx_s'], x['Cy_s'], x['Cz_s'], x['Cl_s'], x['Cm_s'], x['Cn_s'] ]
    except mx.exceptions.SolverNotConvergedError:
        fm = [None] * 6
    return (*inp, *fm)

def initializeCases(j):
    n = pf.decompose_j(j, Nvec)
    vals = [None] * 16
    for i in range(dofs):
        if i < 11:
            vals[i] = DEFL[n[i]]
        else:
            vals[i] = data[i-11][n[i]]
    i = -1
    while None in vals:
        vals[i] = 0.
        i -= 1
    return vals #.append(mx.Scene(sceneDict)))

N = 5
d = 20

DEFL = np.linspace(-d,d,N)

PBAR = np.linspace(-.2,.2,N)
QBAR = np.linspace(-.02,.02,N)
RBAR = np.linspace(-.15,.15,N)

AOA = np.linspace(-30,30,N)
BETA = np.linspace(-30,30,N)

data = [PBAR, QBAR, RBAR, AOA, BETA]

dofs = 9

J = N ** dofs
Nvec = [N-1] * dofs

# input(str(J))
it = [None]*J
prog = zm.io.Progress(J, title='Initializes Cases to Run')
with Pool() as pool:
    for i,ans in enumerate(pool.imap_unordered(initializeCases, range(J))):
        it[i] = ans
        prog.display()








'''
# if __name__ == '__main__':
n = 28
# prog = zm.io.Progress(n, title='Running cases')

# for i in range(1,J//n+2):
    # s = (i-1)*n
    # if i * n < J:
        # s, e = (i-1)*n, i*n
        # print((i-1)*n, i*n)
        # with Pool(8) as pool:
            # x = pool.map(f, it[(i-1)*n:i*n])
    # else:
        # print((i-1)*n,J)
        # s, e = (i-1)*n, J

for i in range(0,J,n):
    # with Pool(8) as pool:
        # if i+n > J:
            # x = pool.map(f, it[i:])
        # else:
            # x = pool.map(f, it[i:i+n])
    if i+n > J:
        with Pool(processes=6) as pool:
            x = pool.map(f, it[i:])
    else:
        with Pool(processes=6) as pool:
            x = pool.map(f, it[i:i+n])
    
    
    # obj = [k for k in x]
    
    
    ## write data
    print()
    print(x)
    input()
    ## progress bar update
    # prog.display()
'''

if __name__ == '__main__':
    
    fn = 'HorizonAerodynamicDatabase.csv'
    f = open(fn, 'w')
    f.write(zm.io.csvLineWrite( 'Center',
                                'R0',
                                'R1',
                                'R2',
                                'R3',
                                'R4',
                                'L0',
                                'L1',
                                'L2',
                                'L3',
                                'L4',
                                'PBAR',
                                'QBAR',
                                'RBAR',
                                'AOA',
                                'BETA',
                                'Cx_s',
                                'Cy_s',
                                'Cz_s',
                                'Cl_s',
                                'Cm_s',
                                'Cn_s' ) )
    f.close()
    
    
    zm.nm.runCases(horizonForcesMoments, it, fn, nBatch=3000, chunkSize=3, progKW={'title':'Running Cases: batches'})
    
    
    
    # nBatch = 3000
    # prog = zm.io.Progress(J//nBatch+1, title='Running Cases: batches')
    # for i in range(0,J,nBatch):
        
        # if i+nBatch>J:
            # n = J - i
            # x = [None] * n
            # with Pool() as pool:
                # for j,ans in enumerate(pool.imap_unordered(horizonForcesMoments, it[i:], 3)):
                    # x[j] = ans
        # else:
            # x = [None] * nBatch
            # with Pool() as pool:
                # for j,ans in enumerate(pool.imap_unordered(horizonForcesMoments, it[i:i+nBatch], 3)):
                    # x[j] = ans
        
        # zm.io.appendToFile(fn, *x, multiLine=True)
        
        # prog.display()
    
    
    
    
    # x = [None] * J
    # prog = zm.io.Progress(J, title='Running cases: solve and write')
    # with Pool() as pool:
        # for i,ans in enumerate(pool.imap_unordered(horizonForcesMoments, it, 3)):
            # x[i] = ans
            # zm.io.appendToFile(fn, *ans)
            # prog.display()
    
    
    
    # for line in x:
        # print(line)
        # input()
    
    # input()
    
    

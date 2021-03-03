from datetime import datetime as dt
from HorizonAircraft import scene, sceneDict, updateControls, updateState, bw, cbar, forcesOptions
from multiprocessing import Pool, cpu_count
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

it = [None]*J
prog = zm.io.Progress(J, title='Initializing {} dofs with {} spread each for a total of {} cases'.format(dofs, N, J))
with Pool() as pool:
    for i,ans in enumerate(pool.imap_unordered(initializeCases, range(J))):
        it[i] = ans
        prog.display()



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
    
    bat = 5000
    chu = 1
    
    zm.nm.runCases(horizonForcesMoments, it, fn, nBatch=bat, chunkSize=chu, progKW={'title':'Running Cases: {}/batch, {}/chunck'.format(bat,chu)})


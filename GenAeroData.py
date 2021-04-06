from datetime import datetime as dt
from HorizonAircraft import scene, sceneDict, updateControls, updateState, bw, cbar, forcesOptions
from multiprocessing import Pool, cpu_count
import numpy as np
import matplotlib.pyplot as plt
import ZachsModules as zm
import polyFits as pf
import machupX as mx
import sys

forcesOptions = {'body_frame':True, 'stab_frame':False, 'wind_frame':False,
                    'dimensional':False}#, 'verbose':True}

pbar2p = 2. * 54. / bw
qbar2q = 2. * 54. / cbar

def horizonForcesMoments(j):
    ## unpack inputs
    n = pf.decompose_j(j, Nvec)
    
    ## update the aircraft state
    updateState(54., AOA[n[14]], BETA[n[15]], [ PBAR[n[11]]*pbar2p,
                                                QBAR[n[12]]*qbar2q,
                                                RBAR[n[13]]*pbar2p], scene)
    r = [DEFL[i] for i in n[1: 6]]
    l = [DEFL[i] for i in n[6:11]]
    s = [(r[i]+l[i])/2. for i in range(5)]
    a = [(r[i]-l[i])/2. for i in range(5)]
    updateControls([DEFL[n[0]]]+s, a, scene)
    try:
        x = scene.solve_forces(**forcesOptions)['Horizon']['total']
        fm = [ x['Cx'], x['Cy'], x['Cz'], x['Cl'], x['Cm'], x['Cn'] ]
    except mx.exceptions.SolverNotConvergedError:
        fm = [None] * 6
    return (j, *fm)

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

N = 3
d = 20

DEFL = np.linspace(-d,d,N)

PBAR = np.linspace(-.2,.2,N)
QBAR = np.linspace(-.02,.02,N)
RBAR = np.linspace(-.15,.15,N)

AOA = np.linspace(-30,30,N)
BETA = np.linspace(-30,30,N)

data = [PBAR, QBAR, RBAR, AOA, BETA]

dofs = 16

J = N ** dofs
Nvec = [N-1] * dofs

if __name__ == '__main__':
    
    start = int(sys.argv[1])
    end   = int(sys.argv[2])
    it    = list(range(start,end+1))
    
    jid = str(dt.now()).replace(':','_').replace(' ','__')
    
    fn = 'LinearHorizonAerodynamicDatabase_start{}_end{}.csv'.format(start, end)
    f = open(fn, 'w')
    
    f.write(zm.io.csvLineWrite( 'J',
                                'Center',
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
                                'BETA' ) )
    f.write(zm.io.csvLineWrite( J, *[N]*16) )
    f.write('\n')
    
    f.write(zm.io.csvLineWrite( 'j',
                                'Cx',
                                'Cy',
                                'Cz',
                                'Cl',
                                'Cm',
                                'Cn' ) )
    
    f.close()
    
    bat = int(sys.argv[3])
    chu = int(sys.argv[4])
    
    zm.nm.runCases(horizonForcesMoments, it, fn, nBatch=bat, chunkSize=chu, progKW={'title':'Running Cases: {}/batch, {}/chunck'.format(bat,chu)})


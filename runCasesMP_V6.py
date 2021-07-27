import ZachsModules as zm
import numpy as np
import sys
from OptimizerMP import Coefficient, runOptimizationCase, scene, V, bw, cbar
from multiprocessing import Pool, cpu_count
import json
import polyFits as pf

mode2 = pf.polyFit('mode2_V3_F2', verbose=False)

def computeX0(CL, Cm, pbar):
    x = [CL, Cm, pbar]
    
    c = [mode2.evaluate(0, x)]
    s = [mode2.evaluate(i, x) for i in range(1,6)]
    a = [mode2.evaluate(i, x) for i in range(6,11)]
    aoa = [mode2.evaluate(11, x)]
    r = [None] * 5
    l = [None] * 5
    for i in range(5):
        r[i] = s[i] + a[i]
        l[i] = s[i] - a[i]
    return c + r + l + aoa

def mirror(X):
    x = list(X)
    c = x[0:1]
    r = x[1:6]
    l = x[6:11]
    a = x[11:]
    return c + l + r + a

def runStep(CL, Cm, r, c, x0, cnt):
    pbar = pbarRange[r]
    omega = [2. * V * pbar / bw, 0., 0.]
    Cn =  Cnrange[c]
    caseTimer = zm.io.Timer()
    sol = runOptimizationCase(CL, Cm, Cn, pbar, 0., 0., x0, cnt)
    
    fm = Coefficient(sol.x, scene, 'jac', setup, omega)
    CLerr = abs(CL-fm[1])
    Clerr = abs(Cl-fm[2])
    Cmerr = abs(Cm-fm[3])
    Cnerr = abs(Cn-fm[4])
    maxerr = max([CLerr,Clerr,Cmerr,Cnerr])
    caseTimer.stop()
    zm.io.appendToFile(fn, CL, Cl, Cm, Cn, pbar, 0., 0., sol.message, *fm, CLerr, Clerr, Cmerr, Cnerr, maxerr, *sol.x, sol.nit, caseTimer.length())
    
    if pbar != 0. and Cn != 0.:
        xMirror = mirror(sol.x)
        omega[0] *= -1.
        fmMirror = Coefficient(xMirror, scene, 'jac', setup, omega)
        CLerr = abs(CL-fmMirror[1])
        Clerr = abs(Cl-fmMirror[2])
        Cmerr = abs(Cm-fmMirror[3])
        Cnerr = abs(-Cn-fmMirror[4])
        maxerr = max([CLerr,Clerr,Cmerr,Cnerr])
        zm.io.appendToFile(fn, CL, Cl, Cm, -Cn, -pbar, 0., 0., sol.message, *fmMirror, CLerr, Clerr, Cmerr, Cnerr, maxerr, *xMirror, sol.nit, caseTimer.length())
    
    if maxerr <= 1e-7:
        return sol.x
    else:
        return None

def runPath(args):
    
    CL, Cm, path, X0, cnt = args
    
    x0 = np.copy(X0)
    success = 0
    ml = NCn
    end = ml - 1
    
    r = c = l + 1
    
    k = cnt
    if path == 1:
        r -= 1
        val = runStep(CL, Cm, r, c, x0, k)
        k += 1
        if type(val) != type(None):
            x0 = np.copy(val)
            success += 1
        temp = np.copy(x0)
        for i in range(1,l):
            val = runStep(CL, Cm, r-i, c+i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
        r -= 1
        val = runStep(CL, Cm, r, c, temp, k)
        k += 1
        if type(val) != type(None):
            x0 = np.copy(val)
            success += 1
        for i in range(1,l):
            val = runStep(CL, Cm, r-i, c+i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
        return success
    
    k += 2*l
    if path == 2:
        bb = []
        for i in range(1,l):
            val = runStep(CL, Cm, r-i, c+i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
            bb.append((r-i, c+i, x0[:]))
        for b in bb:
            r, c, x0 = b
            while c != end:
                c += 1
                val = runStep(CL, Cm, r, c, x0, k)
                k += 1
                if type(val) != type(None):
                    x0 = np.copy(val)
                    success += 1
        return success
    
    k += sum(range(l))
    if path == 3:
        bb = []
        for i in range(1,l):
            val = runStep(CL, Cm, r, c+i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
            bb.append((r, c+i, x0))
        for b in bb:
            r, c, x0 = b
            while c != end:
                r += 1
                c += 1
                val = runStep(CL, Cm, r, c, x0, k)
                k += 1
                if type(val) != type(None):
                    x0 = np.copy(val)
                    success += 1
        return success
    
    k += sum(range(l))
    if path == 4:
        for i in range(1,l):
            val = runStep(CL, Cm, r+i, c+i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
        return success
    
    k += l-1
    if path == 5:
        bb = []
        for i in range(1,l):
            val = runStep(CL, Cm, r+i, c, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
            bb.append((r+i, c, x0))
        for b in bb:
            r, c, x0 = b
            while r != end:
                r += 1
                c += 1
                val = runStep(CL, Cm, r, c, x0, k)
                k += 1
                if type(val) != type(None):
                    x0 = np.copy(val)
                    success += 1
        return success
    
    k += sum(range(l))
    if path == 6:
        bb = []
        for i in range(1,l):
            val = runStep(CL, Cm, r+i, c-i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
            bb.append((r+i, c-i, x0[:]))
        for b in bb:
            r, c, x0 = b
            while r != end:
                r += 1
                val = runStep(CL, Cm, r, c, x0, k)
                k += 1
                if type(val) != type(None):
                    x0 = np.copy(val)
                    success += 1
        return success
    
    k += sum(range(l))
    if path == 7:
        c -= 1
        val = runStep(CL, Cm, r, c, x0, k)
        k += 1
        if type(val) != type(None):
            x0 = np.copy(val)
            success += 1
        for i in range(1,l):
            val = runStep(CL, Cm, r+i, c-i, x0, k)
            k += 1
            if type(val) != type(None):
                x0 = np.copy(val)
                success += 1
        return success
    
    raise ValueError('path should be 1-7 not {}'.format(path))

setup = {   'Cx_s': {'valDes': 0.0, 'mult': -1.0},
            'Cz_s': {'valDes': 0.0, 'mult': -1.0},
            'Cl_s': {'valDes': 0.0, 'mult':  1.0},
            'Cm_s': {'valDes': 0.0, 'mult':  1.0},
            'Cn_s': {'valDes': 0.0, 'mult':  1.0}}

NCL = 5
CLrange = np.linspace(0.1, 0.9, NCL)

NCm = 11
Cmrange = np.linspace(-0.1, 0.1, NCm)

NCn = 33
Cnrange = np.linspace(-0.02, 0.02, NCn)

Npbar = NCn
pbarRange = np.linspace(-0.1, 0.1, Npbar)

Cl = rbar = qbar = 0.0

l = (NCn-1)//2
start = (l,l)
node = (l+1,l+1)
Nlat = sum(range(NCn+1))-l
Ncases = NCL * NCm * Nlat

nameData = 'M4_V6' #input('Enter name of data set: ')
fn = 'data_{}.csv'.format(nameData)

if __name__ == '__main__':
    zm.io.appendToFile(fn, 'CLDes', 'ClDes', 'CmDes', 'CnDes', 'pbar', 'qbar', 'rbar', 'status', 'CD', 'CL', 'Cl', 'Cm', 'Cn', 'CL error', 'Cl error', 'Cm error', 'Cn error', 'max error', 'Center', 'r00', 'r01', 'r02', 'r03', 'r04', 'l00', 'l01', 'l02', 'l03', 'l04', 'aoa', 'Niter', 'Duration')
    
    Nprog = NCL*NCm
    prog = zm.io.oneLineProgress(Nprog, msg='Setting up MultiProcessing')
    
    def mySetup(args):
        CL, Cm, cnt = args
        x0 = computeX0(CL, Cm, 0.)
        val = runStep(CL, Cm, *start, x0, cnt)
        cnt += 1
        if type(val) != type(None): x0 = val[:]
        val = runStep(CL, Cm, *node, x0, cnt)
        cnt += 1
        if type(val) != type(None): x0 = val[:]
        it = [None] * 7
        for path in range(1,8):
            it[path-1] = (CL, Cm, path, x0, cnt)
        return it
    
    cnt = 1
    i = 0
    IT = [None] * (NCL * NCm)
    for CL in CLrange:
        for Cm in Cmrange:
            
            IT[i] = (CL, Cm, cnt)
            i += 1
            cnt += Nlat
            
            prog.display()
    
    prog = zm.io.oneLineProgress(Nprog, msg='Computing starting nodes for lateral maneuvers of {}'.format(nameData))
    
    it = []
    with Pool(6) as pool:
        for i in pool.imap_unordered(mySetup, IT):
            it += i
            prog.display()
    
    successes = 0
    prog = zm.io.oneLineProgress(Nprog*7, msg='Computing lateral maneuvers of {}'.format(nameData))
    
    with Pool(6) as pool:
        for val in pool.imap_unordered(runPath, it):
            successes += val
            prog.display()
    
    zm.io.oneLineText('{:.3f}% success rate'.format(successes / Ncases * 100.))
    
    

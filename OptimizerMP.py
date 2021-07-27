import ZachsModules as zm
from HorizonAircraft import scene, V, bw, cbar, updateControls, updateState, mx, forcesOptions
from scipy.optimize import minimize
import numpy as np
import os


## default values
maxiter = 300
h = 1.e-6

def Coefficient(x, *args):
    '''
    x array is as follows:
    ======================
    index   name
    0       center
    
    1       right00
    2       right01
    3       right02
    4       right03
    5       right04
    
    6       left00
    7       left01
    8       left02
    9       left03
    10      left04
    
    11      aoa
    # 12      beta
    
    
    args tuple is as follows:
    =========================
    index   name
    0       scene object for mux
    1       which coefficient to solve for
    3       dictionary of the desired value for the coefficient and
            multiplier for the error of the true coef value and the desired value
    '''
    
    ## separate x
    right = x[1:6]
    left  = x[6:11]
    center = x[0]
    aoa = x[11]
    # beta = x[12]
    
    ## combine left and right to sym and asym
    symDefl = [center] + [(right[i]+left[i])/2. for i in range(5)]
    asymDefl =           [(right[i]-left[i])/2. for i in range(5)]
    
    ## separate args
    scene = args[0]
    coef = args[1]
    if coef != 'jac':
        val = args[2][coef]['valDes']
        mult = args[2][coef]['mult']
    omega = args[3]
    
    ## generate control functions and update scene
    updateControls(symDefl, asymDefl, scene)
    
    ## update state
    updateState(V, aoa, 0., omega, scene)
    
    try:
        fm = scene.solve_forces(**forcesOptions)['Horizon']['total']
    except mx.exceptions.SolverNotConvergedError:
        fm = {  'Cx_s': -1000,
                'Cy_s': 0,
                'Cz_s': -1000,
                'Cl_s': 1000,
                'Cm_s': 1000,
                'Cn_s': 1000}
    
    if coef != 'jac':
        ## solve for the desired coefficient
        return (fm[coef] - val) * mult
    else:
        
        # cfs = ['Cx_s', 'Cz_s', 'Cl_s'
        cfs = 'xzlmn'
        vals = np.zeros(5)
        for i in range(5):
            key = 'C'+cfs[i]+'_s'
            vals[i] = (fm[key] - args[2][key]['valDes']) * args[2][key]['mult']
        return vals

def Jacobian(x, *args):
    
    display.order.append(args[1])
    if args[1] == 'Cx_s':
        Args = (args[0], 'jac', args[2], args[3])
        ## compute display
        fm = Coefficient(x, *Args)
        cfs = 'xzlmn'
        dataStr = 'DLlmn'
        for i in range(5):
            key = 'C'+cfs[i]+'_s'
            data = fm[i] / Args[2][key]['mult'] + Args[2][key]['valDes']
            if i < 2:
                data *= -1.
            display.msg[i+1] = 'C{} = {:20.12e}'.format(dataStr[i], data)
        
        ## compute gradients
        l = 12
        grad = [None]*l
        Jacobian.CL = [None]*l
        Jacobian.Cl = [None]*l
        Jacobian.Cm = [None]*l
        Jacobian.Cn = [None]*l
        for i in range(l):
            temp = x[:]
            temp[i] += h
            fpos = Coefficient(x, *Args)
            temp[i] -= 2. * h
            fneg = Coefficient(x, *Args)
            grad[i]       = (fpos[0] - fneg[0]) / 2. / h
            Jacobian.CL[i] = (fpos[1] - fneg[1]) / 2. / h
            Jacobian.Cl[i] = (fpos[2] - fneg[2]) / 2. / h
            Jacobian.Cm[i] = (fpos[3] - fneg[3]) / 2. / h
            Jacobian.Cn[i] = (fpos[4] - fneg[4]) / 2. / h
        return grad
    elif args[1] == 'Cz_s':
        return Jacobian.CL
    elif args[1] == 'Cl_s':
        return Jacobian.Cl
    elif args[1] == 'Cm_s':
        return Jacobian.Cm
    elif args[1] == 'Cn_s':
        return Jacobian.Cn
    else:
        print(args[1])
        raise ValueError('Unknown parameter in Jacobian')
Jacobian.CL = [None]*12
Jacobian.Cl = [None]*12
Jacobian.Cm = [None]*12
Jacobian.Cn = [None]*12

def cost(x, *args):
    return sum(i**2 for i in x)

def costJac(x, *args):
    l = len(x)
    grad = [0.]*l
    for i in range(l):
        temp = x[:]
        temp[i] += h
        fpos = cost(temp)
        temp[i] -= 2.*h
        fneg = cost(temp)
        grad[i] = (fpos - fneg) / 2. / h
    
    Jacobian(x, *args)
    
    return grad

def display(x):
    display.cnt += 1
    
    flag = False
    cfs = 'xzlmn'
    i = len(display.order) - len(cfs)
    
    for j in range(len(cfs)):
        if display.order[i+j] != 'C'+cfs[j]+'_s': flag = True
    
    if flag:
        print('Improper order of jacobians')
        print(display.order)
        input()
    
    fm = [None]*5
    for i in range(5):
        fm[i] = display.msg[i+1][-20:]
    zm.io.appendToFile(display.fn, display.timer.length(), display.cnt, *x, *fm)
    
    display.msg[0] = 'Iter = {}'.format(display.cnt)
    # display.prog.display()
    
    display.order = []
display.cnt = 0
# display.data = [None]*5
display.order = []
display.msg = [' ']*6
display.timer = None
display.fn = None


def runOptimizationCase(CLDes, CmDes, CnDes, pbar, qbar, rbar, x0, caseNumber, maxiter=maxiter):
    
    workDir = os.getcwd()
    caseDir = workDir + '/caseHistory'
    if not os.path.exists(caseDir):
        os.mkdir(caseDir)
    display.fn = 'caseHistory/case{}.csv'.format(caseNumber)
    if os.path.isfile(display.fn):
        os.remove(display.fn)
    zm.io.appendToFile(display.fn, 'Duration', 'Iteration', 'Center', 'R0', 'R1', 'R2', 'R3', 'R4', 'L0', 'L1', 'L2', 'L3', 'L4', 'aoa', 'CD', 'CL', 'Cl', 'Cm', 'Cn')
    
    display.cnt = 0
    
    ## angular rate of the aircraft in stability frame [rad/sec]
    omega = [   2. * V * pbar / bw,
                2. * V * qbar / cbar,
                2. * V * rbar / bw]
        
    ## setup additional arguments
    setup = {   'Cx_s': {'valDes':    0.0, 'mult': -1.0},
                'Cz_s': {'valDes': -CLDes, 'mult': -1.0},
                'Cl_s': {'valDes':    0.0, 'mult':  1e5},
                'Cm_s': {'valDes':  CmDes, 'mult':  1e5},
                'Cn_s': {'valDes':  CnDes, 'mult':  1e5}}
    
    CDargs = (scene, 'Cx_s', setup, omega)
    CLargs = (scene, 'Cz_s', setup, omega)
    Clargs = (scene, 'Cl_s', setup, omega)
    Cmargs = (scene, 'Cm_s', setup, omega)
    Cnargs = (scene, 'Cn_s', setup, omega)
    
    ## setup constraints
    cons = ({'type': 'eq', 'fun': Coefficient, 'jac': Jacobian, 'args': CLargs},
            {'type': 'eq', 'fun': Coefficient, 'jac': Jacobian, 'args': Clargs},
            {'type': 'eq', 'fun': Coefficient, 'jac': Jacobian, 'args': Cmargs},
            {'type': 'eq', 'fun': Coefficient, 'jac': Jacobian, 'args': Cnargs})
    
    ## setup bounds
    bnds = tuple([(-20,20)] * 11 + [(None,None)])
    
    ## setup progress bar
    # display.prog = zm.io.Progress(maxiter+1, msg=[' ']*6, title='CL {:.3e},  Cm {:.3e},  Cn {:.3e},  pbar {:.3e},  rbar {:.3e}'.format(CLDes, CmDes, CnDes, pbar, rbar))
    
    display.timer = zm.io.Timer()
    
    ## initial conditions
    # x0 = np.zeros(12)
    
    sol = minimize(Coefficient, x0, args=CDargs, method='SLSQP',
                    jac = Jacobian,
                    bounds = bnds,
                    constraints = cons,
                    options = {'ftol': 1.e-8, 'disp': False, 'maxiter': maxiter},
                    callback=display)
    
    # display.prog.Set(maxiter+1)
    # display.prog.display()
    
    return sol

if __name__ == '__main__':
    
    CL = float(input('Enter desired CL: '))
    Cm = float(input('Enter desired Cm: '))
    Cn = float(input('Enter desired Cn: '))
    
    pbar = float(input('Enter desired pbar: '))
    qbar = float(input('Enter desired qbar: '))
    rbar = float(input('Enter desired rbar: '))
    
    
    neg = [0, 2, 4]
    pos = [1, 3]
    delpbar = np.array([6., 11., 15., 18., 20.])
    delCm = np.array([-5.]*5)
    delCL = np.array([0.,0.,0.,-2.5,-5.])
    
    def mode4Initializer(dL, dm, sgn=1):
        l = np.zeros(5)
        r = np.zeros(5)
        c = [-5.-5.*dm / 0.1]
        if sgn > 0.:
            r[neg] = -20.
            r[pos] = 20.
            l = delpbar - 5. + delCm * dm / 0.1 + delCL * dL / 0.5
        else:
            l[neg] = -20.
            l[pos] = 20.
            r = delpbar - 5. + delCm * dm / 0.1 + delCL * dL / 0.5
        l = list(l)
        r = list(r)
        return l[::-1] + c + r + [dL/0.5*8.]
    
    
    # x0 = mode4Initializer(CL, Cm, sgn=Cn)
    # print(x0)
    x0 = np.zeros(12)
    
    s = runOptimizationCase(CL, Cm, Cn, pbar, qbar, rbar, x0, 0)
    
    t = runOptimizationCase(CL, Cm, -Cn, -pbar, qbar, rbar, x0, 0)
    
    print(s)
    print()
    print(t)
    
    print()
    print(s.x)
    print(t.x)
    
    # center = float(input('Center: '))
    # r0     = float(input('    r0: '))
    # r1     = float(input('    r1: '))
    # r2     = float(input('    r2: '))
    # r3     = float(input('    r3: '))
    # r4     = float(input('    r4: '))
    # l0     = float(input('    l0: '))
    # l1     = float(input('    l1: '))
    # l2     = float(input('    l2: '))
    # l3     = float(input('    l3: '))
    # l4     = float(input('    l4: '))
    # aa     = float(input('    aa: '))
    
    # x = [center, r0, r1, r2, r3, r4, l0, l1, l2, l3, l4, aa]
    
    # setup = { 'Cx_s': {'valDes': 0.0, 'mult': -1.0},
                # 'Cz_s': {'valDes': 0.0, 'mult': -1.0},
                # 'Cl_s': {'valDes': 0.0, 'mult':  1e0},
                # 'Cm_s': {'valDes': 0.0, 'mult':  1e0},
                # 'Cn_s': {'valDes': 0.0, 'mult':  1e0}}
    
    
    
    # fm = Coefficient(x, scene, 'jac', setup)
    
    # cfs = ('CD', 'CL', 'Cl', 'Cm', 'Cn')
    
    # for i in range(5):
        # print('{:5s}    {}'.format(cfs[i], fm[i]))
    

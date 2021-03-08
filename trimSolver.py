from HorizonAircraft import scene, sceneDict, horizonDict, updateControls, updateState, Sw, bw, cbar, forcesOptions
from scipy.optimize import minimize
import ZachsModules as zm
from numpy import cos, sin, pi

omega = sceneDict['scene']['aircraft']['Horizon']['state']['angular_rates']

V = sceneDict['scene']['aircraft']['Horizon']['state']['velocity']
rho = sceneDict['scene']['atmosphere']['rho']
dynPress = rho * V ** 2 / 2

T0 = horizonDict['engines']['EDF']['T0']
T1 = horizonDict['engines']['EDF']['T1']
T2 = horizonDict['engines']['EDF']['T2']
thrustMax = T0 + V * (T1 + V * T2)
zOffset = horizonDict['engines']['EDF']['position'][2]


W = horizonDict['weight']
CW = W / dynPress / Sw



def engineFM(dt):
    if dt < 0 or dt > 1: raise ValueError('throttle setting out of range, {}'.format(thr))
    thrust = dt * thrustMax
    CT = thrust / dynPress / Sw
    CmEngine = zOffset * thrust / dynPress / Sw / cbar
    return CT, CmEngine

def boundActuators(sym, asym, d=20.):
    s = list(sym[1:])
    a = asym[:]
    for i in range(5):
        r = s[i] + a[i]
        l = s[i] - a[i]
        flag = False
        if r >  d:
            r =  d
            flag = True
        elif r < -d:
            r = -d
            flag = True
        if l >  d:
            l =  d
            flag = True
        elif l < -d:
            l = -d
            flag = True
        if flag:
            s[i] = (l+r)/2.
            a[i]  = (l-r)/2.
    return [sym[0]]+s, a

def Mode1(dl, dm):
    return boundActuators([20.*dm]*6, [20.*dl]*5)

'''
Definitions

x = [   dt,
        dm,
        aoa]
'''


def cost(x, separate=False):
    
    dt, dm, aoa = x
    
    updateState(V, aoa, 0., omega, scene)
    updateControls(*Mode1(0., dm), scene)
    
    fm = scene.solve_forces(**forcesOptions)['Horizon']['total']
    
    CT, CmEngine = engineFM(dt)
    
    aoaRad = aoa * pi / 180.
    
    xEr = (fm['Cx_s'] + CT * cos(aoaRad)) ** 2.
    zEr = (fm['Cz_s'] + CW - CT * sin(aoaRad)) ** 2.
    mEr = (fm['Cm_s'] + CmEngine) ** 2.
    
    if separate:
        return xEr, zEr, mEr
    
    return (xEr + zEr + mEr) * 100.


def display(x):
    display.cnt += 1
    
    xEr, zEr, mEr = cost(x, separate=True)
    
    display.prog.msg[1] = 'x_stab balance = {:20.12e}'.format(xEr**0.5)
    display.prog.msg[2] = 'y_stab balance = {:20.12e}'.format(zEr**0.5)
    display.prog.msg[3] = 'pitch balance  = {:20.12e}'.format(mEr**0.5)
    
    display.prog.msg[0] = 'Iter = {}'.format(display.cnt)
    display.prog.display()
display.cnt = 0
display.prog = None


## setup bounds
bnds = ((0,1), (-1,1), (None,None))

## setup initial x array
x0 = [0.5, -0.1, 3.]

## setup progress bar
maxiter = 200
display.prog = zm.io.Progress(maxiter+1, msg=[' ']*4, title='Horizon Trim Solver')

sol = minimize(cost, x0, method='SLSQP',
                jac = 'cs',
                bounds = bnds,
                options = { 'ftol': 1.e-15,
                            'disp': False,
                            'maxiter': maxiter,
                            'finite_diff_rel_step': 0.01},
                tol = 1.e-15,
                callback=display)

print(sol)

updateState(V, sol.x[2], 0., omega, scene)
updateControls(*Mode1(0., sol.x[1]), scene)

fm = scene.solve_forces(**forcesOptions)['Horizon']['total']
CT,cm = engineFM(sol.x[0])

print()
print(fm)
print(CT,cm)

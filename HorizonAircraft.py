import json
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import machupX as mx
import numpy as np

## constants
#####################################################################
#####################################################################
#####################################################################

## span frac locations
s_bay = 0.07560964056743188
s_taper = 0.1613005665438547
s_0 = 0.42047294 #0.32904045
s_1 = 0.65427572 #0.49678034
s_2 = 0.83982266 #0.66452023
s_3 = 0.95895113 #0.83226011

## optional quick reference keyword argument inputs
aircraft = {'aircraft': 'Horizon'}
sacs = {'control_state': {'sym': None, 'asym':None}}
sas  = {'state': {  "velocity": None,
                    "alpha": None,
                    # "beta": None,
                    "angular_rates": [None]*3,
                    "angular_rate_frame": "stab"}}
forcesOptions = {'body_frame':False, 'stab_frame':True, 'wind_frame':False,
                    'dimensional':False}#, 'verbose':True}

## readin in json files
f = open('HorizonAircraft.json', 'r')
horizonDict = json.load(f)
f.close()
f = open('HorizonScene.json', 'r')
sceneDict = json.load(f)
f.close()

## extract needed data
Sw = horizonDict['reference']['area']
bw = horizonDict['reference']['lateral_length']
cbar = horizonDict['reference']['longitudinal_length']
V = sceneDict['scene']['aircraft']['Horizon']['state']['velocity']

## useful functions
#####################################################################
#####################################################################
#####################################################################

def interpolate(x1,x2,x,y1,y2):
    return (y2-y1)/(x2-x1)*(x-x1)+y1

def isIterable(val):
    try:
        iter(val)
        return True
    except:
        return False

def chord(s):
    if s < 0. or s > 1.: raise ValueError('Unexpected value for span fraction')
    if s <= s_bay: return 2.75      ## bay chord is 2.75 ft
    if s >= s_taper: return interpolate(s_taper, 1., s,
                                1.1458333333333333, 0.9166666666666666)
    return -3.16641665e+00 + s * (1.85435652e+02 + s * (-1.80033656e+03 +
                                                        5.06167082e+03 * s))
def Chord(S):
    if isIterable(S):
        return np.array([chord(s) for s in S])
    else:
        return chord(S)

def generateControlFunctions(sd, ad):
    def sym(s, symDefl=sd):
        if s < 0. or s > 1.: raise ValueError('Unexpected value for span fraction')
        if s < s_bay: return symDefl[0]
        if s < s_taper: return 0.
        if s < s_0: return symDefl[1]
        if s < s_1: return symDefl[2]
        if s < s_2: return symDefl[3]
        if s < s_3: return symDefl[4]
        return symDefl[5]
    def asym(s, asymDefl=ad):
        if s < 0. or s > 1.: raise ValueError('Unexpected value for span fraction')
        if s < s_taper: return 0.
        if s < s_0: return asymDefl[0]
        if s < s_1: return asymDefl[1]
        if s < s_2: return asymDefl[2]
        if s < s_3: return asymDefl[3]
        return asymDefl[4]
    def Sym(S):
        if isIterable(S):
            return np.array([sym(s) for s in S])
        else:
            return sym(S)
    def Asym(S):
        if isIterable(S):
            return np.array([asym(s) for s in S])
        else:
            return asym(S)
    return Sym, Asym

def updateControls(symDefl, asymDefl, sc):
    ## generate control functions and update scene
    Sym, Asym = generateControlFunctions(symDefl, asymDefl)
    sacs['control_state'][ 'sym'] =  Sym
    sacs['control_state']['asym'] = Asym
    sc.set_aircraft_control_state(**sacs,**aircraft)

def updateState(V, aoa, beta, omega, sc):
    ## update state
    sas['state']['velocity'] = V
    sas['state']['alpha'] = aoa
    sas['state']['beta'] = beta
    sas['state']['angular_rates'] = omega
    sc.set_aircraft_state(**sas,**aircraft)

## update the input json files with necessary items
#####################################################################
#####################################################################
#####################################################################

## chord
horizonDict['wings']['wing']['chord'] = Chord

## horizon file
sceneDict['scene']['aircraft']['Horizon']['file'] = horizonDict

## control surfaces
Sym, Asym = generateControlFunctions([0.]*6, [0.]*5)
sceneDict['scene']['aircraft']['Horizon']['control_state'][ 'sym'] =  Sym
sceneDict['scene']['aircraft']['Horizon']['control_state']['asym'] = Asym



## MUX
#####################################################################
#####################################################################
#####################################################################
## create scene
scene = mx.Scene(sceneDict)

# updateState(54., 20., 0.,[0]*3, scene)
# updateControls([-5]*6, [0]*5, scene)
# fm = scene.solve_forces(**forcesOptions)['Horizon']['total']
# print(fm)

'''
                Setting the Control Surfaces
#####################################################################
#####################################################################
#####################################################################

Currently the control surfaces are all set to 0 degrees deflection. To set
the control surfaces, you can use the function 'updateControls'

updateControls( [symBay,  sym0,  sym1,  sym2,  sym3,  sym4],
                [        asym0, asym1, asym2, asym3, asym4],
                scene )

where 'sym' represents the elevator deflection of that servo and 'asym'
represents the aileron deflection of that servo. Note, that

abs(symI) + abs(asymI) <= 20.     for I = 0 to 4

This represents the deflection limits for flap deflection in the airfoil
database


                Setting the Aircraft State
#####################################################################
#####################################################################
#####################################################################

Currently the state is set to:
    velocity = 54.     ft/sec
    alpha = 0.
    beta = 0.
    omega = [0., 0., 0.]      [p, q, r]

To set the aircraft state you can use the function 'updateState'

updateState(V, aoa, beta, omega, scene)

'''


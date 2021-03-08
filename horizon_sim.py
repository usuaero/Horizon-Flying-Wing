## read in horizon input dictionary
from HorizonAircraft import horizonDict
import json
import pylot
from numpy import pi, sin, cos

## read in scene json file
f = open('HorizonScene.json', 'r')
simDict = json.load(f)
f.close()

## update the input json files with necessary items
#####################################################################
#####################################################################
#####################################################################

## sim aircraft
simDict['aircraft'] = {
    'name' : 'Horizon',
    'file' : horizonDict
}

## Add simulator arguments
simDict["simulation"] = {
    "real_time" : False,
    'timestep' : 0.001,
    "enable_graphics" : True,
    "simple_graphics" : True,
    "integrator" : "ABM4"
}

## Set initial state
V = 54.
a = 11.39897311 * pi / 180
simDict['aircraft']['initial_state'] = {
    "position" : [0.0, 0.0, -1000.0],
    "velocity" : [V*cos(a), 0., V*sin(a)],
    "orientation": [0., a*180/pi, 0.]
}

## Set state output
simDict['aircraft']['state_output'] = 'states.txt'

## Set controller
simDict["aircraft"]["controller"] = 'user-defined'


## run the sim
#####################################################################
#####################################################################
#####################################################################
sim = pylot.simulator.Simulator(simDict, verbose=True)
sim.run_sim()


'''
def interpolate(x1,x2,x,y1,y2):
    return (y2-y1)/(x2-x1)*(x-x1)+y1

def isIterable(val):
    try:
        iter(val)
        return True
    except:
        return False

f = open('HorizonAircraft.json', 'r')
horizonDict = json.load(f)
f.close()
f = open('HorizonScene.json', 'r')
input_dict = json.load(f)
f.close()

## extract needed data
V = input_dict['scene']['aircraft']['Horizon']['state']['velocity']
Sw = horizonDict['reference']['area']
bw = horizonDict['reference']['lateral_length']
cbar = horizonDict['reference']['longitudinal_length']

## update the chord function in the main wing of the aircraft
#####################################################################
def chord(s):
    if s < 0. or s > 1.: raise ValueError('Unexpected value for span fraction')
    
    s_bay = 0.07560964056743188
    c_bay = 2.75                ## ft
    if s <= s_bay: return c_bay
    
    s_taper = 0.1613005665438547
    
    c_tip = 0.9166666666666666      ## 11. / 12.
    c_taper = 1.1458333333333333     ## c_tip / 0.8
    
    if s >= s_taper: return interpolate(s_taper, 1., s, c_taper, c_tip)
    
    A, B, C, D = 5.06167082e+03, -1.80033656e+03,  1.85435652e+02, -3.16641665e+00
    
    return A * s**3. + B * s**2. + C * s + D

def Chord(S):
    if isIterable(S):
        return np.array([chord(s) for s in S])
    else:
        return chord(S)

horizonDict['wings']['wing']['chord'] = Chord

## update input_dict with horizon dict
#####################################################################
input_dict['aircraft'] = {
    'name' : 'Horizon',
    'file' : horizonDict
}

# Add simulator arguments
input_dict["simulation"] = {
    "real_time" : True,
    "enable_graphics" : True,
    "simple_graphics" : True,
    "integrator" : "ABM4"
}

# Set initial state
input_dict['aircraft']['initial_state'] = {
    "position" : [0.0, 0.0, -1000.0],
    "velocity" : [50.0, 0.0, 0.0],
}

# Set state output
input_dict['aircraft']['state_output'] = 'states.txt'

# Set controller
input_dict["aircraft"]["controller"] = 'user-defined'

sim = pylot.simulator.Simulator(input_dict, verbose=True)
sim.run_sim()
'''


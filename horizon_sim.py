## read in horizon input dictionary
from HorizonAircraft import horizonDict
import json
import pylot
from numpy import pi, sin, cos
import controlMapping as cm


## read in scene json file
f = open('HorizonScene.json', 'r')
simDict = json.load(f)
f.close()

## update the input json files with necessary items
#####################################################################
#####################################################################
#####################################################################

## set sim atmosphere
simDict['atmosphere'] = {
    'density': simDict['scene']['atmosphere']['rho']
}

## sim aircraft
simDict['aircraft'] = {
    'name' : 'Horizon',
    'file' : horizonDict
}

## Add simulator arguments
simDict["simulation"] = {
    "real_time" : False,
    'timestep' : 0.01,
    "enable_graphics" : True,
    "simple_graphics" : True,
    "integrator" : "ABM4"
}

## Set initial state
V = 54.
a = cm.mode2Trim(control=False) * pi / 180
simDict['aircraft']['initial_state'] = {
    "position" : [0.0, 0.0, -1000.0],
    "velocity" : [V*cos(a), 0., V*sin(a)],
    "orientation": [0., cm.mode2Trim(control=False), 0.]
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

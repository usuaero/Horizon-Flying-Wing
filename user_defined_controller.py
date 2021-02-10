import numpy as np
# import multiprocessing as mp
from pylot.controllers import BaseController
from HorizonAircraft import generateControlFunctions

def controlMappingMode1(dl, dm):
    d = 10.
    sym  = np.ones(6) * dm**3 * d
    asym = np.ones(5) * dl**3 * d
    return sym, asym


class UserDefinedController(BaseController):
    """
    Controller based on time only
    
    Parameters
    ----------
    control_dict : dict
        A dictionary of control names and specifications.
    """
    
    def __init__(self, control_dict, quit_flag, view_flag, pause_flag, data_flag, enable_interface, control_output):
        super().__init__(control_dict, quit_flag, view_flag, pause_flag, data_flag, enable_interface, control_output)
    
    def get_control(self, t, state_vec, prev_controls):
        """Returns the controls based on the inputted state and keyboard/joystick inputs.
        
        Parameters
        ----------
        t : float
            simulation time
        
        state_vec : list
            State vector of the entity being controlled.
        
        prev_controls : dict
            Previous control values.
        
        Returns
        -------
        controls : dict
            Dictionary of controls.
        """
        
        
        ## these controls can vary from -1 to 1 with 0 being no deflection
        delta_ell = 0.0
        delta_m = -0.3
        throttle = 0.2
        
        if t > 0.5:
            delta_ell = 1.0
        
        symDefl, asymDefl = controlMappingMode1(delta_ell, delta_m)
        
        Sym, Asym = generateControlFunctions(symDefl, asymDefl)
        
        return {'sym' : Sym, 'asym' : Asym, 'throttle' : throttle}


import numpy as np
# import multiprocessing as mp
from pylot.controllers import BaseController
from HorizonAircraft import generateControlFunctions
import controlMapping as cm

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
        
        thr, dl, dm = cm.mode1Trim()
        
        if thr > 1.: thr = 1.
        if thr < 0.: thr = 0.
        
        if dm >  1.: dm = 1.
        if dm < -1.: dm = -1.
        
        if dl >  1.: dl = 1.
        if dl < -1.: dl = -1.
        
        symDefl, asymDefl = cm.mode1(dl, dm)
        
        Sym, Asym = generateControlFunctions(symDefl, asymDefl)
        
        return {'sym' : Sym, 'asym' : Asym, 'throttle' : thr}


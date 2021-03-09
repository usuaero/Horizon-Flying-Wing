import numpy as np
# import multiprocessing as mp
from pylot.controllers import BaseController
from HorizonAircraft import generateControlFunctions

def controlMappingMode1(dl, dm):
    d = 10.
    sym  = np.ones(6) * dm**3. * d
    asym = np.ones(5) * dl**3. * d
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
        
        delta_ell = 0.0
        delta_m = -0.95750555
        throttle = 0.10104812
        
        if type(prev_controls['sym']) == float:
            delta_ell = 0.0
            delta_m = -0.95750555
            throttle = 0.10104812
        else:
            delta_m = abs(prev_controls['sym'](0.) / 10.) ** (1./3.)
            if prev_controls['sym'](0.) < 0.: delta_m *= -1.
            throttle = prev_controls['throttle']
            
            Vd = 54.
            zd = -1000.
            
            u,v,w = state_vec[:3]
            q = state_vec[4]
            z = state_vec[8]
            
            ev = Vd - (u**2.+v**2.+w**2.)**0.5
            ez = zd - z
            
            k_thr = 0.001
            k_ele = 0.00001
            k_q   = 0.001
            
            delta_m  += k_ele * ez + k_q * q
            throttle += k_thr * ev
        
        print('{:9.6f}  {:9.6f}'.format(delta_m, throttle))
        
        if throttle > 1.: throttle = 1.
        if throttle < 0.: throttle = 0.
        
        if delta_m > 1.: delta_m = 1.
        if delta_m < -1.: delta_m = -1.
        
        symDefl, asymDefl = controlMappingMode1(delta_ell, delta_m)
        
        Sym, Asym = generateControlFunctions(symDefl, asymDefl)
        
        return {'sym' : Sym, 'asym' : Asym, 'throttle' : throttle}


import inputs

import numpy as np
import multiprocessing as mp

from pylot.controllers import BaseController

def interpolate(x1,x2,x,y1,y2):
    return (y2-y1)/(x2-x1)*(x-x1)+y1

def isIterable(val):
    try:
        iter(val)
        return True
    except:
        return False

def controlMappingMode1(dl, dm):
    # sym = np.zeros(6)
    # asym = np.zeros(5)
    d = 10.
    sym  = np.ones(6) * dm**3 * d
    asym = np.ones(5) * dl**3 * d
    return sym, asym


class UserDefinedController(BaseController):
    """Zach's righteous awesome controller.

    Parameters
    ----------
    control_dict : dict
        A dictionary of control names and specifications.
    """

    def __init__(self, control_dict, quit_flag, view_flag, pause_flag, data_flag, enable_interface, control_output):
        super().__init__(control_dict, quit_flag, view_flag, pause_flag, data_flag, enable_interface, control_output)

        # Check for joystick
        self._avail_pads = inputs.devices.gamepads
        if len(self._avail_pads) == 0:
            raise RuntimeError("Couldn't find any joysticks!")
        elif len(self._avail_pads) > 1:
            raise RuntimeError("More than one joystick detected!")

        # Set off listener
        self._manager = mp.Manager()
        self._joy_def = self._manager.list()
        self._joy_def[:] = [0.0]*4
        self._throttle_perturbed = self._manager.Value('i', 0)
        self._trim_up_pressed = self._manager.Value('i', 0)
        self._trim_dn_pressed = self._manager.Value('i', 0)
        self._dn_cycles_held = 0
        self._up_cycles_held = 0
        self._trim_tab = 0.0
        self._joy_listener = mp.Process(target=joystick_listener, args=(self._joy_def, self._quit_flag, self._throttle_perturbed, self._trim_dn_pressed, self._trim_up_pressed))
        self._joy_listener.start()

        # Get mapping and limits
        self._axis_mapping = {}
        self._control_limits = {}
        self._angular_control = {} # True for angular deflection, False for 0 to 1.
        self._tied_to_trim_tab = {}
        for key, value in control_dict.items():

            # See if limits have been defined
            limits = value.get("max_deflection", None)
            if limits is not None: # The limits are defined
                self._control_limits[key] = limits
                self._angular_control[key] = True
            else:
                self._angular_control[key] = False
            
            # Get the mapping
            self._axis_mapping[key] = value["input_axis"]
            self._tied_to_trim_tab[key] = value.get("trim_tab", False)

        # Set variable for knowing if the user has perturbed from the trim state yet
        self._perturbed_set = False
        self._perturbed = False
        self._joy_init = [0.0]*4


    def get_control(self, t, state_vec, prev_controls):
        """Returns the controls based on the inputted state and keyboard/joystick inputs.

        Parameters
        ----------
        state_vec : list
            State vector of the entity being controlled.

        prev_controls : dict
            Previous control values.

        Returns
        -------
        controls : dict
            Dictionary of controls.
        """

        # Set perturbation condition
        if not self._perturbed_set:
            self._joy_init[:] = self._joy_def[:]
            self._perturbed_set = True

        # Check if we're perturbed from the start control set
        if not self._perturbed:
            if (np.array(self._joy_def) != np.array(self._joy_init)).any():
                self._perturbed = True
            else:
                return prev_controls # No point in parsing things if nothing's changed

        # Parse commands
        delta_ell, delta_m = self._joy_def[2], self._joy_def[3]
        throttle = (-self._joy_def[1]+1.0)*0.5

        # HERE IS WHERE YOU MAP INPUTS TO DEFLECTIONS
        symDefl, asymDefl = controlMappingMode1(delta_ell, delta_m)

        ## generate control functions and update scene
        Sym, Asym = self._generateControlFunctions(symDefl, asymDefl)
        return {'sym' : Sym, 'asym' : Asym, 'throttle' : throttle}


    def _generateControlFunctions(self, sd, ad):

        s_bay = 0.07560964056743188
        s_taper = 0.1613005665438547
        s_0 = 0.42047294 #0.32904045
        s_1 = 0.65427572 #0.49678034
        s_2 = 0.83982266 #0.66452023
        s_3 = 0.95895113 #0.83226011

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


def joystick_listener(axes_def, quit_flag, throttle_perturbed_flag, trim_dn_pressed, trim_up_pressed):
    """Listens to the joystick input and posts latest values to the manager list."""

    # While the game is still going
    while not quit_flag.value:

        # Wait for events
        events = inputs.get_gamepad()

        # Parse events
        try:
            for event in events:

                # Analog inputs
                if event.ev_type == 'Absolute':

                    # Roll axis
                    if event.code == 'ABS_X':
                        axes_def[0] = event.state/511.5-1.0

                    # Pitch axis
                    elif event.code == 'ABS_Y':
                        axes_def[1] = event.state/511.5-1.0

                    # Yaw axis
                    elif event.code == 'ABS_RZ':
                        axes_def[2] = event.state/127.5-1.0

                    # Throttle axis
                    elif event.code == 'ABS_THROTTLE':
                        if not throttle_perturbed_flag.value:
                            throttle_perturbed_flag.value = 1
                        axes_def[3] = event.state/127.5-1.0

                # Button inputs
                elif event.ev_type == "Key":

                    # Increase trim
                    if event.code == "BTN_TOP2":
                        trim_up_pressed.value = not trim_up_pressed.value

                    # Decrease trim
                    elif event.code == "BTN_THUMB2":
                        trim_dn_pressed.value = not trim_dn_pressed.value

        except BrokenPipeError:
            return

        except FileNotFoundError:
            return

        except ConnectionResetError:
            return

    return

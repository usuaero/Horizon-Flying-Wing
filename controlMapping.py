
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
            s[i] = (r+l)/2.
            a[i] = (r-l)/2.
    return [sym[0]]+s, a


def mode1(dl, dm, d=20.):
    return boundActuators([d*dm]*6, [d*dl]*5, d=d)

############################################################################
############################################################################
############################################################################

def mode1Trim(control=True):
    '''
    if control is true
        returns throttle, roll, pitch commands to trim Horizon in mode 1
    otherwise
        returns aoa (deg) for mode 1 trim condition
    '''
    if control: return 0.099875095666051114, 0.0, -0.431837400614613287
    return 11.266741443177069826


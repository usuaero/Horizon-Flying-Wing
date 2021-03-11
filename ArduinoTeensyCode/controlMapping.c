// I think these will likely need to be commented out on the actual arduino code
#include <studio.h>
#include <math.h>

// define all used functions

double boundActuators(double *sym, double *asym);
double mode1(double dl, double dm);

// define the global constants (they are all in uppercase for ease of identifying)

#define D 20.0                      // max deflection of any given control surface

// main function

int main() {
    
}

double boundActuators(double *sym, double*asym) {
    /*
     * sym  =  symmetric values in degrees [center, 0, 1, 2, 3, 4]
     * asym = asymmetric values in degrees [0,1,2,3,4]
     */
    double s[6], a[5], r, l;
    int i, flag;
    
    r = sym[0];
    if (r > D) {
        s[0] = D;
    } else if (r < -D) {
        s[0] = -D;
    } else s[0] = sym[0];
    
    for (i=0; i<5; i++) {
        r = sym[i+1] + asym[i];
        l = sym[i-1] - asym[i];
        flag = 0;
        if (r > D) {
            r = D;
            flag = 1;
        } else if (r < -D) {
            r = -D;
            flag = 1;
        }
        if (l > D) {
            l = D;
            flag = 1;
        } else if (l < -D) {
            l = -D;
            flag = 1;
        }
        if (flag) {
            s[i+1] = (

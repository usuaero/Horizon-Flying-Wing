#include <stdio.h>
#include <math.h>

// define all used functions

double left(double s, double a);
double right(double s, double a);
double bounds(double x);
double calc_dL(double V, double phi, double p, double climbRate);

void mode1(double dl, double dm, double *lr);

// define the global constants (they are all in uppercase for ease of identifying)
//##########################################################################

// constants
#define PI 3.14159265               // pi
#define G 32.174                    // acceleration due to gravity (ft/sec^2)
#define RHO 0.0020482               // density of air (slugs/ft^3)

#define D 20.0                      // max deflection of any given control surface (deg)
#define DL_MAX 0.9                  // max acceptable value for dL
#define DL_MIN 0.1                  // mIN acceptable value for dL
#define PSCT_GAIN 4.0               // tunable gain on the SCT criteria for the calcCL2 function
// aircraft properties
#define W 16.0                      // weight of aircraft (lbf)
#define S 12.29923797666569         // planform area of main wing (ft^2)
#define B 9.91936999529998          // wingspan (ft)

// main function

int main() {
    
    double deg[11], dl, dm;
    int i;
    
    while(1) {
        printf("\n\n\n=================================================\n\n\n");
        
        printf("\nEnter a value for dl:    ");
        scanf("%lf", &dl);
        printf("Enter a value for dm:  ");
        scanf("%lf", &dm);
        
        mode1(dl, dm, deg);
        
        printf("\nResults in degrees are:\n");
        for (i=0; i<5; i++) {
            printf("l%1d = %.6lf\n", 4-i, deg[i]);
        }
        printf("ce = %.6lf\n", deg[5]);
        for (i=0; i<5; i++) {
            printf("r%1d = %.6lf\n", i, deg[i+6]);
        }
        
        
    }
    
    
}

double left(double s, double a) {
    /*
     * s = symmetric value (deg)
     * a = asymmetric value (deg)
    */
    return s - a;
}

double right(double s, double a) {
    /*
     * s = symmetric value (deg)
     * a = asymmetric value (deg)
    */
    return s + a;
}

double bounds(double x) {
    /*
     * x = value in degrees
    */
    if (x < -D) x = -D;
    if (x >  D) x = D;
    return x;
}

double calc_dL(double V, double phi, double p, double climbRate, double theta) {
    /* 
     * inputs
     * V = airspeed (ft/sec)
     * phi = roll attitude, bank angle (radians)
     * p = actual roll rate of the aircraft, body frame (rad/sec)
     * climbRate = climb rate (ft/sec)
     * theta = pitch attitude, elevation angle (radians)
     * 
     * outputs
     * dL
    */
    
    double CL_sct, gamma, p_sct;
    double Omega, p_st_sct, r_st_sct, alpha;
    
    // calculate climb angle
    if (climbRate >= V) {
        gamma = PI/2.;
    }
    else if (climbRate <= -V) {
        gamma = -PI/2.;
    }
    else {
        gamma = asin(climbRate / V);
    }
    
    // calculate the expected roll rate for the steady coordinated turn
    if (V != 0. && fabs(phi) != PI/2.) {
        Omega = G * tan(phi) / V;
        p_st_sct = -sin(gamma) * Omega;
        r_st_sct = cos(phi) * cos(gamma) * Omega;
        
        
        alpha = (theta - gamma) / cos(phi);
        p_sct = cos(alpha) * p_st_sct - sin(alpha) * r_st_sct;
    }
    else {
        p_sct = 0.;
    }
    
    // check for non 0 airspeed
    if (V != 0.) {
        // check that roll rate is less than the expected steady coordinated turn roll rate
        if (fabs(p) <= p_sct * PSCT_GAIN) {
            // check that plane isn't knife edged
            if (fabs(phi) != PI/2.) {
                CL_sct = 2. * W * cos(gamma) / RHO / V / V / S / cos(phi); // Eq. (12)
            }
            else {
                return 0.;  // if plane is knife edged then return CL of 0
            }
        }
        else {
            CL_sct = 2. * W * cos(gamma) * cos(phi) / RHO / V / V / S;   // Eq. (14)
        }
    }
    else {
        return DL_MAX;  // return the max CL value if 0 airspeed
    }
    
    // Eq. (13), ensure that CL is bounded
    if (CL_sct > DL_MAX) {
        return DL_MAX;
    }
    else if (CL_sct < DL_MIN) {
        return DL_MIN;
    }
    else {
        return CL_sct;
    }
}


void mode1(double dl, double dm, double *lr) {
    double s[5], a[5];
    int i;
    
    for (i=0; i<5; i++) {
        s[i] = D * dm;
        a[i] = D * dl;
    }
    
    // set center control surface
    lr[5] = bounds(D * dm);
    // loop thru inboard to outboard control surfaces
    for (i=0; i<5; i++) {
        // set left control surface
        lr[4-i] = bounds( left(s[i], a[i]));
        // set right control surface
        lr[6+i] = bounds(right(s[i], a[i]));
    }
}


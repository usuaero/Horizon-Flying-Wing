/* controlMapping.h - library for the horizon control mapping functions
 * created by Zach Montgomery August 2021
*/

#ifndef controlMapping_h

#define controlMapping_h

//#include <stdio.h>
#include <math.h>

// define all used functions

double left(double s, double a);
double right(double s, double a);
double bounds(double x);
double calc_dL(struct telemetryData in);

double pwm2frac(int pwm);

void mode1(struct pilotCommands in, double *lr);
void mode2(struct pilotCommands pilot, double dL, double *lr);
void mode3(struct pilotCommands pilot, double CL1, double *lr);
void mode4(struct pilotCommands pilot, double CL1, double *lr);

// define the global constants (they are all in uppercase for ease of identifying)
//##########################################################################

// constants
// #define PI 3.14159265               // pi
#define G 9.81                      // acceleration due to gravity (m/sec^2)
#define RHO 0.78                    // density of air (kg/m^3)
// control parameters
#define D 20.0                      // max deflection of any given control surface (deg)
#define DL_MAX 0.9                  // max acceptable value for dL
#define DL_MIN 0.1                  // mIN acceptable value for dL
#define PSCT_GAIN 4.0               // tunable gain on the SCT criteria for the calcCL2 function
#define m1_pitchTrim 0.05           // % uptrim needed for mode 1
// aircraft properties
#define W 97.09                     // weight of aircraft (N)             ====THIS NEEDS TO BE UPDATED====
#define S 1.14                      // planform area of main wing (m^2)
#define B 9.91936999529998          // wingspan (ft)  currently not used
// transmitter values
#define TRANS_PWM_MIN 800.0
#define TRANS_PWM_MAX 2200.0
#define TRANS_PWM_NOM 1500.0
#define TRANS_PWM_NOISE 50.0

// custom struct types
struct telemetryData {
    double airspeed;
    double climbRate;
    double bankAngle;
    double elevationAngle;
    double rollRate;
    int time_msec;
};
struct pilotCommands {
    int ail;
    int ele;
    int rud;
    int modeSwitch;
    int baySwitch;
    int speed;
};

/* main function
int main() {
    
    double deg[11], dL;
    int i, dl, dm;
    
    while(1) {
        printf("\n\n\n=================================================\n\n\n");
        
        // printf("\nEnter a value for dL:    ");
        // scanf("%lf", &dL);
        printf("Enter a value for dl:    ");
        scanf("%d", &dl);
        printf("Enter a value for dm:    ");
        scanf("%d", &dm);
        
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
*/

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

double calc_dL(struct telemetryData in) {
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
    double V, phi, p, climbRate, theta;
    double CL_sct, gamma, p_sct;
    double Omega, p_st_sct, r_st_sct, alpha;
    
    // unpact input struct
    V           = in.airspeed;
    phi         = in.bankAngle;
    climbRate   = in.climbRate;
    p           = in.rollRate;
    theta       = in.elevationAngle;
    
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

double pwm2frac(int pwm) {
    if (pwm > int(TRANS_PWM_MAX + TRANS_PWM_NOISE) || pwm < int(TRANS_PWM_MIN - TRANS_PWM_NOISE)) return 0.0;
    if (pwm > int(TRANS_PWM_MAX)) return 1.0;
    if (pwm < int(TRANS_PWM_MIN)) return -1.0;
    return 2.0 * (double(pwm) - TRANS_PWM_MIN) / (TRANS_PWM_MAX - TRANS_PWM_MIN) - 1.0;
}

void mode1(struct pilotCommands in, double *lr) {
    double s[5], a[5], dl, dm;
    int i;
    
    dl = pwm2frac(in.ail);
    dm = pwm2frac(in.ele);
    
    if (dm >= 0.0) {
        dm = m1_pitchTrim + dm * (1.0 - m1_pitchTrim);
    } else {
        dm = m1_pitchTrim + dm * (1.0 + m1_pitchTrim);
    }
    
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

void mode2(struct pilotCommands pilot, double dL, double *lr) {
    double C, R[5], L[5];
    double CL0, CL1, CL2, CL3, CL4, CL5;
    double Cm0, Cm1, Cm2, Cm3, Cm4, Cm5, Cm6;
    double pbar0, pbar1, pbar2, pbar3, pbar4, pbar5, pbar6;
    int i;
    
    CL1   = dL;
    Cm1   = -pwm2frac(pilot.ele) * 0.3;
    pbar1 = -pwm2frac(pilot.ail) * 0.2;
    
    CL0 = 1.;
    CL2 = CL1 * CL1;
    CL3 = CL2 * CL1;
    CL4 = CL3 * CL1;
    CL5 = CL4 * CL1;
    
    Cm0 = 1.;
    Cm2 = Cm1 * Cm1;
    Cm3 = Cm2 * Cm1;
    Cm4 = Cm3 * Cm1;
    Cm5 = Cm4 * Cm1;
    Cm6 = Cm5 * Cm1;
    
    pbar0 = 1.;
    pbar2 = pbar1 * pbar1;
    pbar3 = pbar2 * pbar1;
    pbar4 = pbar3 * pbar1;
    pbar5 = pbar4 * pbar1;
    pbar6 = pbar5 * pbar1;
    // pbar7 = pbar6 * pbar1;
    
    C =
        CL0 * (Cm0 * (pbar0 * -4.189626264249384) +
               Cm1 * (pbar0 * -53.4710369330344 +
                      pbar2 * 0.6787362557310077) +
               Cm2 * (pbar0 * 49.436020580731935) +
               Cm3 * (pbar0 * 508.504915700918 +
                      pbar2 * -328.68672664041645) +
               Cm4 * (pbar0 * -1502.848158240465) +
               Cm5 * (pbar0 * -4275.51926815944) +
               Cm6 * (pbar0 * 12030.502025928508 +
                      pbar2 * 8878.66929869439)) +
        CL1 * (Cm0 * (pbar0 * -3.9489159156870794) +
               Cm1 * (pbar0 * 8.896571955979216 +
                      pbar2 * -139.2888297520571) +
               Cm2 * (pbar0 * 57.78851343666001) +
               Cm3 * (pbar0 * -562.2785876101645 +
                      pbar2 * -943.1375251662232) +
               Cm4 * (pbar0 * -453.29609484686995 +
                      pbar2 * -7447.955329124725) +
               Cm5 * (pbar0 * 5031.315053189061)) +
        CL2 * (Cm0 * (pbar0 * -1.4161716024894284 +
                      pbar2 * 8.098339459882538) +
               Cm2 * (pbar0 * 14.06042586758219) +
               Cm3 * (pbar0 * 207.85680750984062) +
               Cm4 * (pbar0 * 153.05424064675591 +
                      pbar2 * 8077.088993763857)) +
        CL3 * (Cm1 * (pbar2 * -16.23369594992078) +
               Cm3 * (pbar0 * -228.6429170798376));
    
    R[0] =
        CL0 * (Cm0 * (pbar0 * -0.49381628041800707 +
                      pbar1 * -23.86152206896132 +
                      pbar2 * -28.788953047046043 +
                      pbar4 * 2436.9375478929433) +
               Cm1 * (pbar0 * -7.231633069853711 +
                      pbar1 * 6.093844436789371 +
                      pbar2 * 59.853895524029056 +
                      pbar3 * -315.93387232435964 +
                      pbar6 * -136376.54036584115) +
               Cm2 * (pbar0 * -46.606306861214335 +
                      pbar1 * 148.52807844870702 +
                      pbar2 * 1561.8930377118409 +
                      pbar3 * -4407.706037486741 +
                      pbar4 * -40065.15086756209) +
               Cm3 * (pbar0 * -247.52569659539228 +
                      pbar1 * -878.963260300383 +
                      pbar6 * 2175358.1800112408) +
               Cm4 * (pbar0 * 1542.1821406002066 +
                      pbar1 * 2932.6943019276 +
                      pbar6 * -273832.1815157472) +
               Cm5 * (pbar0 * 1520.495839645387 +
                      pbar1 * 6614.552847362252 +
                      pbar2 * 29346.030515293227 +
                      pbar4 * -555633.253001025) +
               Cm6 * (pbar0 * -13309.063082550721 +
                      pbar1 * -36418.54653571196 +
                      pbar2 * -28193.2703337133)) +
        CL1 * (Cm0 * (pbar0 * -1.764620244417885 +
                      pbar2 * -194.16473865898942 +
                      pbar4 * -3780.0622706464665) +
               Cm1 * (pbar0 * -1.4479258017960444 +
                      pbar1 * 58.37910323208917 +
                      pbar2 * -1002.9697509895426 +
                      pbar3 * -250.27177384391905 +
                      pbar6 * 335616.1584052336) +
               Cm2 * (pbar0 * -53.374362289892865 +
                      pbar2 * 636.6663510320077 +
                      pbar3 * 16016.038934513466 +
                      pbar4 * 26954.38373523809) +
               Cm3 * (pbar0 * -84.09309294711399 +
                      pbar1 * 1180.9128455025775 +
                      pbar2 * -11770.20438528037 +
                      pbar3 * 24950.592649623388) +
               Cm4 * (pbar0 * -994.0702822968549 +
                      pbar2 * -52611.0913594739) +
               Cm5 * (pbar0 * -1919.668348882213) +
               Cm6 * (pbar0 * 20211.656555671074 +
                      pbar4 * 4363945.449042553 +
                      pbar6 * -77278693.82116672)) +
        CL2 * (Cm0 * (pbar0 * 14.691195464582654 +
                      pbar1 * -11.431467708970583 +
                      pbar2 * 555.535723294172 +
                      pbar3 * -4.780931337164024) +
               Cm1 * (pbar1 * -48.76162522030683 +
                      pbar2 * 1566.9031971476877 +
                      pbar4 * 17814.71748056042 +
                      pbar6 * -301255.2666678751) +
               Cm2 * (pbar0 * 601.9700264679915 +
                      pbar1 * -1714.4077083549748 +
                      pbar3 * 4715.300793672838 +
                      pbar4 * 48186.75201314303 +
                      pbar6 * 588316.5099608224) +
               Cm3 * (pbar0 * 64.64490458701226 +
                      pbar2 * 19154.57100262407 +
                      pbar3 * -16482.095012559857 +
                      pbar4 * -261827.32894133625) +
               Cm4 * (pbar0 * -6047.0592524462045 +
                      pbar1 * -742.0440340998118 +
                      pbar2 * 95356.35449157601) +
               Cm5 * (pbar0 * 13018.29851544245 +
                      pbar2 * -37998.86204111134) +
               Cm6 * (pbar0 * 10911.07728519133 +
                      pbar4 * -5947997.751949039)) +
        CL3 * (Cm0 * (pbar0 * -39.851303119682875 +
                      pbar1 * 47.6425743994775 +
                      pbar2 * -170.35086675345923 +
                      pbar4 * -2024.4989193338547 +
                      pbar6 * -4740.481820379482) +
               Cm1 * (pbar0 * 22.613800666234408 +
                      pbar1 * 135.70971609731558 +
                      pbar2 * -834.5696633598354) +
               Cm2 * (pbar0 * -190.675693760791 +
                      pbar1 * 2083.886822334427 +
                      pbar2 * 2129.1341955258085 +
                      pbar3 * -20265.581862302002) +
               Cm3 * (pbar1 * -8478.414967033925 +
                      pbar6 * 1119514.859980704) +
               Cm4 * (pbar0 * 548.8429419134944) +
               Cm5 * (pbar0 * -28074.7156973009 +
                      pbar1 * 5057.995921236469)) +
        CL4 * (Cm0 * (pbar0 * 34.87898342416234 +
                      pbar1 * -39.89111886023366) +
               Cm1 * (pbar0 * -36.38011630439493 +
                      pbar1 * -122.10495577987513 +
                      pbar2 * -257.0821405513501 +
                      pbar6 * -93583.13223067357) +
               Cm2 * (pbar2 * -31797.361326076465) +
               Cm3 * (pbar0 * 158.39826191300375 +
                      pbar1 * 7666.599106547989 +
                      pbar2 * -637.5443119059423 +
                      pbar3 * 4115.840685759143) +
               Cm4 * (pbar2 * 49255.77730083737) +
               Cm5 * (pbar0 * 16621.161844616247)) +
        CL5 * (Cm0 * (pbar0 * -10.302785414685543) +
               Cm2 * (pbar1 * -295.76198016334155 +
                      pbar2 * 20981.47197883458));
    
    R[1] =
        CL0 * (Cm0 * (pbar0 * -4.000820744966266 +
                      pbar1 * -36.36577031919767 +
                      pbar2 * 19.10727158813971) +
               Cm1 * (pbar0 * -31.445233878977447 +
                      pbar1 * 70.84984341582046) +
               Cm2 * (pbar0 * 19.150787351012674 +
                      pbar1 * 130.53598559009868 +
                      pbar2 * 1222.1528732467834 +
                      pbar3 * -2407.3830819931063) +
               Cm3 * (pbar1 * -2774.818594156077 +
                      pbar2 * -584.8080850144843 +
                      pbar3 * 4586.439519523275) +
               Cm4 * (pbar1 * 7098.130771956446 +
                      pbar2 * -58422.01158603406) +
               Cm5 * (pbar1 * 20296.83381486391) +
               Cm6 * (pbar1 * -71432.49825160296 +
                      pbar2 * 494626.02182091505)) +
        CL1 * (Cm0 * (pbar0 * 1.4565929758305551 +
                      pbar1 * -4.07672508811299 +
                      pbar2 * 63.15297857205181) +
               Cm1 * (pbar1 * 9.599374691923451) +
               Cm2 * (pbar0 * 35.3299269804735 +
                      pbar1 * -84.67426911587494 +
                      pbar2 * -4075.1120094292555) +
               Cm3 * (pbar0 * -27.994644659652003 +
                      pbar1 * 692.3515177152115) +
               Cm4 * (pbar2 * 93443.32084976182) +
               Cm5 * (pbar0 * 469.67079961106543 +
                      pbar3 * -27189.009881438025) +
               Cm6 * (pbar2 * -643330.920721617)) +
        CL2 * (Cm3 * (pbar0 * -186.72813385743152 +
                      pbar2 * 414.73154201614153) +
               Cm4 * (pbar0 * 70.99855312933622)) +
        CL3 * (Cm3 * (pbar0 * 178.40433088893056) +
               Cm4 * (pbar0 * -186.30241595598903));
    
    R[2] =
        CL0 * (Cm0 * (pbar0 * -4.090641380277683 +
                      pbar1 * -42.88389096240816 +
                      pbar2 * 36.02726676635025) +
               Cm1 * (pbar0 * -53.75889902162436 +
                      pbar1 * 23.722939247364664) +
               Cm2 * (pbar0 * 24.883249155781495 +
                      pbar1 * 49.17121119898719 +
                      pbar3 * 295.7538444201434) +
               Cm4 * (pbar3 * 7422.865066557291) +
               Cm5 * (pbar2 * 3843.716149927811 +
                      pbar3 * -8146.4085659928005) +
               Cm6 * (pbar2 * -42696.51960034265)) +
        CL1 * (Cm0 * (pbar0 * -2.6122428463734515) +
               Cm1 * (pbar0 * 6.528314824955782 +
                      pbar1 * 21.979404278480267) +
               Cm2 * (pbar0 * 41.34770202859743 +
                      pbar2 * -1547.1097718438002) +
               Cm3 * (pbar1 * -223.6026918177056) +
               Cm4 * (pbar2 * 18190.237868773267) +
               Cm5 * (pbar2 * -3666.054651460231));
    
    R[3] =
        CL0 * (Cm0 * (pbar0 * -3.5547151905794863 +
                      pbar1 * -39.4817202010882 +
                      pbar2 * 14.875842725639538) +
               Cm1 * (pbar0 * -69.0999637859658 +
                      pbar1 * -196.11169348253537 +
                      pbar2 * 394.05812163591514) +
               Cm2 * (pbar0 * -55.51092670879708 +
                      pbar1 * 167.4845821586497) +
               Cm3 * (pbar1 * 6534.46241607704 +
                      pbar3 * -7471.57204712554) +
               Cm4 * (pbar0 * 2208.339428247087 +
                      pbar2 * -1670.8204665256203 +
                      pbar3 * -46775.83938728828 +
                      pbar5 * 789599.9692835169) +
               Cm5 * (pbar1 * -41157.25732172187) +
               Cm6 * (pbar0 * -14089.203923537683 +
                      pbar1 * 8098.260471294522)) +
        CL1 * (Cm0 * (pbar0 * -6.348812167414701 +
                      pbar1 * -6.5110989992160775 +
                      pbar2 * 32.433460164614836) +
               Cm1 * (pbar0 * 11.559668202271967 +
                      pbar1 * 204.76525645057635 +
                      pbar2 * -239.4124099696351) +
               Cm2 * (pbar0 * 42.768952895447235) +
               Cm3 * (pbar1 * -2293.481280254594));
    
    R[4] =
        CL0 * (Cm0 * (pbar0 * -3.861365791315246 +
                      pbar1 * -31.679681051626876 +
                      pbar3 * -315.66351169251675 +
                      pbar4 * 690.3334868134291) +
               Cm1 * (pbar0 * -66.05975857963138 +
                      pbar1 * -54.84533987601278 +
                      pbar2 * 324.58125124816644) +
               Cm2 * (pbar0 * -172.1152242216155 +
                      pbar1 * 637.3442664273614 +
                      pbar3 * 4124.483846477122) +
               Cm3 * (pbar0 * 67.82530981900833 +
                      pbar1 * 4665.462643366804 +
                      pbar2 * 5131.472440655044 +
                      pbar3 * 32629.523111873463) +
               Cm4 * (pbar0 * 3676.3690572775786 +
                      pbar1 * -4948.173901555894 +
                      pbar2 * 25465.754917599468 +
                      pbar5 * 746143.1765036221) +
               Cm5 * (pbar1 * -33466.23218106391 +
                      pbar2 * -102689.00794231019 +
                      pbar3 * -478829.82439308654 +
                      pbar4 * 41174.82478018961) +
               Cm6 * (pbar0 * -12345.365852308982 +
                      pbar2 * -486747.8880302278 +
                      pbar3 * -391109.29802976095 +
                      pbar4 * 3223258.830212292)) +
        CL1 * (Cm0 * (pbar0 * -6.974461835101021 +
                      pbar1 * 15.225959064517644 +
                      pbar2 * -37.0803286258272 +
                      pbar4 * -2169.9040718062347) +
               Cm1 * (pbar0 * -2.5799550967171867 +
                      pbar1 * -19.55034056916778) +
               Cm2 * (pbar0 * 190.76117927856313 +
                      pbar2 * 207.61130617876964) +
               Cm3 * (pbar1 * -1064.7071614884787 +
                      pbar2 * -2361.214965298671) +
               Cm4 * (pbar0 * -2029.8821140413752) +
               Cm5 * (pbar2 * 81329.00263183903 +
                      pbar4 * 148123.97088241638) +
               Cm6 * (pbar1 * 26738.238839480688 +
                      pbar3 * 54546.59687769536)) +
        CL2 * (Cm0 * (pbar1 * -32.941104426542445 +
                      pbar2 * 107.41434139609362) +
               Cm1 * (pbar1 * 131.3814941972023 +
                      pbar2 * -199.1012237380543) +
               Cm2 * (pbar0 * -51.480875121483464) +
               Cm3 * (pbar1 * -933.3634823379546) +
               Cm4 * (pbar0 * 916.4696435645338 +
                      pbar1 * -7068.686034073414) +
               Cm5 * (pbar0 * -511.544768568816)) +
        CL3 * (Cm4 * (pbar1 * 5451.111975350806) +
               Cm5 * (pbar0 * 358.7342194648499));
    
    L[0] =
        CL0 * (Cm0 * (pbar0 * -0.49381625342101687 +
                      pbar1 * 23.861522068961495 +
                      pbar2 * -28.78895759287419 +
                      pbar4 * 2436.9375842280197) +
               Cm1 * (pbar0 * -7.231633110851398 +
                      pbar1 * -6.0938444367895155 +
                      pbar2 * 59.853921561821835 +
                      pbar3 * 315.93387232434725 +
                      pbar6 * -136376.52703773172) +
               Cm2 * (pbar0 * -46.60630922299243 +
                      pbar1 * -148.52807844873107 +
                      pbar2 * 1561.8934831189456 +
                      pbar3 * 4407.706037486672 +
                      pbar4 * -40065.15462513674) +
               Cm3 * (pbar0 * -247.52574440594523 +
                      pbar1 * 878.963260300374 +
                      pbar6 * 2175358.010689285) +
               Cm4 * (pbar0 * 1542.1819851890396 +
                      pbar1 * -2932.69430192682 +
                      pbar6 * -273832.00287296175) +
               Cm5 * (pbar0 * 1520.4964436281723 +
                      pbar1 * -6614.552847362162 +
                      pbar2 * 29346.025659071478 +
                      pbar4 * -555633.1886589839) +
               Cm6 * (pbar0 * -13309.06089495166 +
                      pbar1 * 36418.54653570577 +
                      pbar2 * -28193.316761107766)) +
        CL1 * (Cm0 * (pbar0 * -1.764620443398935 +
                      pbar2 * -194.1647312296036 +
                      pbar4 * -3780.0623139785544) +
               Cm1 * (pbar0 * -1.4479225208880768 +
                      pbar1 * -58.379103232087814 +
                      pbar2 * -1002.9699652523267 +
                      pbar3 * 250.27177384397817 +
                      pbar6 * 335616.1250818997) +
               Cm2 * (pbar0 * -53.37429835152778 +
                      pbar2 * 636.6635015052116 +
                      pbar3 * -16016.038934513133 +
                      pbar4 * 26954.397207273094) +
               Cm3 * (pbar0 * -84.09302867182599 +
                      pbar1 * -1180.9128455023874 +
                      pbar2 * -11770.204029596503 +
                      pbar3 * -24950.592649618215) +
               Cm4 * (pbar0 * -994.0709163744227 +
                      pbar2 * -52611.053218117915) +
               Cm5 * (pbar0 * -1919.6692543789488) +
               Cm6 * (pbar0 * 20211.654603172665 +
                      pbar4 * 4363944.365418564 +
                      pbar6 * -77278702.25602296)) +
        CL2 * (Cm0 * (pbar0 * 14.691195408896478 +
                      pbar1 * 11.431467708970672 +
                      pbar2 * 555.5357630938114 +
                      pbar3 * 4.780931337164553) +
               Cm1 * (pbar1 * 48.761625220192144 +
                      pbar2 * 1566.9037116592913 +
                      pbar4 * 17814.714228500685 +
                      pbar6 * -301255.22414448083) +
               Cm2 * (pbar0 * 601.9699688155022 +
                      pbar1 * 1714.4077083549378 +
                      pbar3 * -4715.300793672649 +
                      pbar4 * 48186.73550695937 +
                      pbar6 * 588316.5546421078) +
               Cm3 * (pbar0 * 64.64488817515348 +
                      pbar2 * 19154.572203058517 +
                      pbar3 * 16482.095012545364 +
                      pbar4 * -261827.30312736685) +
               Cm4 * (pbar0 * -6047.058528591181 +
                      pbar1 * 742.0440340999435 +
                      pbar2 * 95356.32765353871) +
               Cm5 * (pbar0 * 13018.298037467037 +
                      pbar2 * -37998.86552457678) +
               Cm6 * (pbar0 * 10911.07765678432 +
                      pbar4 * -5947995.729944356)) +
        CL3 * (Cm0 * (pbar0 * -39.85130260433661 +
                      pbar1 * -47.64257439947828 +
                      pbar2 * -170.35092233501763 +
                      pbar4 * -2024.4986377091172 +
                      pbar6 * -4740.48279409155) +
               Cm1 * (pbar0 * 22.61378344024334 +
                      pbar1 * -135.7097160970378 +
                      pbar2 * -834.5698887444478) +
               Cm2 * (pbar0 * -190.67570100937564 +
                      pbar1 * -2083.88682233441 +
                      pbar2 * 2129.136519686903 +
                      pbar3 * 20265.581862301442) +
               Cm3 * (pbar1 * 8478.414967032953 +
                      pbar6 * 1119514.770504343) +
               Cm4 * (pbar0 * 548.8429977006542) +
               Cm5 * (pbar0 * -28074.714654271073 +
                      pbar1 * -5057.9959212359245)) +
        CL4 * (Cm0 * (pbar0 * 34.87898219647987 +
                      pbar1 * 39.89111886023393) +
               Cm1 * (pbar0 * -36.38010194001221 +
                      pbar1 * 122.10495577970278 +
                      pbar2 * -257.08224862709574 +
                      pbar6 * -93583.1019846003) +
               Cm2 * (pbar2 * -31797.35874604538) +
               Cm3 * (pbar0 * 158.39829300359298 +
                      pbar1 * -7666.599106547128 +
                      pbar2 * -637.5454485331843 +
                      pbar3 * -4115.840685749586) +
               Cm4 * (pbar2 * 49255.76374289204) +
               Cm5 * (pbar0 * 16621.16128943862)) +
        CL5 * (Cm0 * (pbar0 * -10.302784374504633) +
               Cm2 * (pbar1 * 295.7619801633382 +
                      pbar2 * 20981.46988709383));
    
    L[1] =
        CL0 * (Cm0 * (pbar0 * -4.000820781029116 +
                      pbar1 * 36.36577031919776 +
                      pbar2 * 19.107272805203845) +
               Cm1 * (pbar0 * -31.445233926451582 +
                      pbar1 * -70.84984341581628) +
               Cm2 * (pbar0 * 19.15078745751292 +
                      pbar1 * -130.53598559011294 +
                      pbar2 * 1222.1526669521231 +
                      pbar3 * 2407.3830819931363) +
               Cm3 * (pbar1 * 2774.818594156019 +
                      pbar2 * -584.8080761428724 +
                      pbar3 * -4586.439519523384) +
               Cm4 * (pbar1 * -7098.130771955901 +
                      pbar2 * -58422.00300457003) +
               Cm5 * (pbar1 * -20296.83381486425) +
               Cm6 * (pbar1 * 71432.49825159842 +
                      pbar2 * 494625.9460933299)) +
        CL1 * (Cm0 * (pbar0 * 1.456593038117167 +
                      pbar1 * 4.076725088112872 +
                      pbar2 * 63.15297642361786) +
               Cm1 * (pbar1 * -9.599374691937285) +
               Cm2 * (pbar0 * 35.32992583786469 +
                      pbar1 * 84.6742691158785 +
                      pbar2 * -4075.1116671322643) +
               Cm3 * (pbar0 * -27.99464611898376 +
                      pbar1 * -692.351517714993) +
               Cm4 * (pbar2 * 93443.30789244133) +
               Cm5 * (pbar0 * 469.67067938392665 +
                      pbar3 * 27189.009881439797) +
               Cm6 * (pbar2 * -643330.8101574526)) +
        CL2 * (Cm3 * (pbar0 * -186.728104914801 +
                      pbar2 * 414.73145594408766) +
               Cm4 * (pbar0 * 70.99853803335097)) +
        CL3 * (Cm3 * (pbar0 * 178.4043119706791) +
               Cm4 * (pbar0 * -186.30237967821486));
    
    L[2] =
        CL0 * (Cm0 * (pbar0 * -4.09064142804747 +
                      pbar1 * 42.88389096240798 +
                      pbar2 * 36.02726993726974) +
               Cm1 * (pbar0 * -53.758899408130056 +
                      pbar1 * -23.72293924736626) +
               Cm2 * (pbar0 * 24.883247789172778 +
                      pbar1 * -49.17121119898163 +
                      pbar3 * -295.75384442013393) +
               Cm4 * (pbar3 * -7422.865066556714) +
               Cm5 * (pbar2 * 3843.715678195002 +
                      pbar3 * 8146.408565993857) +
               Cm6 * (pbar2 * -42696.52381947725)) +
        CL1 * (Cm0 * (pbar0 * -2.6122428342016155) +
               Cm1 * (pbar0 * 6.528315442995924 +
                      pbar1 * -21.97940427848188) +
               Cm2 * (pbar0 * 41.34770506182145 +
                      pbar2 * -1547.1099384024265) +
               Cm3 * (pbar1 * 223.6026918177485) +
               Cm4 * (pbar2 * 18190.23967068455) +
               Cm5 * (pbar2 * -3666.0540028098594));
    
    L[3] =
        CL0 * (Cm0 * (pbar0 * -3.554715135312482 +
                      pbar1 * 39.48172020108803 +
                      pbar2 * 14.875839517927238) +
               Cm1 * (pbar0 * -69.09996223793803 +
                      pbar1 * 196.1116934825445 +
                      pbar2 * 394.05806085445283) +
               Cm2 * (pbar0 * -55.51092644712947 +
                      pbar1 * -167.48458215864343) +
               Cm3 * (pbar1 * -6534.462416077175 +
                      pbar3 * 7471.572047125511) +
               Cm4 * (pbar0 * 2208.339584446141 +
                      pbar2 * -1670.8200965430244 +
                      pbar3 * 46775.83938729213 +
                      pbar5 * -789599.9692835577) +
               Cm5 * (pbar1 * 41157.25732172145) +
               Cm6 * (pbar0 * -14089.205658052879 +
                      pbar1 * -8098.260471295071)) +
        CL1 * (Cm0 * (pbar0 * -6.348812254660877 +
                      pbar1 * 6.511098999215912 +
                      pbar2 * 32.43346391045733) +
               Cm1 * (pbar0 * 11.559665918251667 +
                      pbar1 * -204.76525645060565 +
                      pbar2 * -239.41232029030147) +
               Cm2 * (pbar0 * 42.76894938053898) +
               Cm3 * (pbar1 * 2293.481280255066));
    
    L[4] =
        CL0 * (Cm0 * (pbar0 * -3.86136579354715 +
                      pbar1 * 31.679681051626744 +
                      pbar3 * 315.66351169251277 +
                      pbar4 * 690.3334879806071) +
               Cm1 * (pbar0 * -66.05975858510737 +
                      pbar1 * 54.845339876012815 +
                      pbar2 * 324.58125188980665) +
               Cm2 * (pbar0 * -172.11522393244633 +
                      pbar1 * -637.3442664273438 +
                      pbar3 * -4124.483846477042) +
               Cm3 * (pbar0 * 67.82530976851554 +
                      pbar1 * -4665.462643366867 +
                      pbar2 * 5131.472416540916 +
                      pbar3 * -32629.523111873263) +
               Cm4 * (pbar0 * 3676.3690512093754 +
                      pbar1 * 4948.173901555719 +
                      pbar2 * 25465.75489844725 +
                      pbar5 * -746143.1765036428) +
               Cm5 * (pbar1 * 33466.23218106415 +
                      pbar2 * -102689.00767401261 +
                      pbar3 * 478829.8243930842 +
                      pbar4 * 41174.824059438164) +
               Cm6 * (pbar0 * -12345.365810229774 +
                      pbar2 * -486747.88832771825 +
                      pbar3 * 391109.29802978766 +
                      pbar4 * 3223258.8380110357)) +
        CL1 * (Cm0 * (pbar0 * -6.97446183512062 +
                      pbar1 * -15.225959064517792 +
                      pbar2 * -37.08032828264797 +
                      pbar4 * -2169.904080059758) +
               Cm1 * (pbar0 * -2.579955093447174 +
                      pbar1 * 19.550340569154926) +
               Cm2 * (pbar0 * 190.7611790283789 +
                      pbar2 * 207.61130839626867) +
               Cm3 * (pbar1 * 1064.7071614886515 +
                      pbar2 * -2361.21496562583) +
               Cm4 * (pbar0 * -2029.8821124227488) +
               Cm5 * (pbar2 * 81329.00270686777 +
                      pbar4 * 148123.96917017517) +
               Cm6 * (pbar1 * -26738.2388394795 +
                      pbar3 * -54546.5968776766)) +
        CL2 * (Cm0 * (pbar1 * 32.9411044265426 +
                      pbar2 * 107.41434129933484) +
               Cm1 * (pbar1 * -131.38149419720105 +
                      pbar2 * -199.10122354253224) +
               Cm2 * (pbar0 * -51.48087518807188) +
               Cm3 * (pbar1 * 933.3634823380227) +
               Cm4 * (pbar0 * 916.469645645092 +
                      pbar1 * 7068.686034073122) +
               Cm5 * (pbar0 * -511.5447689550243)) +
        CL3 * (Cm4 * (pbar1 * -5451.111975350683) +
               Cm5 * (pbar0 * 358.73421900463893));
    
    // set center control surface
    lr[5] = bounds(C);
    // loop thru inboard to outboard control surfaces
    for (i=0; i<5; i++) {
        // set left control surface
        lr[4-i] = bounds( L[i] );
        // set right control surface
        lr[6+i] = bounds( R[i] );
    }
}

void mode3(struct pilotCommands pilot, double CL1, double *lr) {
    double C, S[5], A[5];
    double CL2, CL3;
    double Cm1, Cm2, Cm3, Cm4, Cm5;
    double pbar1, pbar2, pbar3, pbar4, pbar5;
    int i;
    
    Cm1   = -pwm2frac(pilot.ele) * 0.1;
    pbar1 = -pwm2frac(pilot.ail) * 0.1;
    
    CL2 = CL1 * CL1;
    CL3 = CL2 * CL1;
    
    Cm2 = Cm1 * Cm1;
    Cm3 = Cm2 * Cm1;
    Cm4 = Cm3 * Cm1;
    Cm5 = Cm4 * Cm1;
    
    pbar2 = pbar1 * pbar1;
    pbar3 = pbar2 * pbar1;
    pbar4 = pbar3 * pbar1;
    pbar5 = pbar4 * pbar1;
    
    C = (
              (      (        -3.7048294490565907 +
                      pbar1 * -9.194591412594493e-08 +
                      pbar2 * 316.94518911384296 +
                      pbar3 * 3.505109464724733e-05 +
                      pbar4 * -41073.441642915546 +
                      pbar5 * -0.0027598493470032633) +
               Cm1 * (        -87.26205710580105 +
                      pbar1 * 3.849276250719424e-06 +
                      pbar2 * -14829.38246107556 +
                      pbar3 * -0.0012163071810297045 +
                      pbar4 * -155147.98230948436 +
                      pbar5 * 0.08415684113747787) +
               Cm2 * (        -212.20064489285397 +
                      pbar1 * 9.047808484933834e-05 +
                      pbar2 * -378922.17655438103 +
                      pbar3 * -0.03522366746823071 +
                      pbar4 * 38307393.52024008 +
                      pbar5 * 2.8023424466338502) +
               Cm3 * (        2536.4658428282437 +
                      pbar1 * -0.001680736701849623 +
                      pbar2 * 1098654.3990960782 +
                      pbar3 * 0.5488907142313998 +
                      pbar4 * 204469972.3260286 +
                      pbar5 * -38.97813446163493) +
               Cm4 * (        19136.707917621774 +
                      pbar1 * -0.0090314615920144 +
                      pbar2 * 28348061.70025183 +
                      pbar3 * 3.5282340760517297 +
                      pbar4 * -2908782976.707836 +
                      pbar5 * -281.18061100339753) +
               Cm5 * (        53399.67540371003 +
                      pbar1 * 0.13539097335590194 +
                      pbar2 * 15558840.76933078 +
                      pbar3 * -44.84681258344961 +
                      pbar4 * -19099378768.02346 +
                      pbar5 * 3218.8739023618596)) +
        CL1 * (      (        -9.039645822835478 +
                      pbar1 * 9.100202481869343e-07 +
                      pbar2 * 887.1136359718428 +
                      pbar3 * -0.00034601854262780973 +
                      pbar4 * 164914.2646031874 +
                      pbar5 * 0.027210717308548786) +
               Cm1 * (        106.13342368324089 +
                      pbar1 * -3.3451513097992624e-05 +
                      pbar2 * 53805.19019227413 +
                      pbar3 * 0.010155249811958917 +
                      pbar4 * 8212094.329092853 +
                      pbar5 * -0.6795230957426603) +
               Cm2 * (        2781.596303718651 +
                      pbar1 * -0.0009080321841132118 +
                      pbar2 * 2486389.7107064887 +
                      pbar3 * 0.353122450511591 +
                      pbar4 * -305008958.4223763 +
                      pbar5 * -28.07598931549074) +
               Cm3 * (        -3380.3222580857037 +
                      pbar1 * 0.014961619932001449 +
                      pbar2 * 3827017.881330572 +
                      pbar3 * -4.75382869001806 +
                      pbar4 * -3499703004.263449 +
                      pbar5 * 330.49334325335815) +
               Cm4 * (        -211811.76228462099 +
                      pbar1 * 0.0908541429928648 +
                      pbar2 * -197969371.94721806 +
                      pbar3 * -35.462293423725356 +
                      pbar4 * 23569981902.977535 +
                      pbar5 * 2824.582959868383) +
               Cm5 * (        -1000961.5278173692 +
                      pbar1 * -1.216794120355368 +
                      pbar2 * -1084007816.3731773 +
                      pbar3 * 393.862468961637 +
                      pbar4 * 289356755323.7173 +
                      pbar5 * -27785.831373624376)) +
        CL2 * (      (        3.3558877321075262 +
                      pbar1 * -2.108122206629495e-06 +
                      pbar2 * 690.673925419168 +
                      pbar3 * 0.0008003021922454932 +
                      pbar4 * -258330.05230337786 +
                      pbar5 * -0.06287668449965922) +
               Cm1 * (        12.542637919112753 +
                      pbar1 * 7.0674761856939e-05 +
                      pbar2 * -24361.365233672157 +
                      pbar3 * -0.020777437287665453 +
                      pbar4 * -29470149.918554492 +
                      pbar5 * 1.3517553667198239) +
               Cm2 * (        -6005.782878555546 +
                      pbar1 * 0.002120778680458437 +
                      pbar2 * -3483110.015184681 +
                      pbar3 * -0.8242155033629321 +
                      pbar4 * 502966770.14357036 +
                      pbar5 * 65.50500058954333) +
               Cm3 * (        -22392.425997376744 +
                      pbar1 * -0.03218616468464887 +
                      pbar2 * -46485133.34760266 +
                      pbar3 * 10.015109384322425 +
                      pbar4 * 12028999795.062487 +
                      pbar5 * -684.8727329314347) +
               Cm4 * (        471493.6765401484 +
                      pbar1 * -0.21248581980282785 +
                      pbar2 * 243950825.01068842 +
                      pbar3 * 82.89486682291455 +
                      pbar4 * -36644569122.67144 +
                      pbar5 * -6600.376311034696) +
               Cm5 * (        3531233.7338597854 +
                      pbar1 * 2.635967661425047 +
                      pbar2 * 5425369572.05897 +
                      pbar3 * -838.6696350359257 +
                      pbar4 * -968154512990.0052 +
                      pbar5 * 58399.20284706977)) +
        CL3 * (      (        -0.3324162349365002 +
                      pbar1 * 1.3566803809357146e-06 +
                      pbar2 * -546.6492352180607 +
                      pbar3 * -0.0005145000209145119 +
                      pbar4 * 5789.250688626107 +
                      pbar5 * 0.04039643965204634) +
               Cm1 * (        -74.6436419014009 +
                      pbar1 * -4.249349976199511e-05 +
                      pbar2 * -33669.757125349075 +
                      pbar3 * 0.012169869660871972 +
                      pbar4 * 24719869.674842604 +
                      pbar5 * -0.7729226346771932) +
               Cm2 * (        3289.491862289109 +
                      pbar1 * -0.0013721222235681101 +
                      pbar2 * 1057089.9516321144 +
                      pbar3 * 0.5330408086539097 +
                      pbar4 * -197607529.5948063 +
                      pbar5 * -42.35255544278445) +
               Cm3 * (        27922.11341126055 +
                      pbar1 * 0.019624528467127283 +
                      pbar2 * 50859950.0203951 +
                      pbar3 * -6.007395486908118 +
                      pbar4 * -9957317156.23105 +
                      pbar5 * 405.396914001639) +
               Cm4 * (        -256235.66799317684 +
                      pbar1 * 0.13759667371782638 +
                      pbar2 * -41430178.89628082 +
                      pbar3 * -53.66152012214221 +
                      pbar4 * 12253348075.277355 +
                      pbar5 * 4271.792268482657) +
               Cm5 * (        -2997679.688088811 +
                      pbar1 * -1.6157324985299297 +
                      pbar2 * -5223437351.381798 +
                      pbar3 * 507.29539107830584 +
                      pbar4 * 799461948477.0525 +
                      pbar5 * -34964.19748990726)));
    
    S[0] = (
              (      (        -2.5199417630843484 +
                      pbar1 * -9.383842873573609e-08 +
                      pbar2 * 200.5460531340597 +
                      pbar3 * 3.7188104561841204e-05 +
                      pbar4 * -14833.04273672006 +
                      pbar5 * -0.0029990485386489946) +
               Cm1 * (        -60.71666996299667 +
                      pbar1 * 1.77636112105517e-05 +
                      pbar2 * -3134.7521528288416 +
                      pbar3 * -0.006776503415137062 +
                      pbar4 * -374863.139801983 +
                      pbar5 * 0.5334968455769612) +
               Cm2 * (        -209.49612102018378 +
                      pbar1 * 7.486726178224024e-05 +
                      pbar2 * -159951.39863453634 +
                      pbar3 * -0.029947979320893364 +
                      pbar4 * 15339410.639514394 +
                      pbar5 * 2.4235544519622123) +
               Cm3 * (        4712.629545864574 +
                      pbar1 * -0.007122966667872291 +
                      pbar2 * -828180.3128680608 +
                      pbar3 * 2.7273348370103707 +
                      pbar4 * 213248904.42085353 +
                      pbar5 * -215.1924076974503) +
               Cm4 * (        18508.420578097102 +
                      pbar1 * -0.007130206891835512 +
                      pbar2 * 10603463.301652247 +
                      pbar3 * 2.8586965088349263 +
                      pbar4 * -1100853939.6433141 +
                      pbar5 * -231.55869964166155) +
               Cm5 * (        -190243.1140910589 +
                      pbar1 * 0.551649977024449 +
                      pbar2 * 124709570.8130266 +
                      pbar3 * -211.58629123865825 +
                      pbar4 * -18927980828.167885 +
                      pbar5 * 16711.67796792116)) +
        CL1 * (      (        0.6203081722358273 +
                      pbar1 * 9.735012161452884e-07 +
                      pbar2 * 962.649531337759 +
                      pbar3 * -0.0003853112739709431 +
                      pbar4 * -71159.05438222323 +
                      pbar5 * 0.0310671109924981) +
               Cm1 * (        75.34936770437795 +
                      pbar1 * -0.00017610882259521716 +
                      pbar2 * -17846.921208910997 +
                      pbar3 * 0.06712571944818782 +
                      pbar4 * 8343617.220958288 +
                      pbar5 * -5.281288448165564) +
               Cm2 * (        2572.2672913871784 +
                      pbar1 * -0.0007773020734562826 +
                      pbar2 * 533265.7243169318 +
                      pbar3 * 0.310514474100347 +
                      pbar4 * -78122491.26100752 +
                      pbar5 * -25.11358706424067) +
               Cm3 * (        -20778.926791764217 +
                      pbar1 * 0.07077384804587751 +
                      pbar2 * 21912795.82854527 +
                      pbar3 * -27.083177706334112 +
                      pbar4 * -3456937347.9806414 +
                      pbar5 * 2135.9782594792428) +
               Cm4 * (        -211792.10045871724 +
                      pbar1 * 0.07404015148210526 +
                      pbar2 * -28927521.64555019 +
                      pbar3 * -29.644197724732226 +
                      pbar4 * 5289950594.64516 +
                      pbar5 * 2399.627197582636) +
               Cm5 * (        1256383.7181726596 +
                      pbar1 * -5.486122186648669 +
                      pbar2 * -2181097708.4312825 +
                      pbar3 * 2103.2181944974545 +
                      pbar4 * 280220221286.9196 +
                      pbar5 * -166057.5228624652)) +
        CL2 * (      (        2.0989823822155205 +
                      pbar1 * -2.315188808388559e-06 +
                      pbar2 * 899.707886236298 +
                      pbar3 * 0.0009156665250465111 +
                      pbar4 * 267267.9512411224 +
                      pbar5 * -0.07379757607961172) +
               Cm1 * (        5.083713765677619 +
                      pbar1 * 0.00040807251309361224 +
                      pbar2 * 60705.685495563586 +
                      pbar3 * -0.15545720862214288 +
                      pbar4 * -21427312.595854778 +
                      pbar5 * 12.227418830868535) +
               Cm2 * (        -4255.705052788982 +
                      pbar1 * 0.0018492786749032604 +
                      pbar2 * -258477.8673263768 +
                      pbar3 * -0.7382149698928705 +
                      pbar4 * 59267935.50401933 +
                      pbar5 * 59.681617343144204) +
               Cm3 * (        22203.646591940374 +
                      pbar1 * -0.16420897430923667 +
                      pbar2 * -75037197.34045605 +
                      pbar3 * 62.814252264463725 +
                      pbar4 * 10313570623.942871 +
                      pbar5 * -4952.997404411646) +
               Cm4 * (        423556.5390006107 +
                      pbar1 * -0.17615960474758444 +
                      pbar2 * -51159609.11294409 +
                      pbar3 * 70.48015816398255 +
                      pbar4 * -784237173.8498763 +
                      pbar5 * -5702.995644249584) +
               Cm5 * (        -2029733.5364893277 +
                      pbar1 * 12.735578643436213 +
                      pbar2 * 7271055676.351221 +
                      pbar3 * -4880.890760469486 +
                      pbar4 * -859591331747.5698 +
                      pbar5 * 385302.2804312577)) +
        CL3 * (      (        -1.2005947491069624 +
                      pbar1 * 1.5150844183315015e-06 +
                      pbar2 * -1640.3845295897133 +
                      pbar3 * -0.0005989432096410693 +
                      pbar4 * -232718.01670975582 +
                      pbar5 * 0.04825601329937804) +
               Cm1 * (        -23.381518322231273 +
                      pbar1 * -0.0002625702362131864 +
                      pbar2 * -59214.61036502296 +
                      pbar3 * 0.0999907990539478 +
                      pbar4 * 15520440.539606364 +
                      pbar5 * -7.863342195830358) +
               Cm2 * (        1721.4029522437727 +
                      pbar1 * -0.0012104813970831778 +
                      pbar2 * -275409.9901877651 +
                      pbar3 * 0.48300164721833394 +
                      pbar4 * 26708352.219954126 +
                      pbar5 * -39.038841829378875) +
               Cm3 * (        -5453.437177204897 +
                      pbar1 * 0.10574911150937825 +
                      pbar2 * 62430973.29328059 +
                      pbar3 * -40.44125240501138 +
                      pbar4 * -7985233044.933218 +
                      pbar5 * 3188.4671983094613) +
               Cm4 * (        -214136.16091575794 +
                      pbar1 * 0.11531285254100439 +
                      pbar2 * 95829533.80973135 +
                      pbar3 * -46.11567951862275 +
                      pbar4 * -6297799327.81496 +
                      pbar5 * 3730.606681135319) +
               Cm5 * (        837221.2478804885 +
                      pbar1 * -8.204425587572041 +
                      pbar2 * -5995569076.166275 +
                      pbar3 * 3143.641570245862 +
                      pbar4 * 679370508521.9722 +
                      pbar5 * -248137.53189289884)));
    
    S[1] = (
              (      (        -2.397690716475905 +
                      pbar1 * -5.548459943171024e-08 +
                      pbar2 * 400.6414297509754 +
                      pbar3 * 1.915809931723607e-05 +
                      pbar4 * -49762.817246780105 +
                      pbar5 * -0.0014189767948517115) +
               Cm1 * (        -58.627028915776705 +
                      pbar1 * 3.758907271283657e-06 +
                      pbar2 * -1231.754755825167 +
                      pbar3 * -0.0009568703073977017 +
                      pbar4 * -1251546.1690234784 +
                      pbar5 * 0.05427781022357257) +
               Cm2 * (        -207.9028360999042 +
                      pbar1 * 7.212656369817479e-05 +
                      pbar2 * -272232.11227192753 +
                      pbar3 * -0.027119675874468764 +
                      pbar4 * 35090672.04046451 +
                      pbar5 * 2.1123718047320774) +
               Cm3 * (        1329.1319843243123 +
                      pbar1 * -0.0018508581150138439 +
                      pbar2 * -493049.8732948225 +
                      pbar3 * 0.5324074527970393 +
                      pbar4 * 442529120.77461034 +
                      pbar5 * -34.28966744971149) +
               Cm4 * (        20121.125486046065 +
                      pbar1 * -0.007562002035525133 +
                      pbar2 * 19547623.106827512 +
                      pbar3 * 2.874874428406646 +
                      pbar4 * -2699955258.025568 +
                      pbar5 * -225.2982228425243) +
               Cm5 * (        60303.307958032114 +
                      pbar1 * 0.15543998532192352 +
                      pbar2 * 92429996.3656005 +
                      pbar3 * -46.498066039595706 +
                      pbar4 * -35619104209.167915 +
                      pbar5 * 3100.1444680221675)) +
        CL1 * (      (        -4.10841292193263 +
                      pbar1 * 5.216122090589711e-07 +
                      pbar2 * -3417.6019724193366 +
                      pbar3 * -0.00017671370025560066 +
                      pbar4 * 484463.986922588 +
                      pbar5 * 0.012931798417586437) +
               Cm1 * (        18.670365398702156 +
                      pbar1 * -3.046711478414447e-05 +
                      pbar2 * -5624.956956094868 +
                      pbar3 * 0.0066968872923684215 +
                      pbar4 * 17438141.172880355 +
                      pbar5 * -0.3099309012587887) +
               Cm2 * (        2195.573501420223 +
                      pbar1 * -0.0007084177616102653 +
                      pbar2 * 2104592.376688653 +
                      pbar3 * 0.26503531977837647 +
                      pbar4 * -339548252.7370805 +
                      pbar5 * -20.582364162748807) +
               Cm3 * (        8245.633447795932 +
                      pbar1 * 0.01597817856944343 +
                      pbar2 * 14661171.803400304 +
                      pbar3 * -4.304268601011798 +
                      pbar4 * -6203359114.818226 +
                      pbar5 * 260.2937509074523) +
               Cm4 * (        -195060.1284917788 +
                      pbar1 * 0.07468169856933321 +
                      pbar2 * -148666249.15098503 +
                      pbar3 * -28.281316220356118 +
                      pbar4 * 25923033716.3027 +
                      pbar5 * 2211.142550142944) +
               Cm5 * (        -1458066.1135717167 +
                      pbar1 * -1.369046464792705 +
                      pbar2 * -1625245370.8176901 +
                      pbar3 * 390.302151608886 +
                      pbar4 * 484746055727.1104 +
                      pbar5 * -24955.76648989322)) +
        CL2 * (      (        6.452034649045866 +
                      pbar1 * -1.1721308672124067e-06 +
                      pbar2 * 11699.35481171764 +
                      pbar3 * 0.00039241079374067073 +
                      pbar4 * -1172227.7559179915 +
                      pbar5 * -0.028481943609571236) +
               Cm1 * (        172.0200926892915 +
                      pbar1 * 6.087752102432847e-05 +
                      pbar2 * 75003.64758266378 +
                      pbar3 * -0.011540785262331292 +
                      pbar4 * -54215490.669598654 +
                      pbar5 * 0.3944613198965012) +
               Cm2 * (        -3969.0208964855497 +
                      pbar1 * 0.0016343380497683122 +
                      pbar2 * -3865181.2518931953 +
                      pbar3 * -0.609735279029124 +
                      pbar4 * 721148270.3015114 +
                      pbar5 * 47.26936237262799) +
               Cm3 * (        -66285.25687869075 +
                      pbar1 * -0.03362198868827717 +
                      pbar2 * -66750840.49034352 +
                      pbar3 * 8.580478123870073 +
                      pbar4 * 19400194902.588455 +
                      pbar5 * -489.75733080039583) +
               Cm4 * (        370945.5886848824 +
                      pbar1 * -0.1728356117117503 +
                      pbar2 * 207838631.53403375 +
                      pbar3 * 65.3113207779845 +
                      pbar4 * -51259559492.58329 +
                      pbar5 * -5099.535107313651) +
               Cm5 * (        6144130.094577267 +
                      pbar1 * 2.925089624914093 +
                      pbar2 * 6545938699.4593 +
                      pbar3 * -803.1209132195589 +
                      pbar4 * -1510499031205.3591 +
                      pbar5 * 49585.73470291713)) +
        CL3 * (      (        -2.8717982243041846 +
                      pbar1 * 7.393741336029347e-07 +
                      pbar2 * -7857.234282492839 +
                      pbar3 * -0.0002455472494391052 +
                      pbar4 * 590314.7016581824 +
                      pbar5 * 0.01772016129361603) +
               Cm1 * (        -190.2657680449639 +
                      pbar1 * -3.495676944997421e-05 +
                      pbar2 * -77119.35664522366 +
                      pbar3 * 0.005703766805654714 +
                      pbar4 * 42919923.78256546 +
                      pbar5 * -0.11389126286914086) +
               Cm2 * (        1522.7188887860111 +
                      pbar1 * -0.0010490823390198532 +
                      pbar2 * 1825690.188259679 +
                      pbar3 * 0.3906972235430557 +
                      pbar4 * -372355457.6413332 +
                      pbar5 * -30.254850758888228) +
               Cm3 * (        67951.63620921868 +
                      pbar1 * 0.020154832717791414 +
                      pbar2 * 59702995.088310644 +
                      pbar3 * -4.916275849137083 +
                      pbar4 * -15239120857.662493 +
                      pbar5 * 265.99639701791364) +
               Cm4 * (        -149492.71355060046 +
                      pbar1 * 0.11116868994871064 +
                      pbar2 * -53021637.2810477 +
                      pbar3 * -41.95226271501096 +
                      pbar4 * 23666647884.71626 +
                      pbar5 * 3272.908140648704) +
               Cm5 * (        -5609161.023458999 +
                      pbar1 * -1.774509828384273 +
                      pbar2 * -5756880149.916134 +
                      pbar3 * 472.74563565522595 +
                      pbar4 * 1189952221332.5364 +
                      pbar5 * -28329.979231930323)));
    
    S[2] = (
              (      (        -2.0393741850572384 +
                      pbar1 * 1.8284112465293605e-07 +
                      pbar2 * 608.464994764675 +
                      pbar3 * -7.326930255536842e-05 +
                      pbar4 * -66400.73576447433 +
                      pbar5 * 0.005921279539560187) +
               Cm1 * (        -45.767375462269264 +
                      pbar1 * -2.324808334592862e-05 +
                      pbar2 * -13717.63236834796 +
                      pbar3 * 0.008600798720962106 +
                      pbar4 * 1656505.5793622523 +
                      pbar5 * -0.6665789043226855) +
               Cm2 * (        -373.6389716458617 +
                      pbar1 * -0.00015282378767884417 +
                      pbar2 * -259728.06471475388 +
                      pbar3 * 0.061355680449555264 +
                      pbar4 * 11314368.085440414 +
                      pbar5 * -4.958732189659277) +
               Cm3 * (        -7479.665771801859 +
                      pbar1 * 0.009559984887082533 +
                      pbar2 * 5297081.67631312 +
                      pbar3 * -3.556682753696348 +
                      pbar4 * -492359398.5957254 +
                      pbar5 * 276.5937461239371) +
               Cm4 * (        25602.66692508583 +
                      pbar1 * 0.014840026508485052 +
                      pbar2 * 15030341.927815618 +
                      pbar3 * -5.963361820682233 +
                      pbar4 * -269783801.5691974 +
                      pbar5 * 482.10844637410474) +
               Cm5 * (        700897.7000701749 +
                      pbar1 * -0.7493414215181085 +
                      pbar2 * -337266531.40416247 +
                      pbar3 * 279.49681971722566 +
                      pbar4 * 28443580322.706745 +
                      pbar5 * -21769.25825984615)) +
        CL1 * (      (        -15.070518575587458 +
                      pbar1 * -1.932258334929361e-06 +
                      pbar2 * -7789.182088026656 +
                      pbar3 * 0.0007733655526551965 +
                      pbar4 * 866133.5236599809 +
                      pbar5 * -0.062481457312841576) +
               Cm1 * (        -242.9263870763917 +
                      pbar1 * 0.0002320559824794147 +
                      pbar2 * 174346.22134007874 +
                      pbar3 * -0.0857452215765409 +
                      pbar4 * -21125607.25003519 +
                      pbar5 * 6.63957798987565) +
               Cm2 * (        3526.9320024202193 +
                      pbar1 * 0.0016098270856210618 +
                      pbar2 * 2035611.02763631 +
                      pbar3 * -0.6455267263018102 +
                      pbar4 * -69307533.2651086 +
                      pbar5 * 52.14373864693424) +
               Cm3 * (        99419.44595240666 +
                      pbar1 * -0.09572731612823947 +
                      pbar2 * -56227368.63714723 +
                      pbar3 * 35.587987118597226 +
                      pbar4 * 5720432055.060306 +
                      pbar5 * -2766.084421020919) +
               Cm4 * (        -208249.41178347907 +
                      pbar1 * -0.15619730338638146 +
                      pbar2 * -81066791.45568408 +
                      pbar3 * 62.690426640199156 +
                      pbar4 * -3764239222.7220964 +
                      pbar5 * -5065.351910405138) +
               Cm5 * (        -8581682.730699448 +
                      pbar1 * 7.513185974059156 +
                      pbar2 * 3343711383.673952 +
                      pbar3 * -2800.8295956991337 +
                      pbar4 * -313547811288.5803 +
                      pbar5 * 218060.00650489482)) +
        CL2 * (      (        21.260041014907102 +
                      pbar1 * 4.642540312666532e-06 +
                      pbar2 * 19404.47889469326 +
                      pbar3 * -0.0018570181353122811 +
                      pbar4 * -2228843.5355381295 +
                      pbar5 * 0.14998253731142122) +
               Cm1 * (        825.3505248846426 +
                      pbar1 * -0.0005412625444302524 +
                      pbar2 * -267012.87761114544 +
                      pbar3 * 0.19986290593817582 +
                      pbar4 * 47355139.063347645 +
                      pbar5 * -15.47011066996217) +
               Cm2 * (        -7062.566152780782 +
                      pbar1 * -0.0038611860892925273 +
                      pbar2 * -1893886.782435238 +
                      pbar3 * 1.5474572823974784 +
                      pbar4 * -32272158.563644752 +
                      pbar5 * -124.96437112342454) +
               Cm3 * (        -300226.4306138165 +
                      pbar1 * 0.22367664375322713 +
                      pbar2 * 80748222.14400905 +
                      pbar3 * -83.12211932189658 +
                      pbar4 * -12155425708.35772 +
                      pbar5 * 6459.231673559429) +
               Cm4 * (        351591.0731952094 +
                      pbar1 * 0.3744737633336754 +
                      pbar2 * -182756773.20934215 +
                      pbar3 * -150.21577578665043 +
                      pbar4 * 34951608922.68931 +
                      pbar5 * 12134.073249451349) +
               Cm5 * (        25078260.380570367 +
                      pbar1 * -17.567960586258597 +
                      pbar2 * -3141338174.636169 +
                      pbar3 * 6547.273513590244 +
                      pbar4 * 573372245984.2313 +
                      pbar5 * -509659.4429612595)) +
        CL3 * (      (        -10.461514895224248 +
                      pbar1 * -3.0570660364776584e-06 +
                      pbar2 * -9873.638845938585 +
                      pbar3 * 0.001222414032117604 +
                      pbar4 * 1384097.8339500183 +
                      pbar5 * -0.09870634474177523) +
               Cm1 * (        -636.814975753092 +
                      pbar1 * 0.00035003475592617977 +
                      pbar2 * 66023.49081394532 +
                      pbar3 * -0.1291957462480856 +
                      pbar4 * -28468752.446314048 +
                      pbar5 * 9.997925126646) +
               Cm2 * (        3153.722533161265 +
                      pbar1 * 0.002539987468251889 +
                      pbar2 * -875958.3916204079 +
                      pbar3 * -1.0176503236126944 +
                      pbar4 * 129167333.3580462 +
                      pbar5 * 82.16657065789991) +
               Cm3 * (        236694.34777106598 +
                      pbar1 * -0.14481498996371495 +
                      pbar2 * -13928816.457680713 +
                      pbar3 * 53.802261014494924 +
                      pbar4 * 6614279732.292731 +
                      pbar5 * -4180.314225413284) +
               Cm4 * (        -73972.6707360418 +
                      pbar1 * -0.24627201146907948 +
                      pbar2 * 343638155.21583664 +
                      pbar3 * 98.75992830750717 +
                      pbar4 * -35074792207.842575 +
                      pbar5 * -7976.364782925515) +
               Cm5 * (        -19647677.94233486 +
                      pbar1 * 11.379197981969876 +
                      pbar2 * -1274056563.1110356 +
                      pbar3 * -4240.066754525319 +
                      pbar4 * -243788848861.20648 +
                      pbar5 * 330030.0369516302)));
    
    S[3] = (
              (      (        -2.534746456903358 +
                      pbar1 * -1.388815852243079e-07 +
                      pbar2 * -72.51982214626922 +
                      pbar3 * 5.29812650085703e-05 +
                      pbar4 * 1850.119615327084 +
                      pbar5 * -0.00420159523052924) +
               Cm1 * (        -13.436110436102835 +
                      pbar1 * 3.921110477040182e-05 +
                      pbar2 * 32292.162806834564 +
                      pbar3 * -0.014955014225442913 +
                      pbar4 * -2935138.770368089 +
                      pbar5 * 1.175553159559261) +
               Cm2 * (        -349.57972563861387 +
                      pbar1 * 0.0001263218661363656 +
                      pbar2 * 269117.11681959947 +
                      pbar3 * -0.049484209120189845 +
                      pbar4 * -12867680.64801114 +
                      pbar5 * 3.9691965415818387) +
               Cm3 * (        -15811.433001138994 +
                      pbar1 * -0.015912729689824122 +
                      pbar2 * -14036340.039839711 +
                      pbar3 * 6.0922129179178555 +
                      pbar4 * 1132780341.1136816 +
                      pbar5 * -479.970660907867) +
               Cm4 * (        28463.93721687066 +
                      pbar1 * -0.012536730115367243 +
                      pbar2 * -24116434.546891466 +
                      pbar3 * 4.942494723794997 +
                      pbar4 * 993669407.6690698 +
                      pbar5 * -397.59225815336083) +
               Cm5 * (        876415.5276045408 +
                      pbar1 * 1.2379181945238678 +
                      pbar2 * 932016643.6917746 +
                      pbar3 * -474.7680625145226 +
                      pbar4 * -70536126649.10583 +
                      pbar5 * 37442.82269988508)) +
        CL1 * (      (        -13.751143331415387 +
                      pbar1 * 1.4660704019492039e-06 +
                      pbar2 * -2243.690527972558 +
                      pbar3 * -0.0005571684593448984 +
                      pbar4 * 188014.23554195446 +
                      pbar5 * 0.04413072758973425) +
               Cm1 * (        -621.2074630485756 +
                      pbar1 * -0.00039480054559556645 +
                      pbar2 * -303888.8417183604 +
                      pbar3 * 0.15059456419801434 +
                      pbar4 * 30868106.46576771 +
                      pbar5 * -11.833944964316936) +
               Cm2 * (        3150.7575911319127 +
                      pbar1 * -0.001325097796794819 +
                      pbar2 * -1859592.0301875342 +
                      pbar3 * 0.5171679306089622 +
                      pbar4 * 42345002.93868245 +
                      pbar5 * -41.4124415878168) +
               Cm3 * (        148695.44882642996 +
                      pbar1 * 0.16072101324833435 +
                      pbar2 * 141812544.66872594 +
                      pbar3 * -61.55905577552617 +
                      pbar4 * -11482595122.418242 +
                      pbar5 * 4849.514547375998) +
               Cm4 * (        -333111.5181195531 +
                      pbar1 * 0.13131211355188008 +
                      pbar2 * 139010615.91508073 +
                      pbar3 * -51.58100678873675 +
                      pbar4 * 859827967.640611 +
                      pbar5 * 4142.176434158477) +
               Cm5 * (        -7406832.274246282 +
                      pbar1 * -12.518656789950251 +
                      pbar2 * -9520281671.769114 +
                      pbar3 * 4803.832258727121 +
                      pbar4 * 695881619352.5504 +
                      pbar5 * -378861.43182455044)) +
        CL2 * (      (        8.725222033047276 +
                      pbar1 * -3.515995963900593e-06 +
                      pbar2 * 5576.908227916462 +
                      pbar3 * 0.001333602772156989 +
                      pbar4 * -412769.5623570579 +
                      pbar5 * -0.10551211841047083) +
               Cm1 * (        1319.1489498841404 +
                      pbar1 * 0.0009237316865452663 +
                      pbar2 * 592460.2214730596 +
                      pbar3 * -0.3522602397520657 +
                      pbar4 * -65081544.73472775 +
                      pbar5 * 27.67400973006837) +
               Cm2 * (        -7057.614968614689 +
                      pbar1 * 0.0031667448245316712 +
                      pbar2 * 603492.2006887568 +
                      pbar3 * -1.2337063409425444 +
                      pbar4 * 165825534.15736848 +
                      pbar5 * 98.69932304254985) +
               Cm3 * (        -266117.60356156604 +
                      pbar1 * -0.3767516466629052 +
                      pbar2 * -261547544.54279733 +
                      pbar3 * 144.29344128113698 +
                      pbar4 * 21994288217.266647 +
                      pbar5 * -11365.634117769838) +
               Cm4 * (        910603.8917876137 +
                      pbar1 * -0.31354657184532264 +
                      pbar2 * 162931751.7760348 +
                      pbar3 * 122.94853324300533 +
                      pbar4 * -37353474580.42207 +
                      pbar5 * -9864.696490119939) +
               Cm5 * (        10506271.665360313 +
                      pbar1 * 29.367007063472048 +
                      pbar2 * 15414450099.308626 +
                      pbar3 * -11269.238650306303 +
                      pbar4 * -1141525523050.4226 +
                      pbar5 * 888684.8185706643)) +
        CL3 * (      (        -3.6035286608539208 +
                      pbar1 * 2.3117496216619e-06 +
                      pbar2 * -5059.006593919729 +
                      pbar3 * -0.0008757966738659743 +
                      pbar4 * 357526.13225899875 +
                      pbar5 * 0.06923899406052869) +
               Cm1 * (        -696.2734951756953 +
                      pbar1 * -0.0005982390162287841 +
                      pbar2 * -284305.4422360913 +
                      pbar3 * 0.22807633902554034 +
                      pbar4 * 35291654.61818194 +
                      pbar5 * -17.914493917038396) +
               Cm2 * (        4767.572811382453 +
                      pbar1 * -0.00207766170661181 +
                      pbar2 * 2019353.0369940274 +
                      pbar3 * 0.8085531401320607 +
                      pbar4 * -269608755.2804507 +
                      pbar5 * -64.65008343269255) +
               Cm3 * (        113716.79341456557 +
                      pbar1 * 0.24429660455984453 +
                      pbar2 * 114119170.44002847 +
                      pbar3 * -93.55236817403785 +
                      pbar4 * -10354444184.835527 +
                      pbar5 * 7368.025518756649) +
               Cm4 * (        -668795.1835886559 +
                      pbar1 * 0.20560564407363452 +
                      pbar2 * -385499551.2058444 +
                      pbar3 * -80.53886212063756 +
                      pbar4 * 43559117958.84422 +
                      pbar5 * 6458.6333172011355) +
               Cm5 * (        -2177060.39649887 +
                      pbar1 * -19.051548781033848 +
                      pbar2 * -4960265819.919742 +
                      pbar3 * 7310.265274446975 +
                      pbar4 * 385285898270.1316 +
                      pbar5 * -576432.0933468753)));
    
    S[4] = (
              (      (        -7.7565928942673095 +
                      pbar1 * 2.3740919511985603e-07 +
                      pbar2 * -1703.1108252056201 +
                      pbar3 * -8.341143636567639e-05 +
                      pbar4 * 174483.2292333838 +
                      pbar5 * 0.006319044564366063) +
               Cm1 * (        -232.08053731575313 +
                      pbar1 * -3.185697187096887e-05 +
                      pbar2 * -46946.50042579514 +
                      pbar3 * 0.011331442806428928 +
                      pbar4 * 6267000.843227205 +
                      pbar5 * -0.8476995264181605) +
               Cm2 * (        1688.6015202276753 +
                      pbar1 * -0.0002717313977541358 +
                      pbar2 * 778123.2768683337 +
                      pbar3 * 0.10291084043688793 +
                      pbar4 * -72837960.09238516 +
                      pbar5 * -8.107291050840036) +
               Cm3 * (        59388.75139131861 +
                      pbar1 * 0.013839022722744117 +
                      pbar2 * 21637930.927923616 +
                      pbar3 * -5.010418682742883 +
                      pbar4 * -2399927102.5296326 +
                      pbar5 * 379.4006751220071) +
               Cm4 * (        -142928.7199143101 +
                      pbar1 * 0.028081189665102632 +
                      pbar2 * -40366111.470404655 +
                      pbar3 * -10.761825702410716 +
                      pbar4 * 4426620082.052998 +
                      pbar5 * 852.8451122064797) +
               Cm5 * (        -4282163.128140368 +
                      pbar1 * -1.1055402364959657 +
                      pbar2 * -1426774097.7865906 +
                      pbar3 * 403.1047369351519 +
                      pbar4 * 156574268506.25613 +
                      pbar5 * -30668.34667768187)) +
        CL1 * (      (        38.681233332052926 +
                      pbar1 * -2.3020626161839013e-06 +
                      pbar2 * 22442.183234858087 +
                      pbar3 * 0.0007960835211828261 +
                      pbar4 * -2429316.042429092 +
                      pbar5 * -0.05981752601440653) +
               Cm1 * (        1761.0044898968006 +
                      pbar1 * 0.0003044445113435514 +
                      pbar2 * 457216.6215645648 +
                      pbar3 * -0.10769338961327636 +
                      pbar4 * -65773767.34168342 +
                      pbar5 * 8.01545827592923) +
               Cm2 * (        -17272.838807442196 +
                      pbar1 * 0.0027005553436482955 +
                      pbar2 * -6996525.030318091 +
                      pbar3 * -1.0164604921712077 +
                      pbar4 * 782729456.1365743 +
                      pbar5 * 79.83183816949118) +
               Cm3 * (        -608846.4300967335 +
                      pbar1 * -0.13412921037966732 +
                      pbar2 * -230012368.3446604 +
                      pbar3 * 48.42617350506405 +
                      pbar4 * 25530624434.132042 +
                      pbar5 * -3656.4392790338575) +
               Cm4 * (        1388743.042167015 +
                      pbar1 * -0.2801447804526941 +
                      pbar2 * 432089729.4888779 +
                      pbar3 * 106.81598612231221 +
                      pbar4 * -53970839544.531006 +
                      pbar5 * -8443.404221217443) +
               Cm5 * (        42710311.063220516 +
                      pbar1 * 10.769657413820667 +
                      pbar2 * 15907761554.100067 +
                      pbar3 * -3919.2713297566715 +
                      pbar4 * -1699513505051.5244 +
                      pbar5 * 297529.4240032259)) +
        CL2 * (      (        -90.41937422637547 +
                      pbar1 * 5.257554815276186e-06 +
                      pbar2 * -45001.466968958295 +
                      pbar3 * -0.001800032657724051 +
                      pbar4 * 5204020.977620127 +
                      pbar5 * 0.13446996632264863) +
               Cm1 * (        -4658.730155468447 +
                      pbar1 * -0.0006873265134649261 +
                      pbar2 * -1253097.6712155421 +
                      pbar3 * 0.2419506560641458 +
                      pbar4 * 166871152.4264699 +
                      pbar5 * -17.93779584297655) +
               Cm2 * (        38739.36485623418 +
                      pbar1 * -0.00626213929878624 +
                      pbar2 * 13846283.942088656 +
                      pbar3 * 2.3485427294841132 +
                      pbar4 * -1653500834.260605 +
                      pbar5 * -184.1113832387495) +
               Cm3 * (        1406877.7474724827 +
                      pbar1 * 0.30569038796223524 +
                      pbar2 * 563831149.4446977 +
                      pbar3 * -110.05743872291008 +
                      pbar4 * -61088258999.596924 +
                      pbar5 * 8290.969532166171) +
               Cm4 * (        -3038755.3159826556 +
                      pbar1 * 0.6510983618043981 +
                      pbar2 * -927271872.9235169 +
                      pbar3 * -247.5323692257534 +
                      pbar4 * 120495351139.25548 +
                      pbar5 * 19537.71205663872) +
               Cm5 * (        -96148439.34365772 +
                      pbar1 * -24.62736262773838 +
                      pbar2 * -38269638756.943794 +
                      pbar3 * 8942.892700099257 +
                      pbar4 * 4001532048123.1113 +
                      pbar5 * -677672.6391226016)) +
        CL3 * (      (        45.449757972321265 +
                      pbar1 * -3.349744507025363e-06 +
                      pbar2 * 25251.141366132786 +
                      pbar3 * 0.001139080199593938 +
                      pbar4 * -3073551.101607657 +
                      pbar5 * -0.08474542326546212) +
               Cm1 * (        3091.973101441811 +
                      pbar1 * 0.0004340006893995745 +
                      pbar2 * 850882.5460089972 +
                      pbar3 * -0.1521997414971837 +
                      pbar4 * -109167362.53032519 +
                      pbar5 * 11.250675712446332) +
               Cm2 * (        -21089.53117783903 +
                      pbar1 * 0.004030823093379552 +
                      pbar2 * -7359689.208133522 +
                      pbar3 * -1.5081827094799984 +
                      pbar4 * 938816465.367331 +
                      pbar5 * 118.0886909344325) +
               Cm3 * (        -890334.7361400994 +
                      pbar1 * -0.19431487148638954 +
                      pbar2 * -362520038.54839355 +
                      pbar3 * 69.8039476040953 +
                      pbar4 * 38790443374.83435 +
                      pbar5 * -5249.499018446559) +
               Cm4 * (        1635056.4492374791 +
                      pbar1 * -0.4197345697912111 +
                      pbar2 * 516947615.02633333 +
                      pbar3 * 159.27302033590695 +
                      pbar4 * -70842972720.85524 +
                      pbar5 * -12559.452319759925) +
               Cm5 * (        60362303.85483276 +
                      pbar1 * 15.691329039853658 +
                      pbar2 * 24250627401.943775 +
                      pbar3 * -5688.0151049455335 +
                      pbar4 * -2508880286645.806 +
                      pbar5 * 430436.30257774336)));
    
    A[0] = (
              (      (        8.98564046013181e-06 +
                      pbar1 * -31.18858010975713 +
                      pbar2 * -0.003946680499944514 +
                      pbar3 * 3130.1221582275207 +
                      pbar4 * 0.3352125612446137 +
                      pbar5 * -227226.571668887) +
               Cm1 * (        -0.0005001227569714962 +
                      pbar1 * -42.86388560232157 +
                      pbar2 * 0.22117494961783712 +
                      pbar3 * 3524.541878373939 +
                      pbar4 * -18.838114875628374 +
                      pbar5 * -2735079.3953817533) +
               Cm2 * (        0.009050971842899791 +
                      pbar1 * -3140.8667456149183 +
                      pbar2 * -4.013556492912707 +
                      pbar3 * -71232.70926870957 +
                      pbar4 * 342.2564596042755 +
                      pbar5 * 46953666.216133706) +
               Cm3 * (        0.32475026829604486 +
                      pbar1 * -44930.10993857211 +
                      pbar2 * -143.61413957330635 +
                      pbar3 * 14196233.221947309 +
                      pbar4 * 12232.336957194837 +
                      pbar5 * -178327757.08733782) +
               Cm4 * (        -1.184973853349026 +
                      pbar1 * 17003.23076152268 +
                      pbar2 * 524.8847179138785 +
                      pbar3 * 54084081.740741976 +
                      pbar4 * -44739.069701232016 +
                      pbar5 * -6840662017.228497) +
               Cm5 * (        -30.31859607276496 +
                      pbar1 * 4212074.36556025 +
                      pbar2 * 13407.672316119995 +
                      pbar3 * -1144814056.3774722 +
                      pbar4 * -1142004.6518695666 +
                      pbar5 * 26311705083.10382)) +
        CL1 * (      (        -0.00035874450230979915 +
                      pbar1 * -15.024231984468093 +
                      pbar2 * 0.1583013878645978 +
                      pbar3 * -2759.4816205472553 +
                      pbar4 * -13.471482872796365 +
                      pbar5 * 503859.5826484319) +
               Cm1 * (        -0.005802909311469592 +
                      pbar1 * 2198.2424273636757 +
                      pbar2 * 2.565950568367811 +
                      pbar3 * -580543.9086986985 +
                      pbar4 * -218.56788680663905 +
                      pbar5 * 69027436.40122408) +
               Cm2 * (        0.05368539588178184 +
                      pbar1 * 35633.49163916811 +
                      pbar2 * -23.599537288436206 +
                      pbar3 * -2226055.3320849286 +
                      pbar4 * 2005.0688874450075 +
                      pbar5 * -265281033.42427143) +
               Cm3 * (        2.229227043134744 +
                      pbar1 * 329524.22024422936 +
                      pbar2 * -985.7190976033746 +
                      pbar3 * -57008409.48216997 +
                      pbar4 * 83961.66321090213 +
                      pbar5 * -6250930380.08025) +
               Cm4 * (        -0.036691109321651275 +
                      pbar1 * -2097827.734598608 +
                      pbar2 * 5.1238372294278935 +
                      pbar3 * -90715480.42610958 +
                      pbar4 * -36.41232640521429 +
                      pbar5 * 38966638574.57493) +
               Cm5 * (        -149.47568485937484 +
                      pbar1 * -48243461.554975435 +
                      pbar2 * 66094.3157709674 +
                      pbar3 * 8804521185.35453 +
                      pbar4 * -5629747.984664373 +
                      pbar5 * 102083533406.9372)) +
        CL2 * (      (        0.001640131222536359 +
                      pbar1 * 128.45044237594396 +
                      pbar2 * -0.7243289798099953 +
                      pbar3 * -1858.8230707646073 +
                      pbar4 * 61.66149225444266 +
                      pbar5 * -19766.422548261584) +
               Cm1 * (        0.02383210368444865 +
                      pbar1 * -1453.8331906783355 +
                      pbar2 * -10.53841201503103 +
                      pbar3 * 597914.6187782639 +
                      pbar4 * 897.6425536049426 +
                      pbar5 * -127246008.00003166) +
               Cm2 * (        -0.4382648786519893 +
                      pbar1 * -55740.07140689072 +
                      pbar2 * 193.40947284295106 +
                      pbar3 * 2945825.348623334 +
                      pbar4 * -16459.60038890317 +
                      pbar5 * 551204615.0298692) +
               Cm3 * (        -10.127019518324309 +
                      pbar1 * -1376750.5212588238 +
                      pbar2 * 4478.101099579278 +
                      pbar3 * 204220928.71583435 +
                      pbar4 * -381430.6882060992 +
                      pbar5 * 15039251322.72012) +
               Cm4 * (        24.139216589708816 +
                      pbar1 * 2826456.268993324 +
                      pbar2 * -10643.381757411911 +
                      pbar3 * 384081651.0629266 +
                      pbar4 * 905432.2118388592 +
                      pbar5 * -86395877728.4623) +
               Cm5 * (        754.6259086078117 +
                      pbar1 * 149226192.57174134 +
                      pbar2 * -333689.79772201204 +
                      pbar3 * -23146218925.55468 +
                      pbar4 * 28422521.874333758 +
                      pbar5 * -447497635207.89386)) +
        CL3 * (      (        -0.0014142990228769071 +
                      pbar1 * -42.64342815036535 +
                      pbar2 * 0.6247420125769599 +
                      pbar3 * -12046.15937114196 +
                      pbar4 * -53.18889849498145 +
                      pbar5 * 461497.9338526513) +
               Cm1 * (        -0.019712117132601504 +
                      pbar1 * -430.1557122882379 +
                      pbar2 * 8.716633639241984 +
                      pbar3 * -64550.24617032426 +
                      pbar4 * -742.461066598504 +
                      pbar5 * 69360987.11672239) +
               Cm2 * (        0.4202172732260577 +
                      pbar1 * 10773.720592050568 +
                      pbar2 * -185.5441491220027 +
                      pbar3 * 3288445.786360405 +
                      pbar4 * 15793.801096700168 +
                      pbar5 * -550201011.5972304) +
               Cm3 * (        8.401147227722985 +
                      pbar1 * 1142592.3501892264 +
                      pbar2 * -3714.9570689535517 +
                      pbar3 * -148981294.18192884 +
                      pbar4 * 316427.1472524322 +
                      pbar5 * -11670914163.321375) +
               Cm4 * (        -26.141213709866374 +
                      pbar1 * 58923.65245495448 +
                      pbar2 * 11537.80044993952 +
                      pbar3 * -581712514.1146044 +
                      pbar4 * -981943.4010213464 +
                      pbar5 * 65986719428.55054) +
               Cm5 * (        -638.2203732841185 +
                      pbar1 * -111839136.92542861 +
                      pbar2 * 282218.39230247296 +
                      pbar3 * 14716785401.890638 +
                      pbar4 * -24038292.988327194 +
                      pbar5 * 543264494951.23364)));
    
    A[1] = (
              (      (        -5.941032015671703e-06 +
                      pbar1 * -38.88044163082967 +
                      pbar2 * 0.0026427810610130335 +
                      pbar3 * 4226.2923503600505 +
                      pbar4 * -0.22580801929767316 +
                      pbar5 * -500248.98717095033) +
               Cm1 * (        0.001698192138873216 +
                      pbar1 * -18.516764339892415 +
                      pbar2 * -0.7508341253372792 +
                      pbar3 * -44339.31154143386 +
                      pbar4 * 63.9423033352777 +
                      pbar5 * -6227883.153484833) +
               Cm2 * (        0.011695771921516196 +
                      pbar1 * -4980.415196574141 +
                      pbar2 * -5.178205131305145 +
                      pbar3 * -846099.956869683 +
                      pbar4 * 441.32245151975764 +
                      pbar5 * 225349884.93326455) +
               Cm3 * (        -0.5815194534289332 +
                      pbar1 * -64522.34140203089 +
                      pbar2 * 257.1076473298977 +
                      pbar3 * 33141911.069194984 +
                      pbar4 * -21895.769065210374 +
                      pbar5 * 554210595.4206846) +
               Cm4 * (        -1.3092402557762848 +
                      pbar1 * 16849.39362376866 +
                      pbar2 * 579.4419521960635 +
                      pbar3 * 144388333.09143662 +
                      pbar4 * -49374.4322381012 +
                      pbar5 * -22991569437.542603) +
               Cm5 * (        41.21344398915451 +
                      pbar1 * 5783806.226412728 +
                      pbar2 * -18221.53767557881 +
                      pbar3 * -2578429163.347844 +
                      pbar4 * 1551777.3204967286 +
                      pbar5 * -18542128144.25496)) +
        CL1 * (      (        -9.258552701280032e-05 +
                      pbar1 * -122.07084547962059 +
                      pbar2 * 0.04074337495229339 +
                      pbar3 * -16747.423365833012 +
                      pbar4 * -3.4612086586856936 +
                      pbar5 * 3999976.224385459) +
               Cm1 * (        -0.008023617511010052 +
                      pbar1 * 971.0829636565744 +
                      pbar2 * 3.5467889858682726 +
                      pbar3 * 242495.080031273 +
                      pbar4 * -301.9706143818053 +
                      pbar5 * 101079568.74377637) +
               Cm2 * (        -0.022645417127009237 +
                      pbar1 * 58924.28636763977 +
                      pbar2 * 10.093640655269702 +
                      pbar3 * 6551942.468048522 +
                      pbar4 * -863.1637294605645 +
                      pbar5 * -2424427145.2128115) +
               Cm3 * (        2.8518297603997436 +
                      pbar1 * 892906.2736635241 +
                      pbar2 * -1260.616223944178 +
                      pbar3 * -343877693.04757506 +
                      pbar4 * 107330.8223653561 +
                      pbar5 * -12970054575.221476) +
               Cm4 * (        4.86194218884171 +
                      pbar1 * -2285369.1351840883 +
                      pbar2 * -2156.191965880817 +
                      pbar3 * -1117230932.2855208 +
                      pbar4 * 183914.82621526267 +
                      pbar5 * 233108842508.87646) +
               Cm5 * (        -195.13283238292163 +
                      pbar1 * -98204369.65740594 +
                      pbar2 * 86253.02688742324 +
                      pbar3 * 31721935881.91164 +
                      pbar4 * -7343602.6779095 +
                      pbar5 * 400167872728.09656)) +
        CL2 * (      (        0.0007205264204902244 +
                      pbar1 * 320.25064670906886 +
                      pbar2 * -0.3180681915369013 +
                      pbar3 * 63250.82324275203 +
                      pbar4 * 27.06702723874473 +
                      pbar5 * -10850791.901196737) +
               Cm1 * (        0.0021651531901051057 +
                      pbar1 * 6945.658310463788 +
                      pbar2 * -0.9543728944686921 +
                      pbar3 * -2619768.809792196 +
                      pbar4 * 80.98686361099871 +
                      pbar5 * -177682395.52712432) +
               Cm2 * (        -0.16550163774669766 +
                      pbar1 * -92300.00957906034 +
                      pbar2 * 72.96270657849396 +
                      pbar3 * -23250221.03975993 +
                      pbar4 * -6205.267149453423 +
                      pbar5 * 6412602441.183869) +
               Cm3 * (        -0.3600280758153793 +
                      pbar1 * -4104441.7084027254 +
                      pbar2 * 158.07467911942388 +
                      pbar3 * 1153743436.4248128 +
                      pbar4 * -13363.720444171553 +
                      pbar5 * 28796173599.724285) +
               Cm4 * (        6.886932818544247 +
                      pbar1 * 864041.2452651957 +
                      pbar2 * -3028.0213970252394 +
                      pbar3 * 3305379774.410425 +
                      pbar4 * 257199.4170411124 +
                      pbar5 * -607618157991.2551) +
               Cm5 * (        -20.212207156817165 +
                      pbar1 * 373478624.1720113 +
                      pbar2 * 9022.586086802181 +
                      pbar3 * -96043181672.79051 +
                      pbar4 * -775638.0716361444 +
                      pbar5 * -1014216969760.0985)) +
        CL3 * (      (        -0.0006971079634453878 +
                      pbar1 * -79.57778398286196 +
                      pbar2 * 0.3078761220237631 +
                      pbar3 * -81037.25731226169 +
                      pbar4 * -26.20652341315508 +
                      pbar5 * 8791272.517245922) +
               Cm1 * (        0.005692496820138785 +
                      pbar1 * -9215.346064240435 +
                      pbar2 * -2.519234561844556 +
                      pbar3 * 3062170.3018045174 +
                      pbar4 * 214.76903194937256 +
                      pbar5 * 67929585.36482811) +
               Cm2 * (        0.20747680121193307 +
                      pbar1 * 6200.356615943605 +
                      pbar2 * -91.59005676243613 +
                      pbar3 * 27965414.03963912 +
                      pbar4 * 7794.714188017903 +
                      pbar5 * -4774461914.6199665) +
               Cm3 * (        -2.5855803146380314 +
                      pbar1 * 3784224.3462271877 +
                      pbar2 * 1144.0900093997632 +
                      pbar3 * -989088895.1740052 +
                      pbar4 * -97512.28069804137 +
                      pbar5 * -14266947037.64055) +
               Cm4 * (        -12.866835026175266 +
                      pbar1 * 4013107.0944276387 +
                      pbar2 * 5677.4877598174935 +
                      pbar3 * -3147500783.8316646 +
                      pbar4 * -483084.9209833721 +
                      pbar5 * 439968732656.1458) +
               Cm5 * (        227.7265642931762 +
                      pbar1 * -320495669.97064906 +
                      pbar2 * -100756.74818594029 +
                      pbar3 * 76136946726.97841 +
                      pbar4 * 8586548.583368331 +
                      pbar5 * 559949564741.1304)));
    
    A[2] = (
              (      (        -8.44270073036389e-05 +
                      pbar1 * -0.39305111625817507 +
                      pbar2 * 0.03731974710322718 +
                      pbar3 * -5436.0398756359755 +
                      pbar4 * -3.1805949134074893 +
                      pbar5 * 168589.93440341664) +
               Cm1 * (        0.0036593563847051374 +
                      pbar1 * 162.57497831255282 +
                      pbar2 * -1.6174197088205573 +
                      pbar3 * -212093.46967901065 +
                      pbar4 * 137.62788780598189 +
                      pbar5 * 26490332.345730767) +
               Cm2 * (        0.01973326881795026 +
                      pbar1 * -3931.124507316688 +
                      pbar2 * -8.717578606358524 +
                      pbar3 * -1211668.8200860724 +
                      pbar4 * 742.8094376864029 +
                      pbar5 * 11563832.700757284) +
               Cm3 * (        -1.685671589356725 +
                      pbar1 * -22779.448542319384 +
                      pbar2 * 745.1666711909669 +
                      pbar3 * 81338544.96923073 +
                      pbar4 * -63428.221371428735 +
                      pbar5 * -9060500291.771442) +
               Cm4 * (        -1.1765092735372675 +
                      pbar1 * -324299.4653711638 +
                      pbar2 * 519.5003175635042 +
                      pbar3 * 222702666.44558454 +
                      pbar4 * -44256.28967967692 +
                      pbar5 * -6754832794.807587) +
               Cm5 * (        138.38228767266207 +
                      pbar1 * 3533824.0478910035 +
                      pbar2 * -61176.166707449585 +
                      pbar3 * -6305618594.184381 +
                      pbar4 * 5207876.394983289 +
                      pbar5 * 658244520742.5154)) +
        CL1 * (      (        0.001280010077830711 +
                      pbar1 * -596.7441269447825 +
                      pbar2 * -0.5658618177024565 +
                      pbar3 * 73020.9578958156 +
                      pbar4 * 48.22382643501405 +
                      pbar5 * -1866211.3954633705) +
               Cm1 * (        0.01429017685666102 +
                      pbar1 * -4417.574838871626 +
                      pbar2 * -6.329245617318902 +
                      pbar3 * 2792433.5962598007 +
                      pbar4 * 540.9301705051424 +
                      pbar5 * -340772378.9675023) +
               Cm2 * (        -0.38008199054683994 +
                      pbar1 * 77081.52992130227 +
                      pbar2 * 167.97101483391833 +
                      pbar3 * 3954624.3187926384 +
                      pbar4 * -14311.333782906428 +
                      pbar5 * 463225446.514215) +
               Cm3 * (        -4.438040284549368 +
                      pbar1 * 1467059.5037663188 +
                      pbar2 * 1965.720321174363 +
                      pbar3 * -1089411763.5152373 +
                      pbar4 * -167989.23603162583 +
                      pbar5 * 117090462242.22952) +
               Cm4 * (        25.535717556932592 +
                      pbar1 * 2723787.6941842414 +
                      pbar2 * -11282.813361970168 +
                      pbar3 * -1878034725.818806 +
                      pbar4 * 961159.5572905781 +
                      pbar5 * 30919603155.710403) +
               Cm5 * (        293.8235413834938 +
                      pbar1 * -152999071.93010676 +
                      pbar2 * -130152.47734312707 +
                      pbar3 * 88041607981.72586 +
                      pbar4 * 11123634.635139126 +
                      pbar5 * -8718219336348.649)) +
        CL2 * (      (        -0.0035969887430975732 +
                      pbar1 * 1267.3915156070213 +
                      pbar2 * 1.5900580283876824 +
                      pbar3 * -164438.24342905133 +
                      pbar4 * -135.48974918142085 +
                      pbar5 * 2459237.060687319) +
               Cm1 * (        -0.0968270564230222 +
                      pbar1 * 22387.250209186055 +
                      pbar2 * 42.84113566143112 +
                      pbar3 * -7262947.338745658 +
                      pbar4 * -3653.3180032779333 +
                      pbar5 * 906042561.2136388) +
               Cm2 * (        1.0942825646424883 +
                      pbar1 * -133859.75910342674 +
                      pbar2 * -483.58817504284787 +
                      pbar3 * 7939656.978613654 +
                      pbar4 * 41197.560612469795 +
                      pbar5 * -2473985761.817208) +
               Cm3 * (        37.21656503412353 +
                      pbar1 * -7238541.304392469 +
                      pbar2 * -16464.910645270636 +
                      pbar3 * 2792971163.2006044 +
                      pbar4 * 1403716.7965586304 +
                      pbar5 * -308359847324.1395) +
               Cm4 * (        -72.9929778004235 +
                      pbar1 * -15572521.936116714 +
                      pbar2 * 32250.95201683447 +
                      pbar3 * 2973762000.9169106 +
                      pbar4 * -2747127.574841382 +
                      pbar5 * 63630006697.584984) +
               Cm5 * (        -2808.243770494497 +
                      pbar1 * 686482991.031517 +
                      pbar2 * 1242345.6854489683 +
                      pbar3 * -226277401319.55096 +
                      pbar4 * -105906224.94736248 +
                      pbar5 * 22910361383029.797)) +
        CL3 * (      (        0.0025401726381442465 +
                      pbar1 * -639.2521825080285 +
                      pbar2 * -1.1228555462184346 +
                      pbar3 * 110317.44762577963 +
                      pbar4 * 95.67218966071258 +
                      pbar5 * -849810.4276588992) +
               Cm1 * (        0.0888362613406637 +
                      pbar1 * -20349.46166837002 +
                      pbar2 * -39.298910790085706 +
                      pbar3 * 4569603.155280333 +
                      pbar4 * 3350.019771174239 +
                      pbar5 * -614061315.5285162) +
               Cm2 * (        -0.7738994208328026 +
                      pbar1 * 13381.849904365767 +
                      pbar2 * 341.995690300693 +
                      pbar3 * -12306756.571216678 +
                      pbar4 * -29133.357004684716 +
                      pbar5 * 1936333603.8587666) +
               Cm3 * (        -35.05118672304662 +
                      pbar1 * 7109650.66969964 +
                      pbar2 * 15504.625726183722 +
                      pbar3 * -1894139062.537907 +
                      pbar4 * -1321441.1824278347 +
                      pbar5 * 214927021103.6276) +
               Cm4 * (        51.17509450679425 +
                      pbar1 * 18530245.342240196 +
                      pbar2 * -22610.410325807326 +
                      pbar3 * -1416585794.969322 +
                      pbar4 * 1925848.1417823192 +
                      pbar5 * -68180749261.04409) +
               Cm5 * (        2682.455527762369 +
                      pbar1 * -655931291.6410102 +
                      pbar2 * -1186531.026574351 +
                      pbar3 * 156568565851.6467 +
                      pbar4 * 101119458.0219828 +
                      pbar5 * -16145187042772.635)));
    
    A[3] = (
              (      (        9.498411683910307e-05 +
                      pbar1 * -37.72897376913801 +
                      pbar2 * -0.04202389879963079 +
                      pbar3 * -4981.676533443342 +
                      pbar4 * 3.582146661059555 +
                      pbar5 * 446312.79000171955) +
               Cm1 * (        -0.015670420981460743 +
                      pbar1 * 2235.5888240273666 +
                      pbar2 * 6.929001661403265 +
                      pbar3 * -163159.3298033188 +
                      pbar4 * -590.0814346659312 +
                      pbar5 * 2619803.7667357414) +
               Cm2 * (        -0.09107346244968688 +
                      pbar1 * 14020.41052311361 +
                      pbar2 * 40.280801420311896 +
                      pbar3 * 2038019.9305607842 +
                      pbar4 * -3431.781139544821 +
                      pbar5 * -220825799.8568728) +
               Cm3 * (        5.820602803160065 +
                      pbar1 * -504786.25281466544 +
                      pbar2 * -2573.728028367365 +
                      pbar3 * -65417298.11570167 +
                      pbar4 * 219187.6277559282 +
                      pbar5 * 7103084093.567075) +
               Cm4 * (        10.741287377781925 +
                      pbar1 * 656546.5021739182 +
                      pbar2 * -4750.425589633773 +
                      pbar3 * -456285316.59746903 +
                      pbar4 * 404677.65586029756 +
                      pbar5 * 33418215451.51704) +
               Cm5 * (        -421.0968361498125 +
                      pbar1 * 25100489.386149246 +
                      pbar2 * 186199.12066726093 +
                      pbar3 * 8336281874.982679 +
                      pbar4 * -15857482.721012449 +
                      pbar5 * -728646050668.0773)) +
        CL1 * (      (        -0.0009412230950573524 +
                      pbar1 * -25.903130249163244 +
                      pbar2 * 0.41649395430765684 +
                      pbar3 * 9581.58853253889 +
                      pbar4 * -35.511816446926964 +
                      pbar5 * -1684916.5895563404) +
               Cm1 * (        0.0010666837391979625 +
                      pbar1 * -29148.689492085927 +
                      pbar2 * -0.4641816383533479 +
                      pbar3 * 2817758.991631595 +
                      pbar4 * 38.1922614692138 +
                      pbar5 * -88920054.49974468) +
               Cm2 * (        0.9037118193217099 +
                      pbar1 * -59191.538676923934 +
                      pbar2 * -399.72479554207786 +
                      pbar3 * -27886350.654596776 +
                      pbar4 * 34058.02820497363 +
                      pbar5 * 2319521414.684271) +
               Cm3 * (        5.575730112730983 +
                      pbar1 * 5564746.706519122 +
                      pbar2 * -2468.039447799565 +
                      pbar3 * 689370199.4531558 +
                      pbar4 * 210633.28320415053 +
                      pbar5 * -75259469981.18973) +
               Cm4 * (        -104.65761636142022 +
                      pbar1 * -10990522.837231988 +
                      pbar2 * 46287.467629111125 +
                      pbar3 * 5236441130.657511 +
                      pbar4 * -3943327.555006192 +
                      pbar5 * -343602824138.92053) +
               Cm5 * (        -797.1686289040161 +
                      pbar1 * -191487526.66594666 +
                      pbar2 * 352682.2267064701 +
                      pbar3 * -104614533428.98569 +
                      pbar4 * -30068430.168309074 +
                      pbar5 * 8585579656072.722)) +
        CL2 * (      (        0.0015101525726376964 +
                      pbar1 * -207.11304308456414 +
                      pbar2 * -0.6685076759648021 +
                      pbar3 * 16745.232185625577 +
                      pbar4 * 57.0286345842496 +
                      pbar5 * 1751407.4396942998) +
               Cm1 * (        0.23835035960340945 +
                      pbar1 * 53542.463017021146 +
                      pbar2 * -105.41805355054665 +
                      pbar3 * -4864335.556231945 +
                      pbar4 * 8982.200856084894 +
                      pbar5 * 130228257.57843325) +
               Cm2 * (        -1.8355488759772087 +
                      pbar1 * -71091.85503438122 +
                      pbar2 * 811.9429374853055 +
                      pbar3 * 59271033.661632545 +
                      pbar4 * -69184.90254285016 +
                      pbar5 * -4198832270.722236) +
               Cm3 * (        -113.0980110468012 +
                      pbar1 * -6464577.669879605 +
                      pbar2 * 50018.48772852166 +
                      pbar3 * -2153897072.2840195 +
                      pbar4 * -4261331.963926956 +
                      pbar5 * 194819195534.7252) +
               Cm4 * (        208.65029038530724 +
                      pbar1 * 52806318.10080222 +
                      pbar2 * -92285.13288066105 +
                      pbar3 * -12158722473.035069 +
                      pbar4 * 7862301.350552597 +
                      pbar5 * 698032618808.145) +
               Cm5 * (        9637.909655901747 +
                      pbar1 * -166159105.3163679 +
                      pbar2 * -4262354.272185175 +
                      pbar3 * 287523677267.62384 +
                      pbar4 * 363115469.1698549 +
                      pbar5 * -21270364583607.082)) +
        CL3 * (      (        -0.000591408672232959 +
                      pbar1 * 35.901189247257584 +
                      pbar2 * 0.26199656382554415 +
                      pbar3 * -3636.24906361003 +
                      pbar4 * -22.372193669606006 +
                      pbar5 * -1079052.127601138) +
               Cm1 * (        -0.25690387115948016 +
                      pbar1 * -21912.69161386946 +
                      pbar2 * 113.61609573748821 +
                      pbar3 * 1629456.7426573255 +
                      pbar4 * -9679.309976696388 +
                      pbar5 * -14164398.715864949) +
               Cm2 * (        1.0050225082751796 +
                      pbar1 * 198829.22907515214 +
                      pbar2 * -444.59641369280206 +
                      pbar3 * -38862728.663783416 +
                      pbar4 * 37886.28945642671 +
                      pbar5 * 2215212024.3354707) +
               Cm3 * (        115.77665338798998 +
                      pbar1 * -903903.3316054135 +
                      pbar2 * -51200.93606073894 +
                      pbar3 * 1787742055.6965868 +
                      pbar4 * 4361687.27088443 +
                      pbar5 * -138873188913.14838) +
               Cm4 * (        -113.45389272558512 +
                      pbar1 * -51800360.09544956 +
                      pbar2 * 50182.60927532091 +
                      pbar3 * 8119165221.990485 +
                      pbar4 * -4275537.19458668 +
                      pbar5 * -410753744539.746) +
               Cm5 * (        -9530.600304671705 +
                      pbar1 * 540676299.1988469 +
                      pbar2 * 4214757.059491177 +
                      pbar3 * -213536375453.7962 +
                      pbar4 * -359036683.8631507 +
                      pbar5 * 14421194423737.135)));
    
    A[4] = (
              (      (        0.00010411343060966565 +
                      pbar1 * -206.00505828067006 +
                      pbar2 * -0.04607133179173652 +
                      pbar3 * 9853.295273457661 +
                      pbar4 * 3.9306657688780007 +
                      pbar5 * 307811.1027400337) +
               Cm1 * (        0.021352683358529037 +
                      pbar1 * -7427.920844746899 +
                      pbar2 * -9.444564127349555 +
                      pbar3 * 1104966.455929054 +
                      pbar4 * 804.7993511296756 +
                      pbar5 * -37211992.16095941) +
               Cm2 * (        0.08506711899779457 +
                      pbar1 * 15431.108008439774 +
                      pbar2 * -37.61214030789546 +
                      pbar3 * 4399120.479145874 +
                      pbar4 * 3202.3062380844435 +
                      pbar5 * -583920349.0491844) +
               Cm3 * (        -7.19181436655849 +
                      pbar1 * 1814800.3863346893 +
                      pbar2 * 3181.016538814764 +
                      pbar3 * -242739701.74008474 +
                      pbar4 * -271052.65043434396 +
                      pbar5 * 5187226676.68044) +
               Cm4 * (        -14.603100147089146 +
                      pbar1 * -762796.7751370715 +
                      pbar2 * 6457.443134283983 +
                      pbar3 * -599032155.9825666 +
                      pbar4 * -549953.5144231981 +
                      pbar5 * 65048769254.202614) +
               Cm5 * (        479.90334865744273 +
                      pbar1 * -113304567.35918869 +
                      pbar2 * -212269.52570814505 +
                      pbar3 * 11883289171.268345 +
                      pbar4 * 18087495.665371496 +
                      pbar5 * -42480730796.79092)) +
        CL1 * (      (        -0.0015565330506008934 +
                      pbar1 * 2959.043733208802 +
                      pbar2 * 0.6887470844980385 +
                      pbar3 * -314756.1746884528 +
                      pbar4 * -58.75288528401304 +
                      pbar5 * 5436646.402272362) +
               Cm1 * (        0.002753933151224022 +
                      pbar1 * 84075.69328238704 +
                      pbar2 * -1.1899285293607162 +
                      pbar3 * -13151204.623152997 +
                      pbar4 * 96.67217643577126 +
                      pbar5 * 442500692.8384104) +
               Cm2 * (        -0.8212390668261503 +
                      pbar1 * -569466.2305506023 +
                      pbar2 * 363.07940416113297 +
                      pbar3 * 40616455.46766744 +
                      pbar4 * -30907.17012737853 +
                      pbar5 * 2015018085.9404624) +
               Cm3 * (        -21.27881478288586 +
                      pbar1 * -25085651.76105225 +
                      pbar2 * 9400.643499495456 +
                      pbar3 * 3183922385.416999 +
                      pbar4 * -799287.544253963 +
                      pbar5 * -74690812536.23373) +
               Cm4 * (        143.08657842005888 +
                      pbar1 * 38931617.788884535 +
                      pbar2 * -63270.9625250554 +
                      pbar3 * -961925530.8510534 +
                      pbar4 * 5388254.860740006 +
                      pbar5 * -309330474832.11774) +
               Cm5 * (        2621.2391867092038 +
                      pbar1 * 1669201008.8504963 +
                      pbar2 * -1158499.8762197495 +
                      pbar3 * -183051434291.97122 +
                      pbar4 * 98580022.07670964 +
                      pbar5 * 2847678388475.3955)) +
        CL2 * (      (        0.0034814904599630124 +
                      pbar1 * -5994.060248642037 +
                      pbar2 * -1.540584175554572 +
                      pbar3 * 621075.1483579354 +
                      pbar4 * 131.42840752347215 +
                      pbar5 * -5380966.569318894) +
               Cm1 * (        -0.367194746999345 +
                      pbar1 * -245753.0614198298 +
                      pbar2 * 162.31126905677297 +
                      pbar3 * 39277676.99114737 +
                      pbar4 * -13813.985831597247 +
                      pbar5 * -1527298239.8435416) +
               Cm2 * (        2.0347109585864684 +
                      pbar1 * 1400423.2805796862 +
                      pbar2 * -899.5724399249763 +
                      pbar3 * -124855392.48755212 +
                      pbar4 * 76578.31974158894 +
                      pbar5 * -3152418635.1420546) +
               Cm3 * (        201.01930662378496 +
                      pbar1 * 66953297.8213462 +
                      pbar2 * -88871.26826669973 +
                      pbar3 * -8650285382.361639 +
                      pbar4 * 7566339.819914877 +
                      pbar5 * 241990345032.4529) +
               Cm4 * (        -316.6648976249334 +
                      pbar1 * -96140086.02788448 +
                      pbar2 * 140023.97779840583 +
                      pbar3 * 5017223786.248956 +
                      pbar4 * -11924522.780131346 +
                      pbar5 * 564923883558.49) +
               Cm5 * (        -17847.856229151526 +
                      pbar1 * -4365647514.084069 +
                      pbar2 * 7890985.527422789 +
                      pbar3 * 497177839642.4127 +
                      pbar4 * -671894642.4541433 +
                      pbar5 * -10649548179917.623)) +
        CL3 * (      (        -0.0021283219550834926 +
                      pbar1 * 3028.500837347919 +
                      pbar2 * 0.941824749857305 +
                      pbar3 * -270349.79133041884 +
                      pbar4 * -80.35205712988979 +
                      pbar5 * -3933033.7854195274) +
               Cm1 * (        0.393931487088221 +
                      pbar1 * 170833.86954754798 +
                      pbar2 * -174.15838313928757 +
                      pbar3 * -27500598.822366145 +
                      pbar4 * 14827.118396982622 +
                      pbar5 * 1115950629.7201025) +
               Cm2 * (        -1.3229836471756211 +
                      pbar1 * -692681.6762374239 +
                      pbar2 * 584.9100586635836 +
                      pbar3 * 48478878.69257654 +
                      pbar4 * -49792.68164171996 +
                      pbar5 * 3938595494.3397818) +
               Cm3 * (        -194.7218035280769 +
                      pbar1 * -45401121.67292293 +
                      pbar2 * 86094.1718368942 +
                      pbar3 * 5947394355.245323 +
                      pbar4 * -7331023.869018325 +
                      pbar5 * -180205393147.3751) +
               Cm4 * (        191.3248646086092 +
                      pbar1 * 46621273.25702356 +
                      pbar2 * -84600.34126448381 +
                      pbar3 * -1166107388.8830433 +
                      pbar4 * 7204553.393368429 +
                      pbar5 * -483787929789.3996) +
               Cm5 * (        16509.603933517603 +
                      pbar1 * 2945817871.6639285 +
                      pbar2 * -7299739.899709829 +
                      pbar3 * -344180219160.2722 +
                      pbar4 * 621617739.3131106 +
                      pbar5 * 8502742966395.473)));
    
    // set center control surface
    lr[5] = bounds(C);
    // loop thru inboard to outboard control surfaces
    for (i=0; i<5; i++) {
        // set left control surface
        lr[4-i] = bounds( S[i] - A[i] );
        // set right control surface
        lr[6+i] = bounds( S[i] + A[i] );
    }
}

void mode4(struct pilotCommands pilot, double CL1, double *lr) {
    double C, L[5], R[5];
    double CL2, CL3;
    double Cm1, Cm2, Cm3, Cm4, Cm5;
    double pbar1, pbar2, pbar3, pbar4;
    double Cn1, Cn2, Cn3, Cn4;
    int i;
    
    Cm1   = -pwm2frac(pilot.ele) * 0.1;
    pbar1 = -pwm2frac(pilot.ail) * 0.1;
    Cn1   = -pwm2frac(pilat.rud) * 0.02;
    
    CL2 = CL1 * CL1;
    CL3 = CL2 * CL1;
    
    Cm2 = Cm1 * Cm1;
    Cm3 = Cm2 * Cm1;
    Cm4 = Cm3 * Cm1;
    Cm5 = Cm4 * Cm1;
    
    pbar2 = pbar1 * pbar1;
    pbar3 = pbar2 * pbar1;
    pbar4 = pbar3 * pbar1;
    
    Cn2 = Cn1 * Cn1;
    Cn3 = Cn2 * Cn1;
    Cn4 = Cn3 * Cn1;
    
    C = (
              (      (        (      -4.956389604007608 +
                               Cn1 * -52.26944988503714 +
                               Cn2 * -3314.0129115503423 +
                               Cn3 * 148128.723504207 +
                               Cn4 * 53753154.43816934) +
                      pbar1 * (      -1.7459243833290643 +
                               Cn1 * 11419.736276095911 +
                               Cn2 * 6014.92111488778 +
                               Cn3 * 4456288.151359934 +
                               Cn4 * -4589570.417874665) +
                      pbar2 * (      113.11475467313475 +
                               Cn1 * 21800.769592484092 +
                               Cn2 * 14724169.89918938 +
                               Cn3 * -61135707.86751045 +
                               Cn4 * -68818918806.90326) +
                      pbar3 * (      271.39487990082307 +
                               Cn1 * -446785.8541034468 +
                               Cn2 * -1375875.6422883251 +
                               Cn3 * -727464999.1894585 +
                               Cn4 * 2208787226.1529436) +
                      pbar4 * (      -19180.27472821558 +
                               Cn1 * -1823547.8433697103 +
                               Cn2 * -1421397107.8094206 +
                               Cn3 * 5143477184.928847 +
                               Cn4 * 6921602725172.868)) +
               Cm1 * (        (      -26.533155434126066 +
                               Cn1 * -4102.644875472486 +
                               Cn2 * -380660.6149951107 +
                               Cn3 * 11600261.606386082 +
                               Cn4 * -1084448326.6024585) +
                      pbar1 * (      -88.43465602384171 +
                               Cn1 * 67552.8926669258 +
                               Cn2 * 353897.43933060707 +
                               Cn3 * -98132648.23159356 +
                               Cn4 * -529535615.36249924) +
                      pbar2 * (      -21288.732743510347 +
                               Cn1 * 1867665.1280877704 +
                               Cn2 * 375579840.4368837 +
                               Cn3 * -5325647820.744178 +
                               Cn4 * -518249717930.73834) +
                      pbar3 * (      12935.896079547516 +
                               Cn1 * -21573324.61831331 +
                               Cn2 * -56269570.566763476 +
                               Cn3 * 74243267151.28276 +
                               Cn4 * 84794803252.9589) +
                      pbar4 * (      1483756.401941307 +
                               Cn1 * -160783456.5106683 +
                               Cn2 * -40020173504.82049 +
                               Cn3 * 459786326633.79047 +
                               Cn4 * 97643698774273.19)) +
               Cm2 * (        (      61.77514033318849 +
                               Cn1 * -253.16340503814777 +
                               Cn2 * 8731507.714467304 +
                               Cn3 * 660252.3369897584 +
                               Cn4 * -44253142961.3706) +
                      pbar1 * (      126.21981964594028 +
                               Cn1 * -902406.1150434489 +
                               Cn2 * -2437587.2207654933 +
                               Cn3 * -3595585501.3901134 +
                               Cn4 * 5982277453.641706) +
                      pbar2 * (      -7582.143455940056 +
                               Cn1 * 185503.05973803904 +
                               Cn2 * -9333448976.973366 +
                               Cn3 * -737989863.9720933 +
                               Cn4 * 32210374609159.77) +
                      pbar3 * (      -26799.848979881903 +
                               Cn1 * 154506712.35481066 +
                               Cn2 * 463960542.34763914 +
                               Cn3 * -223153892320.1036 +
                               Cn4 * -1200898563040.9517) +
                      pbar4 * (      8792631.0091254 +
                               Cn1 * -18077152.935008556 +
                               Cn2 * 931997111809.8588 +
                               Cn3 * 61403868786.113464 +
                               Cn4 * -3431407393908730.5)) +
               Cm3 * (        (      -7653.9676199327 +
                               Cn1 * 1152928.215025045 +
                               Cn2 * 122338135.95662265 +
                               Cn3 * -3256831105.970004 +
                               Cn4 * 83689957566.22025) +
                      pbar1 * (      25575.421211797595 +
                               Cn1 * 28573634.4261016 +
                               Cn2 * -109653382.67615812 +
                               Cn3 * -209275701794.04163 +
                               Cn4 * 169864619967.06158) +
                      pbar2 * (      5536133.681324058 +
                               Cn1 * -526014869.22056085 +
                               Cn2 * -85993687800.89659 +
                               Cn3 * 1495077929863.1697 +
                               Cn4 * 89102945957553.95) +
                      pbar3 * (      -3769433.516981385 +
                               Cn1 * 1640999891.4248044 +
                               Cn2 * 17792824709.69132 +
                               Cn3 * -3015844120466.254 +
                               Cn4 * -29281291352138.434) +
                      pbar4 * (      -474206770.3629507 +
                               Cn1 * 45423705993.44347 +
                               Cn2 * 11312605397917.062 +
                               Cn3 * -129583301215807.5 +
                               Cn4 * -2.5919570298499028e+16)) +
               Cm4 * (        (      -6469.538314303859 +
                               Cn1 * 689124.4825750869 +
                               Cn2 * -447199734.72919947 +
                               Cn3 * -1947124545.1122744 +
                               Cn4 * 2568440177648.726) +
                      pbar1 * (      7551.387782607115 +
                               Cn1 * 28834055.75259977 +
                               Cn2 * 202515166.0754928 +
                               Cn3 * 248170430314.24136 +
                               Cn4 * -586477210198.2137) +
                      pbar2 * (      7115659.061128457 +
                               Cn1 * -301242709.6236587 +
                               Cn2 * 626554890932.288 +
                               Cn3 * 873689654716.7401 +
                               Cn4 * -2175210446934006.5) +
                      pbar3 * (      -559827.5030623636 +
                               Cn1 * -11230637839.461922 +
                               Cn2 * -33398356282.444576 +
                               Cn3 * 62348479361899.586 +
                               Cn4 * 96456940628634.42) +
                      pbar4 * (      -1458826053.1376512 +
                               Cn1 * 26089979873.106483 +
                               Cn2 * -63575400752115.38 +
                               Cn3 * -75940854898715.38 +
                               Cn4 * 2.6196205605555056e+17)) +
               Cm5 * (        (      549670.1325463393 +
                               Cn1 * -74566071.34043089 +
                               Cn2 * -8064381411.646961 +
                               Cn3 * 210589466362.73074 +
                               Cn4 * -1953933493334.0093) +
                      pbar1 * (      -1675519.7484756496 +
                               Cn1 * -3364700650.973696 +
                               Cn2 * 7332989713.950002 +
                               Cn3 * 20760222971155.996 +
                               Cn4 * -11239503168615.406) +
                      pbar2 * (      -412358360.3197007 +
                               Cn1 * 34111405476.269405 +
                               Cn2 * 5957146402746.883 +
                               Cn3 * -96734067724912.14 +
                               Cn4 * -6003060830852855.0) +
                      pbar3 * (      249741063.0691901 +
                               Cn1 * 57467643067.34888 +
                               Cn2 * -1213536279667.8037 +
                               Cn3 * -199133774451144.03 +
                               Cn4 * 2024081518600389.8) +
                      pbar4 * (      37300529588.34518 +
                               Cn1 * -2955473261430.7095 +
                               Cn2 * -854050633402404.0 +
                               Cn3 * 8411944524188491.0 +
                               Cn4 * 2.0423934048176727e+18))) +
        CL1 * (      (        (      -8.042429198099049 +
                               Cn1 * 300.53289414194063 +
                               Cn2 * -23029.89050984408 +
                               Cn3 * -851209.0096648438 +
                               Cn4 * -405675164.1664703) +
                      pbar1 * (      13.869465039872113 +
                               Cn1 * -7516.051667541328 +
                               Cn2 * -58348.74877618686 +
                               Cn3 * -96459967.3036268 +
                               Cn4 * 54739576.14596925) +
                      pbar2 * (      3061.2791316716293 +
                               Cn1 * -117533.24741470935 +
                               Cn2 * -118031280.10377975 +
                               Cn3 * 323482152.3429667 +
                               Cn4 * 627443330974.6768) +
                      pbar3 * (      -2393.87248201981 +
                               Cn1 * 1286914.5030071153 +
                               Cn2 * 15811424.676404342 +
                               Cn3 * 2706005128.189161 +
                               Cn4 * -29241637972.24407) +
                      pbar4 * (      -53846.697527422926 +
                               Cn1 * 9643190.266123556 +
                               Cn2 * 11379798953.97423 +
                               Cn3 * -26831137152.5026 +
                               Cn4 * -63359720862767.31)) +
               Cm1 * (        (      -150.70542875819277 +
                               Cn1 * 23786.201966243014 +
                               Cn2 * 3829887.3376381975 +
                               Cn3 * -67292581.42319474 +
                               Cn4 * 13835333223.744818) +
                      pbar1 * (      443.64298956724605 +
                               Cn1 * 1196848.0576089201 +
                               Cn2 * -3224824.000232768 +
                               Cn3 * -3697791426.9754443 +
                               Cn4 * 7782066192.52281) +
                      pbar2 * (      138279.89476087457 +
                               Cn1 * -11405462.039396238 +
                               Cn2 * -2651573421.8041563 +
                               Cn3 * 32908733621.68273 +
                               Cn4 * 1810345504044.4507) +
                      pbar3 * (      -76398.9330380497 +
                               Cn1 * 14574517.691192536 +
                               Cn2 * 571925147.1842818 +
                               Cn3 * -127533520849.04639 +
                               Cn4 * -1249608362762.8381) +
                      pbar4 * (      -10098936.576777317 +
                               Cn1 * 1000724198.6254685 +
                               Cn2 * 330589005556.0538 +
                               Cn3 * -2900495210601.736 +
                               Cn4 * -766715755278633.8)) +
               Cm2 * (        (      529.2155874950214 +
                               Cn1 * 2142.803562327785 +
                               Cn2 * 12024668.157460343 +
                               Cn3 * -1494337.0700722341 +
                               Cn4 * 119617829681.69513) +
                      pbar1 * (      -1467.4097870676758 +
                               Cn1 * 2207826.552310857 +
                               Cn2 * 25823672.17757688 +
                               Cn3 * 38218614602.39755 +
                               Cn4 * -58559790917.21314) +
                      pbar2 * (      -884875.9717117316 +
                               Cn1 * -2236847.390834512 +
                               Cn2 * 53472812333.318565 +
                               Cn3 * 7033655871.343472 +
                               Cn4 * -192181976563235.5) +
                      pbar3 * (      339155.2325616468 +
                               Cn1 * -574490099.5666133 +
                               Cn2 * -5398244741.525984 +
                               Cn3 * 1938341325347.044 +
                               Cn4 * 12613000536283.156) +
                      pbar4 * (      -5217151.81559773 +
                               Cn1 * 253188861.9581993 +
                               Cn2 * -6412147859150.932 +
                               Cn3 * -814641973212.3103 +
                               Cn4 * 2.5205779937982604e+16)) +
               Cm3 * (        (      27079.275612218233 +
                               Cn1 * -6693554.087016398 +
                               Cn2 * -861194779.3291962 +
                               Cn3 * 18979446437.117897 +
                               Cn4 * -3052555226313.6904) +
                      pbar1 * (      -132339.09755883992 +
                               Cn1 * -753029641.7089746 +
                               Cn2 * 986301361.4442772 +
                               Cn3 * 2817379510356.117 +
                               Cn4 * -2283734874243.6206) +
                      pbar2 * (      -24477044.662782837 +
                               Cn1 * 3221189639.8837533 +
                               Cn2 * 332199597462.40314 +
                               Cn3 * -9283567689478.732 +
                               Cn4 * 524074301420116.0) +
                      pbar3 * (      23050948.19396246 +
                               Cn1 * 44663434539.70976 +
                               Cn2 * -179921079606.84204 +
                               Cn3 * -108093000785743.25 +
                               Cn4 * 388763500554366.9) +
                      pbar4 * (      2413100519.553876 +
                               Cn1 * -284089255520.3183 +
                               Cn2 * -55225965174347.46 +
                               Cn3 * 822191626051343.6 +
                               Cn4 * 1.2998341130464418e+17)) +
               Cm4 * (        (      84226.13263405468 +
                               Cn1 * -4076962.926177014 +
                               Cn2 * -4553600984.582233 +
                               Cn3 * 11036349851.361912 +
                               Cn4 * 2833186731561.2246) +
                      pbar1 * (      -8507.411422226578 +
                               Cn1 * 480290424.2553834 +
                               Cn2 * -2218371907.9666123 +
                               Cn3 * -4613427191557.051 +
                               Cn4 * 5616759324126.148) +
                      pbar2 * (      -26346454.757404376 +
                               Cn1 * 1817976675.4233403 +
                               Cn2 * -2268708671726.6055 +
                               Cn3 * -5182926465267.846 +
                               Cn4 * 8456207423803399.0) +
                      pbar3 * (      -5592164.734708986 +
                               Cn1 * 20850282143.478348 +
                               Cn2 * 389762158872.51416 +
                               Cn3 * -497034203135839.6 +
                               Cn4 * -945902330586238.4) +
                      pbar4 * (      9946572943.818169 +
                               Cn1 * -164561150930.86133 +
                               Cn2 * 324270743496583.44 +
                               Cn3 * 484753943677094.6 +
                               Cn4 * -1.6500854404766047e+18)) +
               Cm5 * (        (      -1502859.7459840528 +
                               Cn1 * 433791570.42434114 +
                               Cn2 * 37116612028.16664 +
                               Cn3 * -1232643870412.7239 +
                               Cn4 * 230643269763774.2) +
                      pbar1 * (      8804766.22331042 +
                               Cn1 * 62817134823.202065 +
                               Cn2 * -65123954669.36417 +
                               Cn3 * -243030212445930.3 +
                               Cn4 * 144564330495080.25) +
                      pbar2 * (      1753104930.176697 +
                               Cn1 * -209760547089.91248 +
                               Cn2 * -3608421953647.8887 +
                               Cn3 * 603709426073413.9 +
                               Cn4 * -9.447814533736824e+16) +
                      pbar3 * (      -1567739090.1498797 +
                               Cn1 * -4589465556737.112 +
                               Cn2 * 12278573649864.885 +
                               Cn3 * 9504083809713448.0 +
                               Cn4 * -2.5874362173828584e+16) +
                      pbar4 * (      -190542618806.17694 +
                               Cn1 * 18617451435541.188 +
                               Cn2 * 2214733478637501.2 +
                               Cn3 * -5.376947868635173e+16 +
                               Cn4 * -5.706864769359167e+18))) +
        CL2 * (      (        (      6.156112119403691 +
                               Cn1 * -522.8162316318565 +
                               Cn2 * -18061.661602151806 +
                               Cn3 * 1479118.3969455524 +
                               Cn4 * 1402131843.8581386) +
                      pbar1 * (      -30.52616810822332 +
                               Cn1 * 48255.3563112516 +
                               Cn2 * 115040.36719355638 +
                               Cn3 * 132240789.22365604 +
                               Cn4 * -58441971.27744289) +
                      pbar2 * (      -6828.569692272097 +
                               Cn1 * 185723.12848018768 +
                               Cn2 * 399084748.78481406 +
                               Cn3 * -495044283.7835524 +
                               Cn4 * -1835766841129.5786) +
                      pbar3 * (      5347.264849820857 +
                               Cn1 * -6178674.290702493 +
                               Cn2 * -35259992.478731066 +
                               Cn3 * 14002740080.21559 +
                               Cn4 * 63659369562.5802) +
                      pbar4 * (      294268.43829631543 +
                               Cn1 * -14672788.129996922 +
                               Cn2 * -39103764345.26224 +
                               Cn3 * 39588992383.19398 +
                               Cn4 * 188120275503280.03)) +
               Cm1 * (        (      447.6984869787429 +
                               Cn1 * -41934.3101828879 +
                               Cn2 * -7045325.65453022 +
                               Cn3 * 118723501.28356914 +
                               Cn4 * -42846585230.26643) +
                      pbar1 * (      -513.8720853921236 +
                               Cn1 * -4625164.438762681 +
                               Cn2 * 7728220.473482659 +
                               Cn3 * 11232509490.89696 +
                               Cn4 * -23779190122.034645) +
                      pbar2 * (      -339876.5514545348 +
                               Cn1 * 21525067.393539216 +
                               Cn2 * 2724442413.897823 +
                               Cn3 * -63112325608.28048 +
                               Cn4 * 6442409641491.593) +
                      pbar3 * (      109640.91898064977 +
                               Cn1 * 101755619.8701128 +
                               Cn2 * -1292314459.8325481 +
                               Cn3 * -130293658488.0626 +
                               Cn4 * 3301622459641.578) +
                      pbar4 * (      25838516.104693238 +
                               Cn1 * -1928786081.2569804 +
                               Cn2 * -460763028669.5729 +
                               Cn3 * 5674810230504.504 +
                               Cn4 * 677885132636696.4)) +
               Cm2 * (        (      -3804.4829092782325 +
                               Cn1 * -6004.794216482491 +
                               Cn2 * -7547646.581798932 +
                               Cn3 * 1841110.736398163 +
                               Cn4 * -346870482401.6244) +
                      pbar1 * (      3236.1805214762126 +
                               Cn1 * -18778847.330063656 +
                               Cn2 * -57782772.94934919 +
                               Cn3 * -34891564062.08803 +
                               Cn4 * 132735934780.35367) +
                      pbar2 * (      4698150.243998728 +
                               Cn1 * 6338726.1089457115 +
                               Cn2 * -181651089355.1871 +
                               Cn3 * -16577420400.100605 +
                               Cn4 * 603445767220361.2) +
                      pbar3 * (      -746973.4449639278 +
                               Cn1 * 4173195063.7298846 +
                               Cn2 * 12090666515.34947 +
                               Cn3 * -20744922493792.234 +
                               Cn4 * -27354657171374.195) +
                      pbar4 * (      -200851666.1927343 +
                               Cn1 * -789171554.9513979 +
                               Cn2 * 21600446963495.76 +
                               Cn3 * 2418243087435.495 +
                               Cn4 * -8.089414204606678e+16)) +
               Cm3 * (        (      -54611.616451922644 +
                               Cn1 * 11822468.490632005 +
                               Cn2 * 1257739795.8276825 +
                               Cn3 * -33628678402.039375 +
                               Cn4 * 10678607721485.617) +
                      pbar1 * (      161890.52125666908 +
                               Cn1 * 2184396321.141742 +
                               Cn2 * -2317747878.5666614 +
                               Cn3 * -6943447452855.739 +
                               Cn4 * 6763749308472.93) +
                      pbar2 * (      55746322.28457243 +
                               Cn1 * -6098207064.284814 +
                               Cn2 * 452517419625.065 +
                               Cn3 * 17898238420933.92 +
                               Cn4 * -5367495886747806.0) +
                      pbar3 * (      -34699102.74722361 +
                               Cn1 * -138645539606.11053 +
                               Cn2 * 404861371411.8483 +
                               Cn3 * 353038825213614.8 +
                               Cn4 * -990765738439796.0) +
                      pbar4 * (      -6178822788.403149 +
                               Cn1 * 550300925615.6472 +
                               Cn2 * 14474395129980.492 +
                               Cn3 * -1618424465852572.5 +
                               Cn4 * 1.263351664563529e+17)) +
               Cm4 * (        (      62661.07143534171 +
                               Cn1 * 7411376.904432639 +
                               Cn2 * 9081146538.947094 +
                               Cn3 * -19325500493.562874 +
                               Cn4 * 106445572552.17172) +
                      pbar1 * (      21579.83779811544 +
                               Cn1 * 257141668.49254504 +
                               Cn2 * 5204127062.169032 +
                               Cn3 * 3811241933816.6816 +
                               Cn4 * -14056561246761.92) +
                      pbar2 * (      -180895995.11461195 +
                               Cn1 * -3305026971.3931813 +
                               Cn2 * 9584637893255.281 +
                               Cn3 * 8958944709380.98 +
                               Cn4 * -3.3619785774306772e+16) +
                      pbar3 * (      8807995.249820853 +
                               Cn1 * -369376440131.52454 +
                               Cn2 * -864649817020.928 +
                               Cn3 * 3181482775458569.5 +
                               Cn4 * 2083099255298311.0) +
                      pbar4 * (      -3780678700.8158007 +
                               Cn1 * 316765612978.16046 +
                               Cn2 * -1285129103689510.8 +
                               Cn3 * -919256595522262.1 +
                               Cn4 * 5.964299145026178e+18)) +
               Cm5 * (        (      2452239.32832887 +
                               Cn1 * -767952726.3132735 +
                               Cn2 * -33568735686.5629 +
                               Cn3 * 2191473398189.2754 +
                               Cn4 * -801834215065191.6) +
                      pbar1 * (      -11052299.656785315 +
                               Cn1 * -170645666715.65317 +
                               Cn2 * 150590091921.04526 +
                               Cn3 * 573239453894164.4 +
                               Cn4 * -424677546266295.44) +
                      pbar2 * (      -3927926458.997813 +
                               Cn1 * 399064017850.1406 +
                               Cn2 * -80981289340296.9 +
                               Cn3 * -1171497034094901.5 +
                               Cn4 * 5.3638403173258534e+17) +
                      pbar3 * (      2454182878.9379134 +
                               Cn1 * 12709788769532.55 +
                               Cn2 * -27630080475463.426 +
                               Cn3 * -2.851934362086252e+16 +
                               Cn4 * 6.537114303689424e+16) +
                      pbar4 * (      483755124520.4525 +
                               Cn1 * -36362380626982.89 +
                               Cn2 * 3324093859020524.5 +
                               Cn3 * 1.068882124313158e+17 +
                               Cn4 * -1.9828444112152113e+19))) +
        CL3 * (      (        (      -2.91336502193813 +
                               Cn1 * 281.3685516021455 +
                               Cn2 * 45915.06857270484 +
                               Cn3 * -794417.1444301759 +
                               Cn4 * -1114938035.568339) +
                      pbar1 * (      20.230295345727068 +
                               Cn1 * -58098.7885978134 +
                               Cn2 * -58882.55874382727 +
                               Cn3 * -10490657.302791411 +
                               Cn4 * -26674802.252647355) +
                      pbar2 * (      4426.132331452977 +
                               Cn1 * -86413.90238817228 +
                               Cn2 * -319342697.93262315 +
                               Cn3 * 217164236.39337525 +
                               Cn4 * 1377157367380.4312) +
                      pbar3 * (      -3492.017796387728 +
                               Cn1 * 6813714.268680075 +
                               Cn2 * 21279901.422276683 +
                               Cn3 * -23235512817.81802 +
                               Cn4 * -35763993399.924255) +
                      pbar4 * (      -241495.7876516419 +
                               Cn1 * 6358235.247287523 +
                               Cn2 * 31628388077.943516 +
                               Cn3 * -16084638495.290476 +
                               Cn4 * -143134486173255.06)) +
               Cm1 * (        (      -302.2843174045604 +
                               Cn1 * 22955.013834362824 +
                               Cn2 * 3047997.43268196 +
                               Cn3 * -65089347.40863167 +
                               Cn4 * 32280946301.59476) +
                      pbar1 * (      65.3186542658849 +
                               Cn1 * 3750927.3455957365 +
                               Cn2 * -5535865.227181689 +
                               Cn3 * -9166700115.888065 +
                               Cn4 * 20027247019.92494) +
                      pbar2 * (      226147.37409722898 +
                               Cn1 * -12804400.302870044 +
                               Cn2 * 226929244.38700303 +
                               Cn3 * 38268786622.27132 +
                               Cn4 * -10244723466333.926) +
                      pbar3 * (      -34933.33957984893 +
                               Cn1 * -117721791.55067007 +
                               Cn2 * 833369915.3038348 +
                               Cn3 * 330484942423.6799 +
                               Cn4 * -2398116829815.9155) +
                      pbar4 * (      -17510561.473082837 +
                               Cn1 * 1172859506.8801482 +
                               Cn2 * 109761124060.24455 +
                               Cn3 * -3504202625354.8706 +
                               Cn4 * 228809020521543.72)) +
               Cm2 * (        (      3453.4619261782746 +
                               Cn1 * 4631.525406875964 +
                               Cn2 * -19688923.93381288 +
                               Cn3 * -1926215.0519770216 +
                               Cn4 * 303712051256.5338) +
                      pbar1 * (      -1948.9079078816064 +
                               Cn1 * 23169018.288299486 +
                               Cn2 * 36500780.563931495 +
                               Cn3 * -7754444614.814001 +
                               Cn4 * -86644322594.42494) +
                      pbar2 * (      -4121170.532944178 +
                               Cn1 * -4847315.631430365 +
                               Cn2 * 155105532901.8132 +
                               Cn3 * 11401635070.446804 +
                               Cn4 * -491801911208138.06) +
                      pbar3 * (      431133.3484048792 +
                               Cn1 * -4791274164.171222 +
                               Cn2 * -7350053478.449074 +
                               Cn3 * 22753521081834.734 +
                               Cn4 * 16405444045635.176) +
                      pbar4 * (      222069329.18349522 +
                               Cn1 * 650362912.9555346 +
                               Cn2 * -18246068178838.652 +
                               Cn3 * -1943312469098.7346 +
                               Cn4 * 6.5822658604568856e+16)) +
               Cm3 * (        (      33643.063537739894 +
                               Cn1 * -6484092.294212554 +
                               Cn2 * -316359630.31472796 +
                               Cn3 * 18499936119.423206 +
                               Cn4 * -8307909057482.253) +
                      pbar1 * (      -29577.290672454503 +
                               Cn1 * -1631464315.0096397 +
                               Cn2 * 1634250463.8867579 +
                               Cn3 * 5135121786939.033 +
                               Cn4 * -5636837104997.271) +
                      pbar2 * (      -38004689.61519334 +
                               Cn1 * 3639674348.5973644 +
                               Cn2 * -989640485032.2262 +
                               Cn3 * -10904039521986.787 +
                               Cn4 * 5752907590682343.0) +
                      pbar3 * (      12581523.369041229 +
                               Cn1 * 108483790684.53383 +
                               Cn2 * -261542603871.94925 +
                               Cn3 * -359358181413552.06 +
                               Cn4 * 710198007556478.9) +
                      pbar4 * (      4381618482.80156 +
                               Cn1 * -336411936010.6111 +
                               Cn2 * 56004129467968.125 +
                               Cn3 * 1005518737227314.8 +
                               Cn4 * -3.4450943716543917e+17)) +
               Cm4 * (        (      -167634.70058692648 +
                               Cn1 * -4189086.6741150157 +
                               Cn2 * -3550252086.3882723 +
                               Cn3 * 10634009927.284283 +
                               Cn4 * -7453189078782.336) +
                      pbar1 * (      -36182.298100360094 +
                               Cn1 * -1246128658.7092052 +
                               Cn2 * -3530435905.7861357 +
                               Cn3 * 1359730829292.0608 +
                               Cn4 * 10342217278019.03) +
                      pbar2 * (      231004012.33294 +
                               Cn1 * 1844095007.2323053 +
                               Cn2 * -9500051300646.559 +
                               Cn3 * -4695028251815.354 +
                               Cn4 * 3.1342501001364416e+16) +
                      pbar3 * (      2175466.46757596 +
                               Cn1 * 461110682622.0 +
                               Cn2 * 519799119748.64264 +
                               Cn3 * -3164984963308145.0 +
                               Cn4 * -1285123841847582.8) +
                      pbar4 * (      -7280298973.78535 +
                               Cn1 * -189556676611.1053 +
                               Cn2 * 1224416986516458.5 +
                               Cn3 * 540399764504787.5 +
                               Cn4 * -5.210157570422378e+18)) +
               Cm5 * (        (      -1297304.5850462613 +
                               Cn1 * 422249888.7376083 +
                               Cn2 * -10894609962.428295 +
                               Cn3 * -1209053420991.176 +
                               Cn4 * 610423619259787.9) +
                      pbar1 * (      2280550.71531589 +
                               Cn1 * 124196024112.16745 +
                               Cn2 * -104578764742.77771 +
                               Cn3 * -413429557600011.8 +
                               Cn4 * 353569014463483.0) +
                      pbar2 * (      2657444495.535351 +
                               Cn1 * -239431828745.8853 +
                               Cn2 * 101350648522998.53 +
                               Cn3 * 718216184156147.9 +
                               Cn4 * -5.12778810146516e+17) +
                      pbar3 * (      -977263570.9551365 +
                               Cn1 * -9548757510482.664 +
                               Cn2 * 17920752354041.324 +
                               Cn3 * 2.9232892432785456e+16 +
                               Cn4 * -4.696172380225063e+16) +
                      pbar4 * (      -341464484258.6876 +
                               Cn1 * 22424657107679.484 +
                               Cn2 * -6794241560801215.0 +
                               Cn3 * -6.709063692585487e+16 +
                               Cn4 * 3.270310329957935e+19))));
    
    R[0] = (
              (      (        (      -4.537077288017728 +
                               Cn1 * -169.97008395021146 +
                               Cn2 * 16312.809565676089 +
                               Cn3 * 1860699.2351383837 +
                               Cn4 * 68617777.64317293) +
                      pbar1 * (      -18.435858859166835 +
                               Cn1 * 4980.327638921468 +
                               Cn2 * 347287.1485489601 +
                               Cn3 * 28582442.993808832 +
                               Cn4 * 659520161.0557717) +
                      pbar2 * (      622.3415780847745 +
                               Cn1 * 173944.146419273 +
                               Cn2 * 1494016.2784799933 +
                               Cn3 * -1686476017.0023384 +
                               Cn4 * -87726062750.36772) +
                      pbar3 * (      43.731760027861284 +
                               Cn1 * 444486.440841304 +
                               Cn2 * -28527135.355079446 +
                               Cn3 * -7363922855.173504 +
                               Cn4 * -89177694723.4573) +
                      pbar4 * (      -63192.10699175465 +
                               Cn1 * -15774336.489718867 +
                               Cn2 * 191968698.75941846 +
                               Cn3 * 153093529354.06158 +
                               Cn4 * 6021352894411.727)) +
               Cm1 * (        (      -23.06644903896275 +
                               Cn1 * 3098.7216042590044 +
                               Cn2 * -662136.4613970915 +
                               Cn3 * -31768250.59314449 +
                               Cn4 * 1169366369.774419) +
                      pbar1 * (      218.7460156110027 +
                               Cn1 * 80853.62136492209 +
                               Cn2 * -8931743.946898963 +
                               Cn3 * -1029336676.6786968 +
                               Cn4 * -6371306185.941747) +
                      pbar2 * (      -9488.2056995742 +
                               Cn1 * 3015366.6012843275 +
                               Cn2 * 584825613.480583 +
                               Cn3 * -21851174746.33639 +
                               Cn4 * -2859434318529.73) +
                      pbar3 * (      -56210.699497865586 +
                               Cn1 * -18320226.621645015 +
                               Cn2 * 1567061335.4984221 +
                               Cn3 * 200563408990.3791 +
                               Cn4 * 1552552188754.5183) +
                      pbar4 * (      414259.2707791669 +
                               Cn1 * -337972559.665639 +
                               Cn2 * -47737095672.29849 +
                               Cn3 * 2930749952335.139 +
                               Cn4 * 306461881768732.44)) +
               Cm2 * (        (      745.0464966884368 +
                               Cn1 * 41315.13117622092 +
                               Cn2 * -13847501.3635789 +
                               Cn3 * -838391631.8980601 +
                               Cn4 * -5311283853.231485) +
                      pbar1 * (      970.974464501073 +
                               Cn1 * 3320896.738766734 +
                               Cn2 * -1354960.284293891 +
                               Cn3 * -33248307822.819935 +
                               Cn4 * -854398045409.9484) +
                      pbar2 * (      -396072.2356104707 +
                               Cn1 * -54180381.36359778 +
                               Cn2 * 5207466793.033847 +
                               Cn3 * 563123002169.3922 +
                               Cn4 * 10250102918794.586) +
                      pbar3 * (      304330.25500491797 +
                               Cn1 * -696261627.8120726 +
                               Cn2 * -7494561790.103804 +
                               Cn3 * 6643866239455.241 +
                               Cn4 * 146809022342736.72) +
                      pbar4 * (      45109159.91462862 +
                               Cn1 * 3722037312.7016406 +
                               Cn2 * -771710752616.7185 +
                               Cn3 * -38683845212595.875 +
                               Cn4 * 1057334018987493.1)) +
               Cm3 * (        (      5924.747671904893 +
                               Cn1 * -912652.0818977841 +
                               Cn2 * -41408667.502646625 +
                               Cn3 * 7450709911.869993 +
                               Cn4 * 162814208203.63806) +
                      pbar1 * (      -115554.31396069401 +
                               Cn1 * 50980406.47527222 +
                               Cn2 * 6566173236.055304 +
                               Cn3 * -186583219559.84482 +
                               Cn4 * -21141352955107.945) +
                      pbar2 * (      -2479584.5935518295 +
                               Cn1 * -329066668.5070437 +
                               Cn2 * 7802546301.051802 +
                               Cn3 * 4179587125717.771 +
                               Cn4 * 337967526220029.5) +
                      pbar3 * (      24835410.439357407 +
                               Cn1 * -5688092541.90345 +
                               Cn2 * -1120896112302.1604 +
                               Cn3 * 15922589792172.262 +
                               Cn4 * 2953829120587828.5) +
                      pbar4 * (      361151680.08694357 +
                               Cn1 * 26202732817.91382 +
                               Cn2 * -4129683929197.4404 +
                               Cn3 * -486938550210420.5 +
                               Cn4 * -2.8069925044318916e+16)) +
               Cm4 * (        (      -61674.18853012218 +
                               Cn1 * -1192556.0073171542 +
                               Cn2 * 1374987832.4469004 +
                               Cn3 * 48871736438.94352 +
                               Cn4 * -906812081152.8951) +
                      pbar1 * (      -174523.01413664245 +
                               Cn1 * -430836216.7299107 +
                               Cn2 * -2083280444.7640069 +
                               Cn3 * 3688681785026.926 +
                               Cn4 * 85458413945301.58) +
                      pbar2 * (      42945855.33359831 +
                               Cn1 * 2278426495.5564184 +
                               Cn2 * -837365411592.2711 +
                               Cn3 * -32090153231442.625 +
                               Cn4 * 896953843453643.4) +
                      pbar3 * (      -28302802.664694197 +
                               Cn1 * 83025108896.94073 +
                               Cn2 * 1492848945427.0234 +
                               Cn3 * -742261637468013.9 +
                               Cn4 * -1.7848543113726352e+16) +
                      pbar4 * (      -4960048065.833208 +
                               Cn1 * -45203554286.83856 +
                               Cn2 * 112841902220168.53 +
                               Cn3 * 1406869840504206.8 +
                               Cn4 * -3.122537003479645e+17)) +
               Cm5 * (        (      -629950.1740700303 +
                               Cn1 * 74616671.56464048 +
                               Cn2 * 10885390911.531475 +
                               Cn3 * -473126692437.5698 +
                               Cn4 * -29173597308572.92) +
                      pbar1 * (      8471218.863974346 +
                               Cn1 * -6543280316.49125 +
                               Cn2 * -577161724938.7377 +
                               Cn3 * 36128295643634.01 +
                               Cn4 * 2330238710490382.5) +
                      pbar2 * (      357056984.71954286 +
                               Cn1 * 116053616.46120413 +
                               Cn2 * -7454517093132.228 +
                               Cn3 * -186745807228412.12 +
                               Cn4 * 328735265761713.75) +
                      pbar3 * (      -1825263763.7590475 +
                               Cn1 * 943169371444.7134 +
                               Cn2 * 101131291310989.88 +
                               Cn3 * -5482217820036304.0 +
                               Cn4 * -3.6619601921303104e+17) +
                      pbar4 * (      -42665929971.24946 +
                               Cn1 * 1515968717513.3064 +
                               Cn2 * 1020121422797773.2 +
                               Cn3 * 1.3502388328899842e+16 +
                               Cn4 * -1.2521479954337728e+18))) +
        CL1 * (      (        (      14.673449289106214 +
                               Cn1 * -57.56456494682726 +
                               Cn2 * -304197.2541280417 +
                               Cn3 * -13001452.057754545 +
                               Cn4 * -21839153.00471908) +
                      pbar1 * (      88.55417084173175 +
                               Cn1 * -7308.747727686948 +
                               Cn2 * -5012956.383183447 +
                               Cn3 * -53244345.65063938 +
                               Cn4 * 5226901508.874538) +
                      pbar2 * (      -2862.3258205581287 +
                               Cn1 * -1053644.0330680907 +
                               Cn2 * 10923517.457304828 +
                               Cn3 * 12281701894.703611 +
                               Cn4 * 710445777782.1354) +
                      pbar3 * (      -6252.378013566135 +
                               Cn1 * -2330879.9496630603 +
                               Cn2 * 618056263.1241899 +
                               Cn3 * 31632990034.038082 +
                               Cn4 * -1343055605738.2593) +
                      pbar4 * (      326463.4721559727 +
                               Cn1 * 130196943.7114638 +
                               Cn2 * -890482690.7530295 +
                               Cn3 * -1309671583955.3816 +
                               Cn4 * -59930483630538.31)) +
               Cm1 * (        (      -28.536732580814746 +
                               Cn1 * -55458.37281439091 +
                               Cn2 * 3587726.7514261124 +
                               Cn3 * 660900955.9991132 +
                               Cn4 * 16552954345.861488) +
                      pbar1 * (      -3473.0586657003482 +
                               Cn1 * 813246.3298617481 +
                               Cn2 * 184839178.1476607 +
                               Cn3 * 3524661682.5807047 +
                               Cn4 * -420633356751.51044) +
                      pbar2 * (      59913.53331121985 +
                               Cn1 * -8023218.427896336 +
                               Cn2 * -2836709547.8489504 +
                               Cn3 * 31168902728.366695 +
                               Cn4 * 11096234480481.01) +
                      pbar3 * (      615283.9187411896 +
                               Cn1 * 58958648.53802019 +
                               Cn2 * -22790887759.85203 +
                               Cn3 * -1615036743460.542 +
                               Cn4 * 23835040225258.45) +
                      pbar4 * (      -1988411.1187493282 +
                               Cn1 * 1837528241.643916 +
                               Cn2 * 279800217624.21704 +
                               Cn3 * -16065161045697.594 +
                               Cn4 * -1883912333694864.0)) +
               Cm2 * (        (      -5844.324612187072 +
                               Cn1 * -24316.918181647095 +
                               Cn2 * 184416802.02519488 +
                               Cn3 * 5616638879.2705765 +
                               Cn4 * -137522813200.3397) +
                      pbar1 * (      -52935.8697847058 +
                               Cn1 * -12086206.74136164 +
                               Cn2 * 1883890709.548603 +
                               Cn3 * 147385786504.32135 +
                               Cn4 * -45556816275.90144) +
                      pbar2 * (      2172985.646860287 +
                               Cn1 * 401007903.1548246 +
                               Cn2 * -40597308310.2012 +
                               Cn3 * -4122758361479.694 +
                               Cn4 * -125454868487782.55) +
                      pbar3 * (      -57647.27107448202 +
                               Cn1 * 4055840295.912589 +
                               Cn2 * -135469066992.66571 +
                               Cn3 * -41600041891010.09 +
                               Cn4 * -410814363026764.44) +
                      pbar4 * (      -246485480.36256602 +
                               Cn1 * -38725632839.55254 +
                               Cn2 * 4477630358019.961 +
                               Cn3 * 335167119723994.1 +
                               Cn4 * 1007784079462493.8)) +
               Cm3 * (        (      -64453.32674447475 +
                               Cn1 * 17027117.290733088 +
                               Cn2 * 589394748.8221772 +
                               Cn3 * -158909056194.70953 +
                               Cn4 * -6937410408677.186) +
                      pbar1 * (      1335483.1453695109 +
                               Cn1 * -722967955.8035778 +
                               Cn2 * -82937535822.01192 +
                               Cn3 * 1444052710293.7817 +
                               Cn4 * 249569317307220.84) +
                      pbar2 * (      39885335.4437461 +
                               Cn1 * -1090202873.316444 +
                               Cn2 * -682781458518.1132 +
                               Cn3 * -12066500192310.793 +
                               Cn4 * -80439460436055.03) +
                      pbar3 * (      -238118507.84194273 +
                               Cn1 * 72978756170.33704 +
                               Cn2 * 12852868951192.896 +
                               Cn3 * -42098420618105.65 +
                               Cn4 * -3.4229603086556436e+16) +
                      pbar4 * (      -4509809707.59439 +
                               Cn1 * 46878362194.28742 +
                               Cn2 * 85006649062043.86 +
                               Cn3 * 3253207228186764.5 +
                               Cn4 * 9.76730617579939e+16)) +
               Cm4 * (        (      512608.53498365777 +
                               Cn1 * -3795442.790699989 +
                               Cn2 * -16159801013.713644 +
                               Cn3 * -366398607020.0135 +
                               Cn4 * 15792683888080.822) +
                      pbar1 * (      5479205.584202014 +
                               Cn1 * 2359237473.3499703 +
                               Cn2 * -146851834286.41495 +
                               Cn3 * -21260036294661.527 +
                               Cn4 * -162342323723111.16) +
                      pbar2 * (      -285795182.7074453 +
                               Cn1 * -18415077610.35066 +
                               Cn2 * 7067006685386.989 +
                               Cn3 * 250313669785557.75 +
                               Cn4 * -3503656205832451.5) +
                      pbar3 * (      -85234988.59239472 +
                               Cn1 * -562581412246.873 +
                               Cn2 * 5976110620623.409 +
                               Cn3 * 5287625833596870.0 +
                               Cn4 * 8.754976011154616e+16) +
                      pbar4 * (      32476549568.172264 +
                               Cn1 * 1141953282189.359 +
                               Cn2 * -833039900322392.0 +
                               Cn3 * -1.4414219122628906e+16 +
                               Cn4 * 1.847179505048665e+18)) +
               Cm5 * (        (      6793360.816936167 +
                               Cn1 * -1280698438.6262183 +
                               Cn2 * -107412832703.8641 +
                               Cn3 * 10311870996498.895 +
                               Cn4 * 595705875011424.6) +
                      pbar1 * (      -90767709.2619365 +
                               Cn1 * 72168086806.58325 +
                               Cn2 * 6571304418755.748 +
                               Cn3 * -262946592184938.72 +
                               Cn4 * -2.2586831144798548e+16) +
                      pbar2 * (      -4501915181.675782 +
                               Cn1 * 265602986773.8823 +
                               Cn2 * 110016144679984.9 +
                               Cn3 * 518928367165743.44 +
                               Cn4 * -1.727674465940386e+17) +
                      pbar3 * (      17240882155.770164 +
                               Cn1 * -9778095306746.81 +
                               Cn2 * -1106607073124076.1 +
                               Cn3 * 3.923023316566318e+16 +
                               Cn4 * 3.702423470681174e+18) +
                      pbar4 * (      476033255997.4132 +
                               Cn1 * -33575849825818.46 +
                               Cn2 * -1.2882756199075284e+16 +
                               Cn3 * -1.0121472557650288e+17 +
                               Cn4 * 1.944962634124893e+19))) +
        CL2 * (      (        (      -25.79530577232455 +
                               Cn1 * 143.46403962280826 +
                               Cn2 * 676744.4980759607 +
                               Cn3 * 37715458.577992074 +
                               Cn4 * 361415742.88237524) +
                      pbar1 * (      -352.4901845105631 +
                               Cn1 * 42724.782654234135 +
                               Cn2 * 16695560.89764844 +
                               Cn3 * 59674935.572956815 +
                               Cn4 * -23761521117.292553) +
                      pbar2 * (      8519.13015947066 +
                               Cn1 * 2669557.0061831623 +
                               Cn2 * 52118852.09527056 +
                               Cn3 * -27402510654.49427 +
                               Cn4 * -1785383148784.8066) +
                      pbar3 * (      32910.035608197235 +
                               Cn1 * 2438152.9025957836 +
                               Cn2 * -2175123657.386004 +
                               Cn3 * -56491724278.611305 +
                               Cn4 * 5574058494814.695) +
                      pbar4 * (      -664904.2247246582 +
                               Cn1 * -335270782.9583546 +
                               Cn2 * -6596675630.035956 +
                               Cn3 * 3007582386709.257 +
                               Cn4 * 156048452183926.56)) +
               Cm1 * (        (      203.0968502798565 +
                               Cn1 * 163541.2286240323 +
                               Cn2 * -6994073.488035556 +
                               Cn3 * -1806976767.9197378 +
                               Cn4 * -55889434456.61804) +
                      pbar1 * (      6528.264741260431 +
                               Cn1 * -3449768.2049607453 +
                               Cn2 * -435097606.29049927 +
                               Cn3 * -4437140220.908622 +
                               Cn4 * 985559976749.6257) +
                      pbar2 * (      -231339.77230118797 +
                               Cn1 * -11952149.824854184 +
                               Cn2 * 3919988419.79106 +
                               Cn3 * 99819872892.65134 +
                               Cn4 * -10774356248386.562) +
                      pbar3 * (      -1147176.0812593277 +
                               Cn1 * -11768139.208734604 +
                               Cn2 * 48538272213.295715 +
                               Cn3 * 3396653922552.415 +
                               Cn4 * -43989262487972.46) +
                      pbar4 * (      13478568.606802408 +
                               Cn1 * -1716929587.6451714 +
                               Cn2 * -417958784854.209 +
                               Cn3 * 23010291809936.965 +
                               Cn4 * 3038867127551758.5)) +
               Cm2 * (        (      10303.073005556624 +
                               Cn1 * 444018.8900691704 +
                               Cn2 * -329482465.44388336 +
                               Cn3 * -14558551043.705818 +
                               Cn4 * -11443778759.930946) +
                      pbar1 * (      174333.50064537616 +
                               Cn1 * 1749917.4959915825 +
                               Cn2 * -7203796047.663989 +
                               Cn3 * -179541087930.82928 +
                               Cn4 * 11088721241956.285) +
                      pbar2 * (      -2241696.2142224545 +
                               Cn1 * -1151990177.0076814 +
                               Cn2 * 12957397353.458391 +
                               Cn3 * 9754931222723.492 +
                               Cn4 * 527777573788956.3) +
                      pbar3 * (      -4588995.251348209 +
                               Cn1 * -5526980237.677972 +
                               Cn2 * 719077862600.871 +
                               Cn3 * 71098178570367.08 +
                               Cn4 * -802533662274906.8) +
                      pbar4 * (      302485791.8285335 +
                               Cn1 * 128156773667.44643 +
                               Cn2 * -1639605385975.3506 +
                               Cn3 * -900595871402211.8 +
                               Cn4 * -3.057314725722423e+16)) +
               Cm3 * (        (      123908.15322652605 +
                               Cn1 * -50440412.40673661 +
                               Cn2 * -1731315794.2301726 +
                               Cn3 * 475896691089.22064 +
                               Cn4 * 22719049564297.03) +
                      pbar1 * (      -2395957.898821101 +
                               Cn1 * 1837443362.606212 +
                               Cn2 * 183490169073.90176 +
                               Cn3 * -2489329930199.506 +
                               Cn4 * -504975344680564.5) +
                      pbar2 * (      -74898668.47948435 +
                               Cn1 * 12098536661.331772 +
                               Cn2 * 2442013649396.323 +
                               Cn3 * -29413876446286.637 +
                               Cn4 * -4785246907342798.0) +
                      pbar3 * (      436441650.7017163 +
                               Cn1 * -173604457465.25052 +
                               Cn2 * -27669447016597.164 +
                               Cn3 * -17271140514322.756 +
                               Cn4 * 7.042727973548334e+16) +
                      pbar4 * (      8272960926.094301 +
                               Cn1 * -934099190061.067 +
                               Cn2 * -265513126720611.94 +
                               Cn3 * -2741726461606362.5 +
                               Cn4 * 1.7568139843480336e+17)) +
               Cm4 * (        (      -806975.8674393097 +
                               Cn1 * -31648545.538019367 +
                               Cn2 * 28314891958.753975 +
                               Cn3 * 1016656470213.7944 +
                               Cn4 * -5320556686010.011) +
                      pbar1 * (      -15932424.414061436 +
                               Cn1 * -2693601429.465855 +
                               Cn2 * 571791697072.9885 +
                               Cn3 * 29202109499689.17 +
                               Cn4 * -722792355707411.4) +
                      pbar2 * (      381805508.6652567 +
                               Cn1 * 72254846839.1591 +
                               Cn2 * -9021022258986.27 +
                               Cn3 * -693754103032068.0 +
                               Cn4 * -1.7433000696162074e+16) +
                      pbar3 * (      464969237.99560285 +
                               Cn1 * 865380148506.8129 +
                               Cn2 * -47549794740931.11 +
                               Cn3 * -8884051882062983.0 +
                               Cn4 * -2.223351636707215e+16) +
                      pbar4 * (      -49787793258.24445 +
                               Cn1 * -7170612502726.264 +
                               Cn2 * 1139564313536392.5 +
                               Cn3 * 5.665139627422032e+16 +
                               Cn4 * -1.1014864441495046e+18)) +
               Cm5 * (        (      -14164238.277862646 +
                               Cn1 * 3714077490.8380346 +
                               Cn2 * 260604823326.9909 +
                               Cn3 * -31443282719191.973 +
                               Cn4 * -1798559285687776.5) +
                      pbar1 * (      152609944.31975207 +
                               Cn1 * -171018488773.01614 +
                               Cn2 * -14107709087601.492 +
                               Cn3 * 508217226024461.56 +
                               Cn4 * 4.454108269922847e+16) +
                      pbar2 * (      9099639945.127825 +
                               Cn1 * -1235704617491.1533 +
                               Cn2 * -303982130986465.5 +
                               Cn3 * 2574200340226384.5 +
                               Cn4 * 7.130383958708769e+17) +
                      pbar3 * (      -30899188588.82952 +
                               Cn1 * 21870190455409.688 +
                               Cn2 * 2355992880444254.5 +
                               Cn3 * -7.67298227008369e+16 +
                               Cn4 * -7.660880632363866e+18) +
                      pbar4 * (      -933938563007.7721 +
                               Cn1 * 129981033204217.31 +
                               Cn2 * 3.3377865932385044e+16 +
                               Cn3 * -7.283873984968109e+16 +
                               Cn4 * -6.8671802278013846e+19))) +
        CL3 * (      (        (      15.26253984633742 +
                               Cn1 * 380.8804588425901 +
                               Cn2 * -386803.31298261206 +
                               Cn3 * -29342958.607966427 +
                               Cn4 * -578625436.6497014) +
                      pbar1 * (      304.73078490833683 +
                               Cn1 * -44790.488256475655 +
                               Cn2 * -13165356.648316821 +
                               Cn3 * -38664812.32438976 +
                               Cn4 * 20783842967.504547) +
                      pbar2 * (      -6509.495107045685 +
                               Cn1 * -1859556.6213566163 +
                               Cn2 * -70605988.97085544 +
                               Cn3 * 18047080878.998768 +
                               Cn4 * 1251911061196.114) +
                      pbar3 * (      -28562.62242528509 +
                               Cn1 * 64354.89569625192 +
                               Cn2 * 1763107882.6242778 +
                               Cn3 * 33086384367.726322 +
                               Cn4 * -4650440109805.241) +
                      pbar4 * (      441461.069027428 +
                               Cn1 * 233269874.7916087 +
                               Cn2 * 7909653904.967823 +
                               Cn3 * -1972574296941.1003 +
                               Cn4 * -109257712877477.34)) +
               Cm1 * (        (      -166.44196614160148 +
                               Cn1 * -111590.97264734695 +
                               Cn2 * 4527866.933490386 +
                               Cn3 * 1211519484.2853847 +
                               Cn4 * 38928267420.62312) +
                      pbar1 * (      -3094.8953082359153 +
                               Cn1 * 2733104.279683155 +
                               Cn2 * 278753484.46087706 +
                               Cn3 * 2428347750.921533 +
                               Cn4 * -596085673080.09) +
                      pbar2 * (      194573.3221917264 +
                               Cn1 * 19314521.142822467 +
                               Cn2 * -1654450852.3466804 +
                               Cn3 * -112316394887.75548 +
                               Cn4 * 2833842338715.997) +
                      pbar3 * (      597682.6134741007 +
                               Cn1 * -34417034.75716745 +
                               Cn2 * -29723175125.151962 +
                               Cn3 * -2126195849889.3828 +
                               Cn4 * 22539702012601.758) +
                      pbar4 * (      -13450217.24065447 +
                               Cn1 * -1001304.8249135199 +
                               Cn2 * 185383338133.69626 +
                               Cn3 * -10072873356352.305 +
                               Cn4 * -1544858500386375.2)) +
               Cm2 * (        (      -5251.6613739584445 +
                               Cn1 * -499382.0929143942 +
                               Cn2 * 158259491.5096924 +
                               Cn3 * 10584546531.036865 +
                               Cn4 * 205572983554.81146) +
                      pbar1 * (      -133923.85839699302 +
                               Cn1 * 10390476.443996103 +
                               Cn2 * 5998601531.575826 +
                               Cn3 * 63716203061.212006 +
                               Cn4 * -11559941567974.844) +
                      pbar2 * (      457665.30887216283 +
                               Cn1 * 875021946.8928839 +
                               Cn2 * 29887895259.199547 +
                               Cn3 * -6532262334065.692 +
                               Cn4 * -451668333410451.25) +
                      pbar3 * (      5185081.982836046 +
                               Cn1 * 1752419599.3916168 +
                               Cn2 * -668728829516.2523 +
                               Cn3 * -35886158483239.83 +
                               Cn4 * 1311343211109817.0) +
                      pbar4 * (      -101723114.93198295 +
                               Cn1 * -103575519401.14082 +
                               Cn2 * -2853104681819.9507 +
                               Cn3 * 649414080554848.0 +
                               Cn4 * 3.219044474372232e+16)) +
               Cm3 * (        (      -66161.44170409601 +
                               Cn1 * 35862016.82389276 +
                               Cn2 * 1235160392.721369 +
                               Cn3 * -334242350127.1599 +
                               Cn4 * -16494991905124.287) +
                      pbar1 * (      1278997.8000103435 +
                               Cn1 * -1278954197.5068948 +
                               Cn2 * -119174675023.68114 +
                               Cn3 * 1406731908434.9773 +
                               Cn4 * 310935802215673.3) +
                      pbar2 * (      35755387.266634084 +
                               Cn1 * -12749239705.094374 +
                               Cn2 * -1963330521424.5012 +
                               Cn3 * 41739093605062.414 +
                               Cn4 * 5005428998222345.0) +
                      pbar3 * (      -240532536.8245596 +
                               Cn1 * 117034062341.85088 +
                               Cn2 * 17403593826392.547 +
                               Cn3 * 28608175606130.74 +
                               Cn4 * -4.4333075199086776e+16) +
                      pbar4 * (      -4028236168.4143767 +
                               Cn1 * 1084707409048.3575 +
                               Cn2 * 205523298963523.9 +
                               Cn3 * -625550916112108.2 +
                               Cn4 * -2.897722788538645e+17)) +
               Cm4 * (        (      347123.5954737605 +
                               Cn1 * 39966440.82539211 +
                               Cn2 * -13323422175.471397 +
                               Cn3 * -750279669252.1315 +
                               Cn4 * -13287538581033.379) +
                      pbar1 * (      11568655.813456336 +
                               Cn1 * 511943253.9918081 +
                               Cn2 * -484384824165.1103 +
                               Cn3 * -11551172150571.514 +
                               Cn4 * 928566253194900.4) +
                      pbar2 * (      -128871092.78329158 +
                               Cn1 * -63382871016.96986 +
                               Cn2 * 2063702493884.6604 +
                               Cn3 * 501333767059820.8 +
                               Cn4 * 2.3094880770854852e+16) +
                      pbar3 * (      -412246097.9028841 +
                               Cn1 * -352742089933.20557 +
                               Cn2 * 48326492667395.25 +
                               Cn3 * 4287663463436486.5 +
                               Cn4 * -7.040023222845641e+16) +
                      pbar4 * (      21745184590.78043 +
                               Cn1 * 7155544593172.834 +
                               Cn2 * -346220898983456.06 +
                               Cn3 * -4.7784279909289384e+16 +
                               Cn4 * -7.425982511874006e+17)) +
               Cm5 * (        (      8237516.766880701 +
                               Cn1 * -2627035767.306494 +
                               Cn2 * -173180381598.96536 +
                               Cn3 * 22154056992840.676 +
                               Cn4 * 1273535626566320.0) +
                      pbar1 * (      -78522665.04273139 +
                               Cn1 * 115185017617.45854 +
                               Cn2 * 9078846644282.008 +
                               Cn3 * -306270506465177.56 +
                               Cn4 * -2.720867360929813e+16) +
                      pbar2 * (      -5002304334.841068 +
                               Cn1 * 1150291387432.116 +
                               Cn2 * 222125067951807.28 +
                               Cn3 * -3260416759391560.0 +
                               Cn4 * -5.915808981034678e+17) +
                      pbar3 * (      16720887808.432669 +
                               Cn1 * -14125460209862.926 +
                               Cn2 * -1469694228926053.5 +
                               Cn3 * 4.700041437028376e+16 +
                               Cn4 * 4.812343938882866e+18) +
                      pbar4 * (      511136835705.5648 +
                               Cn1 * -118547036178496.33 +
                               Cn2 * -2.3729579793931844e+16 +
                               Cn3 * 2.220486134583661e+17 +
                               Cn4 * 5.620600182011429e+19))));
    
    R[1] = (
              (      (        (      -4.0673884895449905 +
                               Cn1 * -331.61682428722537 +
                               Cn2 * -32219.465263403017 +
                               Cn3 * -611403.3599408455 +
                               Cn4 * 34791868.04827002) +
                      pbar1 * (      12.601422406747055 +
                               Cn1 * -1161.7414229102192 +
                               Cn2 * -1492102.273659604 +
                               Cn3 * -42991765.94065411 +
                               Cn4 * 1310265068.0302057) +
                      pbar2 * (      95.82611389858512 +
                               Cn1 * 39746.302720623666 +
                               Cn2 * 16296878.307687838 +
                               Cn3 * 1073478546.4116987 +
                               Cn4 * 3371324818.3320236) +
                      pbar3 * (      -3207.01221239482 +
                               Cn1 * 849975.1847707741 +
                               Cn2 * 154858299.71597007 +
                               Cn3 * 2150251826.709116 +
                               Cn4 * -139479694345.01328) +
                      pbar4 * (      -18206.265525129314 +
                               Cn1 * -10910.37916291226 +
                               Cn2 * -885071576.1275958 +
                               Cn3 * -104847009707.50294 +
                               Cn4 * -1973809470129.6665)) +
               Cm1 * (        (      -14.963207312524881 +
                               Cn1 * -12565.470755056402 +
                               Cn2 * -1918306.9724894923 +
                               Cn3 * 50476267.92463478 +
                               Cn4 * 6508836107.715789) +
                      pbar1 * (      -470.3003727514684 +
                               Cn1 * 108605.42998804201 +
                               Cn2 * 8813441.31920473 +
                               Cn3 * -257344168.62999782 +
                               Cn4 * -3071200595.6302047) +
                      pbar2 * (      -28445.192968514984 +
                               Cn1 * 5291319.770556093 +
                               Cn2 * 1341776616.1749725 +
                               Cn3 * 12380203524.807951 +
                               Cn4 * -2753556418349.1333) +
                      pbar3 * (      22497.3137924002 +
                               Cn1 * -20574671.725931596 +
                               Cn2 * -1681053539.8858616 +
                               Cn3 * 8677262850.510199 +
                               Cn4 * -125654059061.19664) +
                      pbar4 * (      1331808.4345660978 +
                               Cn1 * -514277574.0574533 +
                               Cn2 * -106897403851.08844 +
                               Cn3 * -1073011678557.4727 +
                               Cn4 * 231524081921305.53)) +
               Cm2 * (        (      -63.0818446783093 +
                               Cn1 * 59449.42652939128 +
                               Cn2 * 11626118.678925855 +
                               Cn3 * 381639548.6028041 +
                               Cn4 * -3928706016.909195) +
                      pbar1 * (      -14526.294411423256 +
                               Cn1 * 7994633.16228093 +
                               Cn2 * 1103066028.2485168 +
                               Cn3 * 5937919323.1468115 +
                               Cn4 * -1387101828608.6494) +
                      pbar2 * (      56010.17179479018 +
                               Cn1 * -11463440.59115915 +
                               Cn2 * -6296117160.957017 +
                               Cn3 * -285540299459.4357 +
                               Cn4 * 4502444277008.886) +
                      pbar3 * (      1389928.9836540879 +
                               Cn1 * -870341303.4023834 +
                               Cn2 * -97604855378.76427 +
                               Cn3 * -285684897024.8791 +
                               Cn4 * 60302330028696.55) +
                      pbar4 * (      11155460.45791352 +
                               Cn1 * 1558556696.089053 +
                               Cn2 * 220409188295.22565 +
                               Cn3 * 9655478248154.037 +
                               Cn4 * 15247573472748.871)) +
               Cm3 * (        (      -101.71399943832772 +
                               Cn1 * 4940890.235701423 +
                               Cn2 * 485725437.1249895 +
                               Cn3 * -20966900569.71235 +
                               Cn4 * -1923687925775.515) +
                      pbar1 * (      194509.14979002724 +
                               Cn1 * 2044923.7693842484 +
                               Cn2 * -463238906.2977339 +
                               Cn3 * 240464113966.6234 +
                               Cn4 * 8885506629374.096) +
                      pbar2 * (      5414758.505239253 +
                               Cn1 * -2441458145.9391494 +
                               Cn2 * -383866862391.6503 +
                               Cn3 * 2516542517435.4927 +
                               Cn4 * 1064420046810081.8) +
                      pbar3 * (      -19154140.67142177 +
                               Cn1 * 1395794820.8640647 +
                               Cn2 * 183139946969.82175 +
                               Cn3 * -22391942165998.848 +
                               Cn4 * -1093168498622072.9) +
                      pbar4 * (      -264231535.52250388 +
                               Cn1 * 257677865166.96365 +
                               Cn2 * 34741866307655.39 +
                               Cn3 * -389138104808236.0 +
                               Cn4 * -1.0350430109607958e+17)) +
               Cm4 * (        (      -5757.643480939057 +
                               Cn1 * 1399358.7964982204 +
                               Cn2 * 268496697.5361951 +
                               Cn3 * -18046537607.034374 +
                               Cn4 * -1684219182486.0752) +
                      pbar1 * (      778726.1171831931 +
                               Cn1 * -774964439.0566406 +
                               Cn2 * -91852067489.60843 +
                               Cn3 * -384078593064.77246 +
                               Cn4 * 102846511326512.81) +
                      pbar2 * (      -117569.28162529165 +
                               Cn1 * 258171554.93336707 +
                               Cn2 * 203838010597.25833 +
                               Cn3 * 5077475079077.466 +
                               Cn4 * -539695466575742.5) +
                      pbar3 * (      -83266193.62525722 +
                               Cn1 * 82338313906.53505 +
                               Cn2 * 8249077743595.753 +
                               Cn3 * 41847812728769.93 +
                               Cn4 * -949996295932075.0) +
                      pbar4 * (      -1433569148.5743284 +
                               Cn1 * -208881146097.14917 +
                               Cn2 * 2507796922092.4106 +
                               Cn3 * 1667810904376295.5 +
                               Cn4 * 4.322140665568179e+16)) +
               Cm5 * (        (      -305300.1477065523 +
                               Cn1 * -307689221.37469965 +
                               Cn2 * -20524385243.239086 +
                               Cn3 * 1576237037557.556 +
                               Cn4 * 108015479480094.39) +
                      pbar1 * (      -13416552.325190146 +
                               Cn1 * -1975557883.7960413 +
                               Cn2 * -186387033880.92697 +
                               Cn3 * -24685022635500.598 +
                               Cn4 * -763609480466738.5) +
                      pbar2 * (      -258057438.3975619 +
                               Cn1 * 194492257319.65152 +
                               Cn2 * 24095183560327.11 +
                               Cn3 * -441443449352573.0 +
                               Cn4 * -7.806000228972661e+16) +
                      pbar3 * (      1575481122.474336 +
                               Cn1 * 228517055397.59512 +
                               Cn2 * 18825723151545.055 +
                               Cn3 * 2401125201147764.0 +
                               Cn4 * 1.0753574475190565e+17) +
                      pbar4 * (      14236829891.954382 +
                               Cn1 * -21819229445178.098 +
                               Cn2 * -2418717098197531.0 +
                               Cn3 * 5.950274606813028e+16 +
                               Cn4 * 8.167176235384227e+18))) +
        CL1 * (      (        (      1.6824067101447173 +
                               Cn1 * 612.0258247024889 +
                               Cn2 * 82209.89226672852 +
                               Cn3 * 8272623.826377411 +
                               Cn4 * 226972446.349961) +
                      pbar1 * (      -285.90841635730175 +
                               Cn1 * 102541.43184456018 +
                               Cn2 * 15253411.041373076 +
                               Cn3 * 132521208.92827334 +
                               Cn4 * -23280310998.794624) +
                      pbar2 * (      -2732.6253374028033 +
                               Cn1 * 814842.2629930936 +
                               Cn2 * 88882031.698349 +
                               Cn3 * -7298868851.098745 +
                               Cn4 * -359449033606.943) +
                      pbar3 * (      16618.53170381305 +
                               Cn1 * -13740883.146269007 +
                               Cn2 * -1455726922.9899333 +
                               Cn3 * 8700412195.472551 +
                               Cn4 * 2602780887678.3354) +
                      pbar4 * (      361337.31303826417 +
                               Cn1 * -114954723.29968414 +
                               Cn2 * -13152495409.398996 +
                               Cn3 * 892811517067.0815 +
                               Cn4 * 52969274062651.16)) +
               Cm1 * (        (      67.12291713276839 +
                               Cn1 * 139355.100637644 +
                               Cn2 * 10219929.967117054 +
                               Cn3 * -912961229.1902405 +
                               Cn4 * -56589758269.07513) +
                      pbar1 * (      -1289.1123604225131 +
                               Cn1 * 2190618.4574256707 +
                               Cn2 * 332528186.8525147 +
                               Cn3 * 5453043427.017842 +
                               Cn4 * -583594801703.3993) +
                      pbar2 * (      145409.46668716465 +
                               Cn1 * -16328027.74698962 +
                               Cn2 * -4982044747.305733 +
                               Cn3 * -15091954683.074793 +
                               Cn4 * 13323684686853.3) +
                      pbar3 * (      682053.5980492949 +
                               Cn1 * -116156076.60727033 +
                               Cn2 * -31218176636.180107 +
                               Cn3 * -441933547333.3349 +
                               Cn4 * 80794672612589.92) +
                      pbar4 * (      -1060555.904455524 +
                               Cn1 * 1521100200.3251867 +
                               Cn2 * 332455939844.74457 +
                               Cn3 * 5361539937179.26 +
                               Cn4 * -959524545525589.4)) +
               Cm2 * (        (      -1834.5982930468288 +
                               Cn1 * 759551.7050086698 +
                               Cn2 * 125771564.29223011 +
                               Cn3 * -4422507444.773518 +
                               Cn4 * -443781967213.87897) +
                      pbar1 * (      77129.32012685278 +
                               Cn1 * -84052198.01114248 +
                               Cn2 * -9435810062.471102 +
                               Cn3 * 28039166125.28426 +
                               Cn4 * 15731068705435.504) +
                      pbar2 * (      1682543.9946225572 +
                               Cn1 * -862782506.7677261 +
                               Cn2 * -113646690576.14601 +
                               Cn3 * 2534372191164.6304 +
                               Cn4 * 251736356514418.12) +
                      pbar3 * (      -10798928.182950452 +
                               Cn1 * 8580921228.363402 +
                               Cn2 * 819977628388.1257 +
                               Cn3 * -5108003349140.702 +
                               Cn4 * -850433785959296.8) +
                      pbar4 * (      -297961552.07231593 +
                               Cn1 * 75374555145.50435 +
                               Cn2 * 12984230074948.957 +
                               Cn3 * -152928434676146.16 +
                               Cn4 * -2.9126121037340332e+16)) +
               Cm3 * (        (      -85671.08912908679 +
                               Cn1 * -40419092.796000965 +
                               Cn2 * -1582837357.130789 +
                               Cn3 * 279118637683.3182 +
                               Cn4 * 13958649206314.15) +
                      pbar1 * (      -562105.8728142965 +
                               Cn1 * -814145084.0515679 +
                               Cn2 * -99627800253.21996 +
                               Cn3 * -2517520555194.475 +
                               Cn4 * 117615480552964.16) +
                      pbar2 * (      -11373228.412256055 +
                               Cn1 * 9999123329.866669 +
                               Cn2 * 1262877377388.1926 +
                               Cn3 * -26491906015450.543 +
                               Cn4 * -5081265203817103.0) +
                      pbar3 * (      -18184991.6783553 +
                               Cn1 * 85947517176.52223 +
                               Cn2 * 12246790926819.344 +
                               Cn3 * 211633293835587.62 +
                               Cn4 * -1.9643704875219988e+16) +
                      pbar4 * (      -859284116.9886225 +
                               Cn1 * -942021197126.2987 +
                               Cn2 * -93868380628455.48 +
                               Cn3 * 2104658153344242.2 +
                               Cn4 * 4.3701691664451226e+17)) +
               Cm4 * (        (      291899.80703238375 +
                               Cn1 * -118851575.64797182 +
                               Cn2 * -20387375772.652576 +
                               Cn3 * 301083522787.7867 +
                               Cn4 * 52534618181046.31) +
                      pbar1 * (      -3628572.5061690607 +
                               Cn1 * 8193074627.967087 +
                               Cn2 * 841041905729.9879 +
                               Cn3 * -2138589717899.1162 +
                               Cn4 * -1268475470704011.2) +
                      pbar2 * (      -166777532.2348847 +
                               Cn1 * 85249659484.55731 +
                               Cn2 * 12165372404057.82 +
                               Cn3 * -111040580837105.77 +
                               Cn4 * -2.0969340692115892e+16) +
                      pbar3 * (      687258979.1573979 +
                               Cn1 * -799425286497.0964 +
                               Cn2 * -71293215941887.31 +
                               Cn3 * 75087235088235.14 +
                               Cn4 * 2.8766035590628204e+16) +
                      pbar4 * (      28171611694.049324 +
                               Cn1 * -6164932527529.468 +
                               Cn2 * -1271200189011099.2 +
                               Cn3 * -4109408477008082.5 +
                               Cn4 * 2.220146184288208e+18)) +
               Cm5 * (        (      9086797.752368653 +
                               Cn1 * 2321835064.7554655 +
                               Cn2 * -18081148457.536785 +
                               Cn3 * -19687182264332.12 +
                               Cn4 * -713736211478799.4) +
                      pbar1 * (      68278546.67344834 +
                               Cn1 * 68291474735.53066 +
                               Cn2 * 7872125124982.339 +
                               Cn3 * 215156206632158.3 +
                               Cn4 * -6634282172223734.0) +
                      pbar2 * (      -151925288.27532548 +
                               Cn1 * -768804644685.8745 +
                               Cn2 * -55392339150473.195 +
                               Cn3 * 3354610990817565.0 +
                               Cn4 * 3.473948015470354e+17) +
                      pbar3 * (      -4802271765.930937 +
                               Cn1 * -9063297582989.654 +
                               Cn2 * -1075394682415401.9 +
                               Cn3 * -1.6762628658442324e+16 +
                               Cn4 * 1.1783689612861094e+18) +
                      pbar4 * (      73927222596.37233 +
                               Cn1 * 77760332057407.8 +
                               Cn2 * 4824939030293014.0 +
                               Cn3 * -3.24256412276957e+17 +
                               Cn4 * -3.1753326212073652e+19))) +
        CL2 * (      (        (      4.366667498673346 +
                               Cn1 * -1387.7019614101944 +
                               Cn2 * -365020.8369515469 +
                               Cn3 * -24999886.432956003 +
                               Cn4 * -469292603.4934385) +
                      pbar1 * (      464.50118226538996 +
                               Cn1 * -191364.16648788739 +
                               Cn2 * -28259931.459011134 +
                               Cn3 * -514202175.0812682 +
                               Cn4 * 31210893167.81048) +
                      pbar2 * (      10807.70962050191 +
                               Cn1 * -9807.303372935596 +
                               Cn2 * -97882741.99279162 +
                               Cn3 * 7278614874.8213215 +
                               Cn4 * 288919517402.9132) +
                      pbar3 * (      -4284.13862359945 +
                               Cn1 * 33644652.81162298 +
                               Cn2 * 2978049514.548659 +
                               Cn3 * -14710403248.915028 +
                               Cn4 * -4757017848919.661) +
                      pbar4 * (      -967636.0893443489 +
                               Cn1 * 139201970.20686212 +
                               Cn2 * 22909571569.521782 +
                               Cn3 * -1221581024783.273 +
                               Cn4 * -74763894874281.33)) +
               Cm1 * (        (      -164.80185390933264 +
                               Cn1 * -258662.02530471812 +
                               Cn2 * -13391718.934638217 +
                               Cn3 * 2156019744.9492846 +
                               Cn4 * 109652226337.50845) +
                      pbar1 * (      4263.6366875036165 +
                               Cn1 * -9229381.444656607 +
                               Cn2 * -1098657190.2572956 +
                               Cn3 * -3396576123.730926 +
                               Cn4 * 2213265161987.033) +
                      pbar2 * (      -387633.15233684145 +
                               Cn1 * -46280241.19363491 +
                               Cn2 * 1713132896.2944298 +
                               Cn3 * 142096403443.65756 +
                               Cn4 * -7479609833570.347) +
                      pbar3 * (      -2209509.1081425324 +
                               Cn1 * 530645201.67224985 +
                               Cn2 * 99814296085.32008 +
                               Cn3 * 312379603068.4733 +
                               Cn4 * -268671964032401.53) +
                      pbar4 * (      5603114.3988963 +
                               Cn1 * 3516098389.6018596 +
                               Cn2 * 34847483010.23324 +
                               Cn3 * -26228192162384.77 +
                               Cn4 * 66730904340147.695)) +
               Cm2 * (        (      3389.785097609286 +
                               Cn1 * -2539265.0515505024 +
                               Cn2 * -351575766.2059707 +
                               Cn3 * 12641588951.895184 +
                               Cn4 * 1188146411523.421) +
                      pbar1 * (      -96576.0477875093 +
                               Cn1 * 171871109.7196503 +
                               Cn2 * 17763824316.032635 +
                               Cn3 * -81057013424.08362 +
                               Cn4 * -30681133026957.57) +
                      pbar2 * (      -3527045.6842356357 +
                               Cn1 * 2153747913.9063296 +
                               Cn2 * 285019051633.3531 +
                               Cn3 * -4389993551252.561 +
                               Cn4 * -512894317809030.3) +
                      pbar3 * (      21068924.538817704 +
                               Cn1 * -16880109964.191269 +
                               Cn2 * -1471471709639.239 +
                               Cn3 * 9089748190891.486 +
                               Cn4 * 1119725080940522.9) +
                      pbar4 * (      675723965.3978251 +
                               Cn1 * -176875931097.17868 +
                               Cn2 * -31332010393432.844 +
                               Cn3 * 172467441441644.66 +
                               Cn4 * 5.900762643669221e+16)) +
               Cm3 * (        (      249097.09039859808 +
                               Cn1 * 70192145.97192754 +
                               Cn2 * -865959768.9850373 +
                               Cn3 * -695719811713.9064 +
                               Cn4 * -24405247177911.723) +
                      pbar1 * (      1056431.1292794452 +
                               Cn1 * 3072422852.674082 +
                               Cn2 * 342175993407.77075 +
                               Cn3 * 3000800879835.463 +
                               Cn4 * -603852250201237.4) +
                      pbar2 * (      37059835.037296765 +
                               Cn1 * 6896974988.919677 +
                               Cn2 * 550491281232.8665 +
                               Cn3 * 24750906473832.5 +
                               Cn4 * 3908208728264442.0) +
                      pbar3 * (      176662941.1605791 +
                               Cn1 * -287046212638.36334 +
                               Cn2 * -38899514348771.69 +
                               Cn3 * -228814722863962.56 +
                               Cn4 * 7.940296647962886e+16) +
                      pbar4 * (      380440651.9392093 +
                               Cn1 * -554192832313.621 +
                               Cn2 * -74195817566041.31 +
                               Cn3 * 316148274820439.3 +
                               Cn4 * -2.6171176996977725e+17)) +
               Cm4 * (        (      -550518.5119226226 +
                               Cn1 * 382770739.63971317 +
                               Cn2 * 56521746300.265305 +
                               Cn3 * -963631925436.215 +
                               Cn4 * -143779342931650.6) +
                      pbar1 * (      2112703.164018632 +
                               Cn1 * -16835182746.21557 +
                               Cn2 * -1566937541130.6313 +
                               Cn3 * 9196463101289.432 +
                               Cn4 * 2566864188824429.0) +
                      pbar2 * (      304404474.8081382 +
                               Cn1 * -237012715138.46164 +
                               Cn2 * -32041818519573.15 +
                               Cn3 * 253239133296606.22 +
                               Cn4 * 5.097096354819841e+16) +
                      pbar3 * (      -1467746765.2208123 +
                               Cn1 * 1462563196346.3062 +
                               Cn2 * 118823823203237.47 +
                               Cn3 * 116713447841327.9 +
                               Cn4 * 3805563041238568.0) +
                      pbar4 * (      -61287490093.17014 +
                               Cn1 * 16354788856232.094 +
                               Cn2 * 3178139250572751.5 +
                               Cn3 * 1.2379797102065844e+16 +
                               Cn4 * -5.065723601792141e+18)) +
               Cm5 * (        (      -24835692.86997448 +
                               Cn1 * -3598133229.771573 +
                               Cn2 * 368714252424.3084 +
                               Cn3 * 49563659216504.33 +
                               Cn4 * 1136908293133459.2) +
                      pbar1 * (      -140506916.75007454 +
                               Cn1 * -234218338197.41782 +
                               Cn2 * -25584413408060.13 +
                               Cn3 * -289157655912290.56 +
                               Cn4 * 3.895347435894349e+16) +
                      pbar2 * (      -338178892.63601094 +
                               Cn1 * -478050324563.9023 +
                               Cn2 * -126367022197721.4 +
                               Cn3 * -4667652568908510.0 +
                               Cn4 * -2.1715990519136182e+17) +
                      pbar3 * (      3187888289.257236 +
                               Cn1 * 27232474943534.734 +
                               Cn2 * 3239676248299887.5 +
                               Cn3 * 1.601013025673877e+16 +
                               Cn4 * -5.423151122145701e+18) +
                      pbar4 * (      -46323036887.97211 +
                               Cn1 * 38558057026548.09 +
                               Cn2 * 1.12109273110315e+16 +
                               Cn3 * 2.8294169308648906e+17 +
                               Cn4 * 1.4138623817623507e+19))) +
        CL3 * (      (        (      -5.38743090118902 +
                               Cn1 * 1474.6364597843171 +
                               Cn2 * 377778.28093269636 +
                               Cn3 * 19751947.721153524 +
                               Cn4 * 172611642.8199458) +
                      pbar1 * (      -197.56065366913046 +
                               Cn1 * 92062.20266227667 +
                               Cn2 * 14349901.190332834 +
                               Cn3 * 418899812.3960749 +
                               Cn4 * -8323970403.494919) +
                      pbar2 * (      -8439.845397804525 +
                               Cn1 * -1064845.2545521862 +
                               Cn2 * -35288560.56300542 +
                               Cn3 * -1202510428.6941793 +
                               Cn4 * 113168040111.50467) +
                      pbar3 * (      -15133.402861976716 +
                               Cn1 * -22725395.82208426 +
                               Cn2 * -1750544333.499133 +
                               Cn3 * 7833982008.228394 +
                               Cn4 * 2424037498841.576) +
                      pbar4 * (      613952.7729317924 +
                               Cn1 * -19157126.757520054 +
                               Cn2 * -7528455533.334095 +
                               Cn3 * 468350109689.92816 +
                               Cn4 * 22005079378753.81)) +
               Cm1 * (        (      77.26889032716201 +
                               Cn1 * 127470.95941230774 +
                               Cn2 * 4298848.293745189 +
                               Cn3 * -1320125589.9560993 +
                               Cn4 * -58972285283.44521) +
                      pbar1 * (      -2100.1410701953987 +
                               Cn1 * 7447613.0982745215 +
                               Cn2 * 796696906.0183177 +
                               Cn3 * -3406910045.9664483 +
                               Cn4 * -1780767018828.0635) +
                      pbar2 * (      301842.63364359556 +
                               Cn1 * 70825682.79781806 +
                               Cn2 * 2792718810.769377 +
                               Cn3 * -183292788900.67834 +
                               Cn4 * -5869142279680.999) +
                      pbar3 * (      1681888.7504466842 +
                               Cn1 * -398469830.1270695 +
                               Cn2 * -69804930918.38719 +
                               Cn3 * 192904835441.54016 +
                               Cn4 * 202933845172260.12) +
                      pbar4 * (      -6874208.881861898 +
                               Cn1 * -5530030545.474524 +
                               Cn2 * -332460378464.5442 +
                               Cn3 * 26309580798844.07 +
                               Cn4 * 909437153557868.8)) +
               Cm2 * (        (      -1333.7040456122359 +
                               Cn1 * 1850975.2189243843 +
                               Cn2 * 218122654.68603256 +
                               Cn3 * -9412367136.077723 +
                               Cn4 * -767590065776.1561) +
                      pbar1 * (      34620.414535870514 +
                               Cn1 * -96831839.47254209 +
                               Cn2 * -9499692557.756931 +
                               Cn3 * 44776661803.221436 +
                               Cn4 * 16282580867525.67) +
                      pbar2 * (      1990922.8277454365 +
                               Cn1 * -1319353649.62905 +
                               Cn2 * -171021452130.4154 +
                               Cn3 * 2208694582064.8613 +
                               Cn4 * 262800638790618.6) +
                      pbar3 * (      -11258448.662627107 +
                               Cn1 * 9394604976.051632 +
                               Cn2 * 753503808260.8645 +
                               Cn3 * -2674183488623.7886 +
                               Cn4 * -241953154158696.94) +
                      pbar4 * (      -410564707.3882674 +
                               Cn1 * 103790695444.49413 +
                               Cn2 * 18997123703190.094 +
                               Cn3 * -20520121552894.566 +
                               Cn4 * -3.0720122800940852e+16)) +
               Cm3 * (        (      -164851.4149205597 +
                               Cn1 * -30838943.132260032 +
                               Cn2 * 2530245709.728388 +
                               Cn3 * 444004876494.3947 +
                               Cn4 * 11593177162574.479) +
                      pbar1 * (      -755695.9760989523 +
                               Cn1 * -2341068840.0725417 +
                               Cn2 * -247671655456.2513 +
                               Cn3 * -484617467169.7374 +
                               Cn4 * 503679309204015.3) +
                      pbar2 * (      -38623702.54852771 +
                               Cn1 * -17395056062.39225 +
                               Cn2 * -1629920823386.3376 +
                               Cn3 * 7414784778518.453 +
                               Cn4 * 639478330228019.1) +
                      pbar3 * (      -147206078.71186963 +
                               Cn1 * 202072067680.21594 +
                               Cn2 * 26799333328714.48 +
                               Cn3 * 26392647122924.426 +
                               Cn4 * -6.096987443387358e+16) +
                      pbar4 * (      1287207275.3798058 +
                               Cn1 * 1460610962308.8936 +
                               Cn2 * 147438953186053.72 +
                               Cn3 * -2876216357495010.0 +
                               Cn4 * -1.1550775048988275e+17)) +
               Cm4 * (        (      235470.37784083947 +
                               Cn1 * -280068334.94684416 +
                               Cn2 * -37081954057.07526 +
                               Cn3 * 747818433237.9236 +
                               Cn4 * 95839826307704.92) +
                      pbar1 * (      802366.4054196284 +
                               Cn1 * 9670038765.575518 +
                               Cn2 * 831617871546.1997 +
                               Cn3 * -6825028203079.378 +
                               Cn4 * -1413635775623137.5) +
                      pbar2 * (      -141642185.16115186 +
                               Cn1 * 158734206949.81412 +
                               Cn2 * 20528168831722.61 +
                               Cn3 * -154336941815661.53 +
                               Cn4 * -3.066014380603491e+16) +
                      pbar3 * (      857500645.4117054 +
                               Cn1 * -762604822185.8782 +
                               Cn2 * -55758844708786.83 +
                               Cn3 * -352115062796160.94 +
                               Cn4 * -4.122433224426105e+16) +
                      pbar4 * (      35717860662.63424 +
                               Cn1 * -10490502652910.832 +
                               Cn2 * -2008178761532616.8 +
                               Cn3 * -1.104111813794906e+16 +
                               Cn4 * 2.902996969911872e+18)) +
               Cm5 * (        (      16472624.363474106 +
                               Cn1 * 1255848414.769289 +
                               Cn2 * -381541663667.6144 +
                               Cn3 * -31918398707741.184 +
                               Cn4 * -457343431991066.1) +
                      pbar1 * (      91563253.6306844 +
                               Cn1 * 170309660024.12076 +
                               Cn2 * 17997726401425.426 +
                               Cn3 * 92685709009801.53 +
                               Cn4 * -3.2398725385836136e+16) +
                      pbar2 * (      1109416713.1459327 +
                               Cn1 * 1211646014178.1118 +
                               Cn2 * 170792007611917.5 +
                               Cn3 * 1354059020723210.5 +
                               Cn4 * -8.291111180244363e+16) +
                      pbar3 * (      -910458619.2247283 +
                               Cn1 * -18527668404921.48 +
                               Cn2 * -2183671975890806.8 +
                               Cn3 * -1089597329822492.4 +
                               Cn4 * 4.204983768054948e+18) +
                      pbar4 * (      -78641209217.27798 +
                               Cn1 * -106579045626782.1 +
                               Cn2 * -1.4433081459144882e+16 +
                               Cn3 * 2.709276134213532e+16 +
                               Cn4 * 1.1954093584829876e+19))));
    
    R[2] = (
              (      (        (      -5.566307145097234 +
                               Cn1 * -577.2261321792826 +
                               Cn2 * -43890.68918803931 +
                               Cn3 * 620145.8174195035 +
                               Cn4 * 100814935.9250038) +
                      pbar1 * (      -28.27902191364425 +
                               Cn1 * 27851.160665467276 +
                               Cn2 * 2638115.3601078293 +
                               Cn3 * -35807965.15963846 +
                               Cn4 * -5729267358.950598) +
                      pbar2 * (      530.4211758880602 +
                               Cn1 * 97198.02030506435 +
                               Cn2 * 9074744.20001351 +
                               Cn3 * -198682510.92181483 +
                               Cn4 * -36538941678.20353) +
                      pbar3 * (      -1426.8210858103005 +
                               Cn1 * -2058881.3147204856 +
                               Cn2 * -241485880.68003824 +
                               Cn3 * 1457765776.4271755 +
                               Cn4 * 576335191436.6824) +
                      pbar4 * (      -55892.07670231039 +
                               Cn1 * -11204996.649317218 +
                               Cn2 * -782527442.0266052 +
                               Cn3 * 34867811773.08624 +
                               Cn4 * 3173870875201.0073)) +
               Cm1 * (        (      -20.694905957824858 +
                               Cn1 * 26047.067854833294 +
                               Cn2 * 1784658.2357785276 +
                               Cn3 * -59940200.254923694 +
                               Cn4 * -4315499834.037574) +
                      pbar1 * (      345.48233416318595 +
                               Cn1 * 433720.67496158986 +
                               Cn2 * 32327972.547815923 +
                               Cn3 * -1154658211.408464 +
                               Cn4 * -93561642682.72705) +
                      pbar2 * (      -13946.64443799249 +
                               Cn1 * -15243722.479597265 +
                               Cn2 * -1373509534.9873426 +
                               Cn3 * 13723552293.642736 +
                               Cn4 * 2330939062570.219) +
                      pbar3 * (      29237.714798911275 +
                               Cn1 * -46345830.203559294 +
                               Cn2 * -3941891568.963069 +
                               Cn3 * 144260730454.7866 +
                               Cn4 * 12207312984989.863) +
                      pbar4 * (      1469096.413397161 +
                               Cn1 * 1151621294.6249719 +
                               Cn2 * 96075240192.80756 +
                               Cn3 * -1264461895458.8748 +
                               Cn4 * -161842299508733.1)) +
               Cm2 * (        (      461.6499626576425 +
                               Cn1 * 126748.24777190597 +
                               Cn2 * 10366251.602971314 +
                               Cn3 * -79292308.700846 +
                               Cn4 * -26064760170.577854) +
                      pbar1 * (      -20798.71017699205 +
                               Cn1 * -17717938.16101076 +
                               Cn2 * -1806504912.2052557 +
                               Cn3 * 15059368997.998728 +
                               Cn4 * 3826741226372.1465) +
                      pbar2 * (      -708634.3930123949 +
                               Cn1 * -132277615.04037347 +
                               Cn2 * -5895500680.384345 +
                               Cn3 * 175998478073.11026 +
                               Cn4 * 13279026987176.967) +
                      pbar3 * (      643949.6975996653 +
                               Cn1 * 1091295708.672545 +
                               Cn2 * 160650651203.42877 +
                               Cn3 * 912406530760.3909 +
                               Cn4 * -345222333781211.5) +
                      pbar4 * (      64922696.209219344 +
                               Cn1 * 13087528269.20793 +
                               Cn2 * 542927809427.94574 +
                               Cn3 * -14312942710768.871 +
                               Cn4 * -437463134567606.8)) +
               Cm3 * (        (      -13705.165590684875 +
                               Cn1 * -6608256.292641688 +
                               Cn2 * -318472653.43650293 +
                               Cn3 * 24074338500.8721 +
                               Cn4 * 1129789464129.6301) +
                      pbar1 * (      244840.39692657776 +
                               Cn1 * -123323873.00272003 +
                               Cn2 * -19130152410.366486 +
                               Cn3 * 171586219130.6711 +
                               Cn4 * 48261506933063.26) +
                      pbar2 * (      11552164.784526668 +
                               Cn1 * 5760606113.465893 +
                               Cn2 * 440416492861.26605 +
                               Cn3 * -10141249157401.695 +
                               Cn4 * -957813435055937.8) +
                      pbar3 * (      -21506745.815156702 +
                               Cn1 * 18698269511.89279 +
                               Cn2 * 2511381697403.72 +
                               Cn3 * -25396209771025.824 +
                               Cn4 * -6405120680927482.0) +
                      pbar4 * (      -828356064.3693109 +
                               Cn1 * -438810477457.1477 +
                               Cn2 * -34881896157300.355 +
                               Cn3 * 885305699158933.8 +
                               Cn4 * 8.481928700862435e+16)) +
               Cm4 * (        (      -27693.72738248612 +
                               Cn1 * -19322033.82379176 +
                               Cn2 * -1962800141.5484815 +
                               Cn3 * 12970070174.139765 +
                               Cn4 * 4672853408909.083) +
                      pbar1 * (      2141007.643020295 +
                               Cn1 * 1507931088.355313 +
                               Cn2 * 156714175200.06824 +
                               Cn3 * -974997274258.8318 +
                               Cn4 * -327954956643462.44) +
                      pbar2 * (      64011520.09572267 +
                               Cn1 * 14188988244.487259 +
                               Cn2 * 814428221193.8204 +
                               Cn3 * -6968923972004.578 +
                               Cn4 * -1216595480730283.5) +
                      pbar3 * (      -76063137.15682189 +
                               Cn1 * -94451856049.72572 +
                               Cn2 * -14752307239902.898 +
                               Cn3 * -154121027483094.0 +
                               Cn4 * 3.061930271868067e+16) +
                      pbar4 * (      -5877979661.830637 +
                               Cn1 * -1306885132465.2664 +
                               Cn2 * -64688200830115.48 +
                               Cn3 * 107524034897503.86 +
                               Cn4 * -1.269488262611221e+16)) +
               Cm5 * (        (      1098605.9328305156 +
                               Cn1 * 364141034.662914 +
                               Cn2 * 8931266018.989473 +
                               Cn3 * -1776095994868.0806 +
                               Cn4 * -55355834076285.65) +
                      pbar1 * (      -25514145.841079753 +
                               Cn1 * 9731754278.076546 +
                               Cn2 * 1760064386341.8818 +
                               Cn3 * -4247555292843.823 +
                               Cn4 * -4126855182579322.0) +
                      pbar2 * (      -1034505137.5455331 +
                               Cn1 * -421357417023.6442 +
                               Cn2 * -29696818327818.2 +
                               Cn3 * 932118179017721.1 +
                               Cn4 * 7.40323833890777e+16) +
                      pbar3 * (      1682922039.3626873 +
                               Cn1 * -1546513170609.9214 +
                               Cn2 * -236609693140974.1 +
                               Cn3 * 420877738610338.25 +
                               Cn4 * 5.617796121983105e+17) +
                      pbar4 * (      68975647545.48807 +
                               Cn1 * 32931377772689.695 +
                               Cn2 * 2606201577706499.0 +
                               Cn3 * -8.241709457711027e+16 +
                               Cn4 * -7.606713642793983e+18))) +
        CL1 * (      (        (      -2.2648227040018756 +
                               Cn1 * 2807.6099888367867 +
                               Cn2 * 310560.016573778 +
                               Cn3 * -426612.16993544146 +
                               Cn4 * -632206077.3732408) +
                      pbar1 * (      -62.555406689627084 +
                               Cn1 * -128269.46559862443 +
                               Cn2 * -15188006.966827333 +
                               Cn3 * 245829478.99094388 +
                               Cn4 * 37119330481.12239) +
                      pbar2 * (      782.8873662685496 +
                               Cn1 * -1744790.229072893 +
                               Cn2 * -233621639.86814752 +
                               Cn3 * 496570577.5085171 +
                               Cn4 * 586936948815.8915) +
                      pbar3 * (      9100.050028756972 +
                               Cn1 * 7384438.973155176 +
                               Cn2 * 1305947589.2184305 +
                               Cn3 * -8015172333.903224 +
                               Cn4 * -3854982588535.998) +
                      pbar4 * (      162274.44066138694 +
                               Cn1 * 173735712.64622697 +
                               Cn2 * 17614933425.708397 +
                               Cn3 * -309437435965.2262 +
                               Cn4 * -50027813459058.07)) +
               Cm1 * (        (      -181.4309126491767 +
                               Cn1 * -98887.98729138085 +
                               Cn2 * -2606212.69564782 +
                               Cn3 * 531607905.37556034 +
                               Cn4 * 22505456544.063427) +
                      pbar1 * (      1074.338905104528 +
                               Cn1 * -8215580.073864306 +
                               Cn2 * -913955320.182938 +
                               Cn3 * 15524059112.55939 +
                               Cn4 * 2194560106908.8767) +
                      pbar2 * (      -61337.55027951926 +
                               Cn1 * 28128547.83128207 +
                               Cn2 * 4771567075.174467 +
                               Cn3 * 47154837050.28296 +
                               Cn4 * -6946503881510.615) +
                      pbar3 * (      -910787.8427615473 +
                               Cn1 * 867107991.086087 +
                               Cn2 * 100531497995.88892 +
                               Cn3 * -2037436747184.5242 +
                               Cn4 * -261013802262441.25) +
                      pbar4 * (      1746462.7472677825 +
                               Cn1 * -788400592.8580858 +
                               Cn2 * -203191627280.81906 +
                               Cn3 * -7328980376822.695 +
                               Cn4 * 71659899724237.06)) +
               Cm2 * (        (      -1502.1669865780461 +
                               Cn1 * -2591261.5373306302 +
                               Cn2 * -239201785.0360107 +
                               Cn3 * 3990408336.3194594 +
                               Cn4 * 635222717111.58) +
                      pbar1 * (      156623.46122995467 +
                               Cn1 * 99912224.15889162 +
                               Cn2 * 10559006390.838074 +
                               Cn3 * -110537667816.29147 +
                               Cn4 * -23993044560232.484) +
                      pbar2 * (      5393357.379299846 +
                               Cn1 * 2321431679.35465 +
                               Cn2 * 179993023911.35614 +
                               Cn3 * -2604361599977.263 +
                               Cn4 * -411423713520108.5) +
                      pbar3 * (      4435951.354705435 +
                               Cn1 * -2186256282.274734 +
                               Cn2 * -864771849136.3765 +
                               Cn3 * -15991382321744.258 +
                               Cn4 * 2017163074796282.2) +
                      pbar4 * (      -481727296.2890205 +
                               Cn1 * -196841093326.83392 +
                               Cn2 * -13888307783368.885 +
                               Cn3 * 235470466628294.66 +
                               Cn4 * 2.6620922733012588e+16)) +
               Cm3 * (        (      139856.4512024207 +
                               Cn1 * 31474705.939372994 +
                               Cn2 * -488163341.71128917 +
                               Cn3 * -207209041487.46432 +
                               Cn4 * -4839635573213.876) +
                      pbar1 * (      -3265309.971554264 +
                               Cn1 * 2280342233.5036087 +
                               Cn2 * 331574837024.51587 +
                               Cn3 * -3197776718162.0537 +
                               Cn4 * -774242054901112.6) +
                      pbar2 * (      -64915550.32869848 +
                               Cn1 * -19352482212.558414 +
                               Cn2 * -1377529622779.7578 +
                               Cn3 * 31672618172347.477 +
                               Cn4 * 2867593937946174.5) +
                      pbar3 * (      297938304.98327786 +
                               Cn1 * -324889745759.546 +
                               Cn2 * -42403229766742.35 +
                               Cn3 * 533070912323435.3 +
                               Cn4 * 1.0505377195907091e+17) +
                      pbar4 * (      3707096217.4729414 +
                               Cn1 * 682067869370.4594 +
                               Cn2 * 66805841245597.72 +
                               Cn3 * -865982600475380.2 +
                               Cn4 * -1.6658434359748403e+17)) +
               Cm4 * (        (      211049.4625340901 +
                               Cn1 * 316398607.8489303 +
                               Cn2 * 30059084437.031246 +
                               Cn3 * -444162145458.14197 +
                               Cn4 * -76886790647570.95) +
                      pbar1 * (      -17515495.467399325 +
                               Cn1 * -9414176511.615147 +
                               Cn2 * -1007372894028.9064 +
                               Cn3 * 7251911829292.975 +
                               Cn4 * 2201927019092459.8) +
                      pbar2 * (      -557066377.1094979 +
                               Cn1 * -228503236373.55896 +
                               Cn2 * -17633882321135.934 +
                               Cn3 * 187110086806088.72 +
                               Cn4 * 3.5922042300991892e+16) +
                      pbar3 * (      -310444970.139479 +
                               Cn1 * 204349823944.99796 +
                               Cn2 * 85485470261038.27 +
                               Cn3 * 2139926200904271.2 +
                               Cn4 * -1.8361968608874134e+17) +
                      pbar4 * (      47035468398.916115 +
                               Cn1 * 18750123102235.918 +
                               Cn2 * 1331633513420234.0 +
                               Cn3 * -1.2916463815950776e+16 +
                               Cn4 * -1.875099036213694e+18)) +
               Cm5 * (        (      -11224196.135281483 +
                               Cn1 * -1350584331.348243 +
                               Cn2 * 150698812003.50897 +
                               Cn3 * 14796564235101.174 +
                               Cn4 * 99826159150306.02) +
                      pbar1 * (      288297594.62268776 +
                               Cn1 * -160423768435.8153 +
                               Cn2 * -25457104420302.38 +
                               Cn3 * 149217549788060.9 +
                               Cn4 * 5.769434981140656e+16) +
                      pbar2 * (      7357101533.377279 +
                               Cn1 * 1483511467574.7888 +
                               Cn2 * 73661194385157.12 +
                               Cn3 * -3975402965046768.5 +
                               Cn4 * -2.1492290382885392e+17) +
                      pbar3 * (      -18376397914.35305 +
                               Cn1 * 24828706877700.086 +
                               Cn2 * 3434106528913246.5 +
                               Cn3 * -2.572171989370105e+16 +
                               Cn4 * -8.263150977879567e+18) +
                      pbar4 * (      -401823343103.9463 +
                               Cn1 * -50492385688622.375 +
                               Cn2 * -4206875143350666.5 +
                               Cn3 * 1.9356119723687658e+17 +
                               Cn4 * 2.0715247187153318e+19))) +
        CL2 * (      (        (      9.662820260590557 +
                               Cn1 * 368.53465366079365 +
                               Cn2 * -105834.18182798565 +
                               Cn3 * -6461181.8548410125 +
                               Cn4 * 268799817.2381324) +
                      pbar1 * (      196.42354866981495 +
                               Cn1 * 149146.32117907755 +
                               Cn2 * 18131993.975953333 +
                               Cn3 * -275842046.11927533 +
                               Cn4 * -40689947154.83247) +
                      pbar2 * (      -7827.272796544742 +
                               Cn1 * 933252.4170604852 +
                               Cn2 * 402421608.2853876 +
                               Cn3 * 10777872726.74732 +
                               Cn4 * -800711476636.2356) +
                      pbar3 * (      -65303.22753015884 +
                               Cn1 * -7985807.04510054 +
                               Cn2 * -1579328675.9949665 +
                               Cn3 * -3907258525.098585 +
                               Cn4 * 5127889408950.873) +
                      pbar4 * (      -144187.62056642538 +
                               Cn1 * -169869358.01757407 +
                               Cn2 * -28415752321.624657 +
                               Cn3 * -275006071535.2079 +
                               Cn4 * 64668373992696.27)) +
               Cm1 * (        (      614.051945717205 +
                               Cn1 * 162441.9954180221 +
                               Cn2 * -1384768.052045093 +
                               Cn3 * -1122327092.0886736 +
                               Cn4 * -38523698360.05596) +
                      pbar1 * (      2016.5720246365254 +
                               Cn1 * 21589005.240193725 +
                               Cn2 * 2365070064.640424 +
                               Cn3 * -46203073838.50417 +
                               Cn4 * -5925655427538.678) +
                      pbar2 * (      224654.99363134895 +
                               Cn1 * 67195080.99676548 +
                               Cn2 * 29263159.796156384 +
                               Cn3 * -389739227989.8559 +
                               Cn4 * -11213626030879.643) +
                      pbar3 * (      2654484.8256151285 +
                               Cn1 * -2162173032.8204226 +
                               Cn2 * -258104543045.31924 +
                               Cn3 * 5721836024910.872 +
                               Cn4 * 713784892059118.4) +
                      pbar4 * (      -5852816.604271519 +
                               Cn1 * -10233860665.109222 +
                               Cn2 * -523672954764.755 +
                               Cn3 * 53054151605183.41 +
                               Cn4 * 2627940656007582.0)) +
               Cm2 * (        (      2777.9324962811042 +
                               Cn1 * 5710978.479628247 +
                               Cn2 * 520265600.48208106 +
                               Cn3 * -10326072667.853909 +
                               Cn4 * -1500681770830.148) +
                      pbar1 * (      -336895.5534947288 +
                               Cn1 * -139692274.0938476 +
                               Cn2 * -14720810051.520868 +
                               Cn3 * 157260209794.77567 +
                               Cn4 * 34505944416231.773) +
                      pbar2 * (      -10113256.779809004 +
                               Cn1 * -4808904079.237143 +
                               Cn2 * -413747327586.7952 +
                               Cn3 * 3223656588909.936 +
                               Cn4 * 900380867415711.6) +
                      pbar3 * (      -11632287.72621059 +
                               Cn1 * -4322556603.2990465 +
                               Cn2 * 980444175378.162 +
                               Cn3 * 47024012681404.38 +
                               Cn4 * -2275555576970609.5) +
                      pbar4 * (      911942729.6810218 +
                               Cn1 * 382232581982.9652 +
                               Cn2 * 30050613219133.934 +
                               Cn3 * -248719552725060.16 +
                               Cn4 * -5.466732384948545e+16)) +
               Cm3 * (        (      -315996.61788848694 +
                               Cn1 * -30299617.0339388 +
                               Cn2 * 4915959177.88797 +
                               Cn3 * 413924814665.3532 +
                               Cn4 * 3860326013163.712) +
                      pbar1 * (      6683562.901026312 +
                               Cn1 * -6458346684.644905 +
                               Cn2 * -876902394891.6008 +
                               Cn3 * 11398314734986.344 +
                               Cn4 * 2122258999296317.2) +
                      pbar2 * (      100847187.02018727 +
                               Cn1 * -7473095359.188059 +
                               Cn2 * -852305695479.996 +
                               Cn3 * 45563317937596.68 +
                               Cn4 * 3577835710424278.5) +
                      pbar3 * (      -786799471.2660291 +
                               Cn1 * 872820327080.3824 +
                               Cn2 * 109826487287652.36 +
                               Cn3 * -1839044132656950.5 +
                               Cn4 * -2.887877605548185e+17) +
                      pbar4 * (      -5536645505.381288 +
                               Cn1 * 3377608087835.4556 +
                               Cn2 * 225066562208482.38 +
                               Cn3 * -1.2745297408319842e+16 +
                               Cn4 * -7.34864811223784e+17)) +
               Cm4 * (        (      -601768.2905104283 +
                               Cn1 * -782642861.9517727 +
                               Cn2 * -71142921416.18358 +
                               Cn3 * 1221405587815.949 +
                               Cn4 * 187707067114035.53) +
                      pbar1 * (      34398539.224020675 +
                               Cn1 * 13552651693.344893 +
                               Cn2 * 1489817130266.1934 +
                               Cn3 * -10011517091919.309 +
                               Cn4 * -3350765199604689.0) +
                      pbar2 * (      1093207403.723124 +
                               Cn1 * 503020901302.90125 +
                               Cn2 * 42166540718993.53 +
                               Cn3 * -255357821394837.6 +
                               Cn4 * -8.360600543457131e+16) +
                      pbar3 * (      1503148964.8095438 +
                               Cn1 * 442056233236.7986 +
                               Cn2 * -103296974141361.67 +
                               Cn3 * -5679543809489151.0 +
                               Cn4 * 2.1244612222975862e+17) +
                      pbar4 * (      -88865155549.27277 +
                               Cn1 * -38460110223989.16 +
                               Cn2 * -3031852583172104.0 +
                               Cn3 * 1.0417246159294744e+16 +
                               Cn4 * 4.272292171426553e+18)) +
               Cm5 * (        (      23589455.699011654 +
                               Cn1 * -347079687.285984 +
                               Cn2 * -646688212147.4208 +
                               Cn3 * -29260249133108.05 +
                               Cn4 * 336141416369417.1) +
                      pbar1 * (      -601003648.7550437 +
                               Cn1 * 457885468512.5306 +
                               Cn2 * 66402118591005.08 +
                               Cn3 * -641407399816639.6 +
                               Cn4 * -1.565382461301788e+17) +
                      pbar2 * (      -12929006915.347155 +
                               Cn1 * 569275720831.1586 +
                               Cn2 * 135388479852198.84 +
                               Cn3 * -287753568522677.1 +
                               Cn4 * -2.9560387296504755e+17) +
                      pbar3 * (      45897586294.67839 +
                               Cn1 * -67246826826109.84 +
                               Cn2 * -8772346316090579.0 +
                               Cn3 * 1.0926523134960891e+17 +
                               Cn4 * 2.254366636996492e+19) +
                      pbar4 * (      648037500080.3654 +
                               Cn1 * -273042681571126.06 +
                               Cn2 * -1.9783453056706404e+16 +
                               Cn3 * 7.404299962404436e+17 +
                               Cn4 * 4.135033839534536e+19))) +
        CL3 * (      (        (      -7.630404081082711 +
                               Cn1 * -1584.7286023045212 +
                               Cn2 * -43453.241759470984 +
                               Cn3 * 5953018.356382994 +
                               Cn4 * 34806917.487852) +
                      pbar1 * (      -45.433060351923395 +
                               Cn1 * -22219.960383377922 +
                               Cn2 * -4797352.90848112 +
                               Cn3 * -25129551.631808307 +
                               Cn4 * 6062510492.448015) +
                      pbar2 * (      8492.579006120686 +
                               Cn1 * 858255.41899804 +
                               Cn2 * -206876718.45524463 +
                               Cn3 * -12254465016.547012 +
                               Cn4 * 286342214820.4472) +
                      pbar3 * (      63431.033644399184 +
                               Cn1 * 1546397.7714899855 +
                               Cn2 * 440759262.8960808 +
                               Cn3 * 15791807884.458292 +
                               Cn4 * -1658487007591.604) +
                      pbar4 * (      165.86376628138373 +
                               Cn1 * 343656.15829601284 +
                               Cn2 * 12759747538.622684 +
                               Cn3 * 623334967725.4618 +
                               Cn4 * -18495789273353.28)) +
               Cm1 * (        (      -487.94842412069903 +
                               Cn1 * -87802.37499850625 +
                               Cn2 * 2785395.6816310924 +
                               Cn3 * 660495029.8405685 +
                               Cn4 * 19943543656.9078) +
                      pbar1 * (      -3744.985881998173 +
                               Cn1 * -14694685.0511751 +
                               Cn2 * -1565980765.8986247 +
                               Cn3 * 34951974561.86092 +
                               Cn4 * 4080341617410.0483) +
                      pbar2 * (      -154579.79994430736 +
                               Cn1 * -95365357.52961238 +
                               Cn2 * -4641503633.964738 +
                               Cn3 * 374756562880.555 +
                               Cn4 * 19675481395203.637) +
                      pbar3 * (      -2004061.3813613395 +
                               Cn1 * 1405140447.116191 +
                               Cn2 * 169741198510.38968 +
                               Cn3 * -4112318597597.952 +
                               Cn4 * -491561196386941.5) +
                      pbar4 * (      912587.7012794998 +
                               Cn1 * 11258360213.472591 +
                               Cn2 * 768562224540.7522 +
                               Cn3 * -49284043233571.766 +
                               Cn4 * -2983667668820442.5)) +
               Cm2 * (        (      -2208.291069714223 +
                               Cn1 * -3595556.4323307388 +
                               Cn2 * -323786632.035636 +
                               Cn3 * 6907736881.644665 +
                               Cn4 * 972001109328.7195) +
                      pbar1 * (      201828.13919586243 +
                               Cn1 * 53254064.45469473 +
                               Cn2 * 5549547743.822888 +
                               Cn3 * -54471779522.25935 +
                               Cn4 * -13367323740574.557) +
                      pbar2 * (      5505693.166431472 +
                               Cn1 * 2686177870.7533545 +
                               Cn2 * 250811812981.123 +
                               Cn3 * -596762898076.0219 +
                               Cn4 * -521860168146888.56) +
                      pbar3 * (      5535025.52780193 +
                               Cn1 * 6105361380.098861 +
                               Cn2 * -215659801151.7348 +
                               Cn3 * -33719015542259.45 +
                               Cn4 * 447493866965915.0) +
                      pbar4 * (      -515206741.373484 +
                               Cn1 * -201461440898.5921 +
                               Cn2 * -17249440880949.908 +
                               Cn3 * 834749529666.7743 +
                               Cn4 * 2.89795905469575e+16)) +
               Cm3 * (        (      196342.68745699874 +
                               Cn1 * 1237155.3541034844 +
                               Cn2 * -4616487264.734528 +
                               Cn3 * -232168549881.8182 +
                               Cn4 * 590435254665.2589) +
                      pbar1 * (      -4023011.330257099 +
                               Cn1 * 4492107560.139129 +
                               Cn2 * 587734360668.1921 +
                               Cn3 * -9318547668046.135 +
                               Cn4 * -1478519736845378.2) +
                      pbar2 * (      -57390935.27437679 +
                               Cn1 * 25002182679.98459 +
                               Cn2 * 2232925539003.437 +
                               Cn3 * -79103713318647.86 +
                               Cn4 * -6817027709986157.0) +
                      pbar3 * (      551944623.3510122 +
                               Cn1 * -592791418573.8812 +
                               Cn2 * -72600045351637.98 +
                               Cn3 * 1480601458120338.5 +
                               Cn4 * 2.014405984216578e+17) +
                      pbar4 * (      3624585211.1771855 +
                               Cn1 * -4090974539905.9766 +
                               Cn2 * -305677532030702.9 +
                               Cn3 * 1.4653757226166492e+16 +
                               Cn4 * 9.874572850169857e+17)) +
               Cm4 * (        (      456564.8439405221 +
                               Cn1 * 511440312.5582864 +
                               Cn2 * 45216747094.92694 +
                               Cn3 * -839675353190.1509 +
                               Cn4 * -121541250746736.44) +
                      pbar1 * (      -20646219.715579636 +
                               Cn1 * -5509965018.15405 +
                               Cn2 * -604577470108.2784 +
                               Cn3 * 3778173373412.254 +
                               Cn4 * 1407918689284370.8) +
                      pbar2 * (      -628359279.9280107 +
                               Cn1 * -296621362614.95154 +
                               Cn2 * -26186648599000.473 +
                               Cn3 * 62726515374238.35 +
                               Cn4 * 5.031227385970206e+16) +
                      pbar3 * (      -1040760387.1067288 +
                               Cn1 * -611778786525.5472 +
                               Cn2 * 27269625331029.316 +
                               Cn3 * 3855907637406862.5 +
                               Cn4 * -4.531722786409656e+16) +
                      pbar4 * (      50025721951.37354 +
                               Cn1 * 21361259577878.086 +
                               Cn2 * 1808791917935604.0 +
                               Cn3 * 4826280697854262.0 +
                               Cn4 * -2.399223521481286e+18)) +
               Cm5 * (        (      -13494607.384519897 +
                               Cn1 * 1806243187.9088566 +
                               Cn2 * 536142746817.2162 +
                               Cn3 * 16227032383452.717 +
                               Cn4 * -458737368420702.9) +
                      pbar1 * (      382087497.97585225 +
                               Cn1 * -317582424749.59045 +
                               Cn2 * -44272581422964.66 +
                               Cn3 * 556310872111261.6 +
                               Cn4 * 1.0860168627907584e+17) +
                      pbar2 * (      7634406688.418061 +
                               Cn1 * -1897430521739.2285 +
                               Cn2 * -213336459210336.9 +
                               Cn3 * 4139676056966383.0 +
                               Cn4 * 5.373923019005469e+17) +
                      pbar3 * (      -31304493924.267868 +
                               Cn1 * 45932029223338.74 +
                               Cn2 * 5765854380338853.0 +
                               Cn3 * -9.527445098585878e+16 +
                               Cn4 * -1.571273231931201e+19) +
                      pbar4 * (      -392950316386.7094 +
                               Cn1 * 325725019043744.7 +
                               Cn2 * 2.4977791612833964e+16 +
                               Cn3 * -1.0041960431736127e+18 +
                               Cn4 * -6.738535634295446e+19))));
    
    R[3] = (
              (      (        (      -1.7118235433859434 +
                               Cn1 * 1200.9961534141385 +
                               Cn2 * 73877.4656033299 +
                               Cn3 * -1949422.3462686324 +
                               Cn4 * -145354711.72783712) +
                      pbar1 * (      -174.9939286562722 +
                               Cn1 * -34423.20827163021 +
                               Cn2 * -1775028.3441327678 +
                               Cn3 * 62401795.27847281 +
                               Cn4 * 3918623849.101659) +
                      pbar2 * (      -788.5866330621153 +
                               Cn1 * -427417.38705758675 +
                               Cn2 * -25500562.421890453 +
                               Cn3 * 908636234.3856788 +
                               Cn4 * 55061599048.63035) +
                      pbar3 * (      7257.658262469447 +
                               Cn1 * 2168649.7483992167 +
                               Cn2 * 133806683.92819516 +
                               Cn3 * -3476297696.051204 +
                               Cn4 * -298491253175.3202) +
                      pbar4 * (      58027.905952021734 +
                               Cn1 * 35957362.13924429 +
                               Cn2 * 2077848612.1816564 +
                               Cn3 * -86683878233.50894 +
                               Cn4 * -4519599261033.891)) +
               Cm1 * (        (      -66.56419920356926 +
                               Cn1 * -6061.724314931242 +
                               Cn2 * -558526.9424679426 +
                               Cn3 * 13527457.437816234 +
                               Cn4 * 1527748570.5701249) +
                      pbar1 * (      274.111653012028 +
                               Cn1 * -406953.9286825884 +
                               Cn2 * -52806998.68895513 +
                               Cn3 * 417742261.96797514 +
                               Cn4 * 122653712156.48737) +
                      pbar2 * (      36037.94170866179 +
                               Cn1 * 12143889.920345493 +
                               Cn2 * 934551275.5888661 +
                               Cn3 * -14677105077.507458 +
                               Cn4 * -2123122891180.2334) +
                      pbar3 * (      56640.228053812534 +
                               Cn1 * 47071647.06990591 +
                               Cn2 * 5028375387.898791 +
                               Cn3 * -50842020773.90324 +
                               Cn4 * -13483721991797.41) +
                      pbar4 * (      -2709227.3261733125 +
                               Cn1 * -811807706.1225986 +
                               Cn2 * -71768294468.83571 +
                               Cn3 * 288341317304.5599 +
                               Cn4 * 170756355406190.5)) +
               Cm2 * (        (      -837.7730089158157 +
                               Cn1 * -365404.6620417974 +
                               Cn2 * -21801697.692779753 +
                               Cn3 * 603987291.1094627 +
                               Cn4 * 40609519907.51931) +
                      pbar1 * (      69097.99020321737 +
                               Cn1 * 15786149.062885365 +
                               Cn2 * 467067074.26895744 +
                               Cn3 * -31171852594.47064 +
                               Cn4 * -874902096668.9548) +
                      pbar2 * (      748909.7400933177 +
                               Cn1 * 269224043.71338516 +
                               Cn2 * 14492491929.482016 +
                               Cn3 * -475489543749.46094 +
                               Cn4 * -30343497833461.934) +
                      pbar3 * (      -3510562.25175759 +
                               Cn1 * -961396935.6479 +
                               Cn2 * -30856795936.383335 +
                               Cn3 * 1663156044485.9683 +
                               Cn4 * 32350376974636.695) +
                      pbar4 * (      -58655438.25486628 +
                               Cn1 * -23427024877.18227 +
                               Cn2 * -1296903429449.564 +
                               Cn3 * 42632128160017.91 +
                               Cn4 * 2721810400085819.0)) +
               Cm3 * (        (      -11272.519647142233 +
                               Cn1 * -2410773.3382882816 +
                               Cn2 * -68663317.10972698 +
                               Cn3 * 1739528957.5762901 +
                               Cn4 * -66617613584.92512) +
                      pbar1 * (      -545811.2820243193 +
                               Cn1 * 75592201.92996307 +
                               Cn2 * 14199776946.017687 +
                               Cn3 * -75520392899.74316 +
                               Cn4 * -30703486481516.258) +
                      pbar2 * (      -19768798.56492213 +
                               Cn1 * -5422363797.351084 +
                               Cn2 * -315075541896.4844 +
                               Cn3 * 7476544230768.819 +
                               Cn4 * 736244287830337.8) +
                      pbar3 * (      660102.1291963259 +
                               Cn1 * -13653293419.419287 +
                               Cn2 * -1328463400393.8027 +
                               Cn3 * 19760904275919.355 +
                               Cn4 * 3207834955482590.5) +
                      pbar4 * (      1558205422.8703637 +
                               Cn1 * 415117309830.1833 +
                               Cn2 * 25812713839156.652 +
                               Cn3 * -443208090778704.7 +
                               Cn4 * -6.1695156721520024e+16)) +
               Cm4 * (        (      55355.249643737996 +
                               Cn1 * 26282049.00747226 +
                               Cn2 * 1729540226.4258494 +
                               Cn3 * -41763694942.34968 +
                               Cn4 * -3124009393280.964) +
                      pbar1 * (      -4959177.065921224 +
                               Cn1 * -1205865857.0340238 +
                               Cn2 * -31865256825.047787 +
                               Cn3 * 2520552119072.3804 +
                               Cn4 * 55224153523300.34) +
                      pbar2 * (      -65050848.283040576 +
                               Cn1 * -23154670643.013294 +
                               Cn2 * -1261266765922.3765 +
                               Cn3 * 40645471922764.82 +
                               Cn4 * 2670112027445697.5) +
                      pbar3 * (      246052542.18298385 +
                               Cn1 * 79050325050.77747 +
                               Cn2 * 2037698199092.9214 +
                               Cn3 * -163701428357740.22 +
                               Cn4 * -989862505887191.5) +
                      pbar4 * (      5089272588.829608 +
                               Cn1 * 2052329127072.4927 +
                               Cn2 * 116585595251570.64 +
                               Cn3 * -3705431460196303.5 +
                               Cn4 * -2.5708336446419382e+17)) +
               Cm5 * (        (      823319.8182711382 +
                               Cn1 * 204061308.6902465 +
                               Cn2 * 10024000172.744371 +
                               Cn3 * -103838526624.13472 +
                               Cn4 * -1814852208744.5984) +
                      pbar1 * (      49732908.54100024 +
                               Cn1 * -3585420026.672725 +
                               Cn2 * -887015316073.7313 +
                               Cn3 * 3200434917743.108 +
                               Cn4 * 1802440131379472.0) +
                      pbar2 * (      1634045502.1745048 +
                               Cn1 * 432671418924.8462 +
                               Cn2 * 22147235560288.785 +
                               Cn3 * -650586547478504.0 +
                               Cn4 * -5.258549292770512e+16) +
                      pbar3 * (      -289802672.82665366 +
                               Cn1 * 902332181936.0211 +
                               Cn2 * 76447868059586.08 +
                               Cn3 * -1363849573016591.5 +
                               Cn4 * -1.570600933528997e+17) +
                      pbar4 * (      -129243631739.15388 +
                               Cn1 * -34845481263570.973 +
                               Cn2 * -1877605412420480.8 +
                               Cn3 * 4.880188540675154e+16 +
                               Cn4 * 4.543653544854034e+18))) +
        CL1 * (      (        (      -21.875235632189767 +
                               Cn1 * -5176.840421155735 +
                               Cn2 * -303736.71167017455 +
                               Cn3 * 10219198.25030615 +
                               Cn4 * 662422067.0823127) +
                      pbar1 * (      431.50689689201386 +
                               Cn1 * 94768.07098971924 +
                               Cn2 * 1323364.750386973 +
                               Cn3 * -244602965.6209289 +
                               Cn4 * -6710208668.240278) +
                      pbar2 * (      1355.8215653287862 +
                               Cn1 * 2084932.8248967577 +
                               Cn2 * 204598987.8070979 +
                               Cn3 * -5886786305.403736 +
                               Cn4 * -506492367829.98376) +
                      pbar3 * (      -3680.3216903563693 +
                               Cn1 * -3100953.704410218 +
                               Cn2 * -355122012.04589653 +
                               Cn3 * -606659709.0010351 +
                               Cn4 * 1254912626787.0593) +
                      pbar4 * (      -54325.95108893441 +
                               Cn1 * -175883088.8813239 +
                               Cn2 * -14932818896.938007 +
                               Cn3 * 651153297667.6039 +
                               Cn4 * 37155954942929.05)) +
               Cm1 * (        (      -800.9113707807759 +
                               Cn1 * -200154.5678378972 +
                               Cn2 * -7801331.106887286 +
                               Cn3 * 247714226.0447046 +
                               Cn4 * 14089565313.315842) +
                      pbar1 * (      68.97330006506758 +
                               Cn1 * 5458267.273074148 +
                               Cn2 * 495475387.91803974 +
                               Cn3 * -8040975062.838587 +
                               Cn4 * -1163405174868.8381) +
                      pbar2 * (      -133320.86993339637 +
                               Cn1 * -48455619.60431552 +
                               Cn2 * -4178585654.654749 +
                               Cn3 * 57398622660.287285 +
                               Cn4 * 9068092306664.908) +
                      pbar3 * (      -1033959.9874928017 +
                               Cn1 * -573128559.0112113 +
                               Cn2 * -42479523348.69946 +
                               Cn3 * 851722861584.6248 +
                               Cn4 * 130719066788637.17) +
                      pbar4 * (      13089517.307544356 +
                               Cn1 * 2965498273.1558557 +
                               Cn2 * 331941746131.841 +
                               Cn3 * 5019568336025.591 +
                               Cn4 * -764769376165998.2)) +
               Cm2 * (        (      5620.428887730513 +
                               Cn1 * 2050844.3617516295 +
                               Cn2 * 139477929.98773703 +
                               Cn3 * -4301054701.305792 +
                               Cn4 * -297616603412.47925) +
                      pbar1 * (      -398016.25108396495 +
                               Cn1 * -67277521.83125968 +
                               Cn2 * 774433334.3282841 +
                               Cn3 * 187273490678.06818 +
                               Cn4 * -321293660922.86084) +
                      pbar2 * (      -6586676.640789646 +
                               Cn1 * -2172229766.329736 +
                               Cn2 * -132525661733.05606 +
                               Cn3 * 4688913597583.438 +
                               Cn4 * 318046166620467.8) +
                      pbar3 * (      4869984.172586025 +
                               Cn1 * 1414574345.6544814 +
                               Cn2 * -82102614823.45193 +
                               Cn3 * -3264622896599.1665 +
                               Cn4 * 182299659108442.78) +
                      pbar4 * (      455640294.27872944 +
                               Cn1 * 176324178967.05737 +
                               Cn2 * 10753473421436.48 +
                               Cn3 * -429177457803720.9 +
                               Cn4 * -2.6137050052168836e+16)) +
               Cm3 * (        (      194904.0757059851 +
                               Cn1 * 54089971.30707144 +
                               Cn2 * 2779044601.810722 +
                               Cn3 * -48270582345.02008 +
                               Cn4 * -3874525367610.274) +
                      pbar1 * (      5137877.808023265 +
                               Cn1 * -1151458160.2661128 +
                               Cn2 * -151506155014.24387 +
                               Cn3 * 1872609247251.8352 +
                               Cn4 * 360340774482247.0) +
                      pbar2 * (      156163920.6272135 +
                               Cn1 * 34478824247.944275 +
                               Cn2 * 1575977145617.2651 +
                               Cn3 * -47452988902848.91 +
                               Cn4 * -3694847512214615.0) +
                      pbar3 * (      202215720.0629145 +
                               Cn1 * 197518103576.0203 +
                               Cn2 * 13735594297698.42 +
                               Cn3 * -376241600507697.0 +
                               Cn4 * -3.980841442203439e+16) +
                      pbar4 * (      -12346715807.026222 +
                               Cn1 * -2589705373066.1196 +
                               Cn2 * -128392055430218.75 +
                               Cn3 * 1550106949033849.0 +
                               Cn4 * 2.9582861340915117e+17)) +
               Cm4 * (        (      -359525.0602141796 +
                               Cn1 * -157948672.29223663 +
                               Cn2 * -11446087544.913021 +
                               Cn3 * 353852074179.20215 +
                               Cn4 * 23886801546612.65) +
                      pbar1 * (      30980636.73239766 +
                               Cn1 * 5842802875.739527 +
                               Cn2 * -86584363822.35684 +
                               Cn3 * -18078734292666.906 +
                               Cn4 * 65346709907044.016) +
                      pbar2 * (      621913097.4342152 +
                               Cn1 * 201884745155.64185 +
                               Cn2 * 11818043226152.006 +
                               Cn3 * -439152884438524.75 +
                               Cn4 * -2.9102266373013068e+16) +
                      pbar3 * (      13610549.166024337 +
                               Cn1 * -163869957190.4163 +
                               Cn2 * 8372378172937.672 +
                               Cn3 * 807567435903255.9 +
                               Cn4 * -1.373010606718184e+16) +
                      pbar4 * (      -42525731574.27761 +
                               Cn1 * -16735286823989.178 +
                               Cn2 * -1000578100116145.4 +
                               Cn3 * 4.1482823338170664e+16 +
                               Cn4 * 2.628757619370566e+18)) +
               Cm5 * (        (      -11156583.61096003 +
                               Cn1 * -3350895029.7012386 +
                               Cn2 * -208278204175.021 +
                               Cn3 * 2054556308188.1162 +
                               Cn4 * 260749092068287.12) +
                      pbar1 * (      -491046770.5043932 +
                               Cn1 * 63090537819.4205 +
                               Cn2 * 10147229232875.725 +
                               Cn3 * -111150422681066.1 +
                               Cn4 * -2.3741005343290308e+16) +
                      pbar2 * (      -14328645011.338457 +
                               Cn1 * -3094970277214.4688 +
                               Cn2 * -110904666819992.7 +
                               Cn3 * 4674373280573455.0 +
                               Cn4 * 2.6277229158615757e+17) +
                      pbar3 * (      -14041257707.926851 +
                               Cn1 * -14125525824964.951 +
                               Cn2 * -892502564908682.6 +
                               Cn3 * 2.7566696948341456e+16 +
                               Cn4 * 2.3868730471882906e+18) +
                      pbar4 * (      1100252513140.6702 +
                               Cn1 * 244902481343850.3 +
                               Cn2 * 9250542471116950.0 +
                               Cn3 * -2.810199352693556e+17 +
                               Cn4 * -2.159643145920035e+19))) +
        CL2 * (      (        (      5.778259400598068 +
                               Cn1 * 2904.572311681263 +
                               Cn2 * 307683.539514128 +
                               Cn3 * -9499081.098660842 +
                               Cn4 * -733650490.1129686) +
                      pbar1 * (      -829.605681550582 +
                               Cn1 * -66153.55430502533 +
                               Cn2 * 9103811.609515514 +
                               Cn3 * 291824743.32715774 +
                               Cn4 * -9929869114.41915) +
                      pbar2 * (      641.6835047163152 +
                               Cn1 * -3642565.5966640944 +
                               Cn2 * -408939020.4570888 +
                               Cn3 * 11603515373.067984 +
                               Cn4 * 1044783659797.6678) +
                      pbar3 * (      -20249.308592227608 +
                               Cn1 * -4126927.1355173094 +
                               Cn2 * 289962638.66201293 +
                               Cn3 * 36940881925.69667 +
                               Cn4 * -1665799621383.404) +
                      pbar4 * (      -50615.6474554126 +
                               Cn1 * 311449920.10021424 +
                               Cn2 * 26710058927.478065 +
                               Cn3 * -1353572593444.798 +
                               Cn4 * -64773317283532.516)) +
               Cm1 * (        (      2264.097815785264 +
                               Cn1 * 512925.0180269921 +
                               Cn2 * 20021254.89326508 +
                               Cn3 * -632394130.4764442 +
                               Cn4 * -39287764368.37878) +
                      pbar1 * (      6993.474367183677 +
                               Cn1 * -10821731.1135432 +
                               Cn2 * -890467151.6770012 +
                               Cn3 * 20201074557.765854 +
                               Cn4 * 2271058567963.7373) +
                      pbar2 * (      333856.86421408784 +
                               Cn1 * 94441597.62062387 +
                               Cn2 * 4831564124.289342 +
                               Cn3 * -137672062623.47964 +
                               Cn4 * -7445949115887.288) +
                      pbar3 * (      1844652.7877798027 +
                               Cn1 * 1121771002.564442 +
                               Cn2 * 77303887126.83855 +
                               Cn3 * -2049017180045.327 +
                               Cn4 * -298764614543184.75) +
                      pbar4 * (      -34502320.33861028 +
                               Cn1 * -4923042098.399919 +
                               Cn2 * -364524153500.0866 +
                               Cn3 * -18688004831930.387 +
                               Cn4 * 526121927057657.9)) +
               Cm2 * (        (      -1928.4855662039338 +
                               Cn1 * -1460562.8684024627 +
                               Cn2 * -149284454.40078935 +
                               Cn3 * 5543938423.514274 +
                               Cn4 * 349350191210.56616) +
                      pbar1 * (      769139.7831563947 +
                               Cn1 * 75972906.89009751 +
                               Cn2 * -7523342474.330731 +
                               Cn3 * -317748612749.6309 +
                               Cn4 * 11215528222136.48) +
                      pbar2 * (      12217106.3985407 +
                               Cn1 * 4123486319.8646574 +
                               Cn2 * 267600764571.60886 +
                               Cn3 * -9733870254040.402 +
                               Cn4 * -665060466435569.4) +
                      pbar3 * (      5116104.15370631 +
                               Cn1 * 3968744810.601318 +
                               Cn2 * 566169532562.7772 +
                               Cn3 * -7425645678284.868 +
                               Cn4 * -888757278649198.0) +
                      pbar4 * (      -818040113.7032456 +
                               Cn1 * -336623852279.6835 +
                               Cn2 * -20417662562604.234 +
                               Cn3 * 969261030645790.4 +
                               Cn4 * 5.139535916786412e+16)) +
               Cm3 * (        (      -569224.6016819731 +
                               Cn1 * -143133967.27541524 +
                               Cn2 * -7141695501.035877 +
                               Cn3 * 130881028542.93582 +
                               Cn4 * 10994172638195.979) +
                      pbar1 * (      -14091680.23880944 +
                               Cn1 * 2421859774.661255 +
                               Cn2 * 321269207191.57367 +
                               Cn3 * -5177257439303.254 +
                               Cn4 * -827170563518165.2) +
                      pbar2 * (      -340867777.3474591 +
                               Cn1 * -68132079724.70972 +
                               Cn2 * -2041789489100.1047 +
                               Cn3 * 97984700900636.48 +
                               Cn4 * 4171089575797064.5) +
                      pbar3 * (      -354070008.8452865 +
                               Cn1 * -440656583240.21515 +
                               Cn2 * -29922592528105.734 +
                               Cn3 * 1015890303126505.1 +
                               Cn4 * 1.0390495792068344e+17) +
                      pbar4 * (      27399783097.446392 +
                               Cn1 * 4847399231706.992 +
                               Cn2 * 150643609434663.28 +
                               Cn3 * -215291528600786.53 +
                               Cn4 * -2.3966286792388806e+17)) +
               Cm4 * (        (      149334.29881146096 +
                               Cn1 * 134864537.3028633 +
                               Cn2 * 12776453413.758959 +
                               Cn3 * -550794088640.7402 +
                               Cn4 * -29348505555361.22) +
                      pbar1 * (      -57119122.49811048 +
                               Cn1 * -6902400797.278721 +
                               Cn2 * 647611967579.6116 +
                               Cn3 * 33346468782590.145 +
                               Cn4 * -854936072966725.5) +
                      pbar2 * (      -1178873069.2052999 +
                               Cn1 * -388794280043.9923 +
                               Cn2 * -23708923701960.473 +
                               Cn3 * 940415251711614.9 +
                               Cn4 * 6.085719975126054e+16) +
                      pbar3 * (      -1627237564.406647 +
                               Cn1 * -208574692801.45163 +
                               Cn2 * -47151995046964.53 +
                               Cn3 * -1069414902573887.2 +
                               Cn4 * 3.6846110954823384e+16) +
                      pbar4 * (      76008090497.83888 +
                               Cn1 * 32822352120119.043 +
                               Cn2 * 1931443348700765.5 +
                               Cn3 * -9.928032629853339e+16 +
                               Cn4 * -5.472439519109354e+18)) +
               Cm5 * (        (      33710129.61646312 +
                               Cn1 * 9021745221.85624 +
                               Cn2 * 533975207681.0124 +
                               Cn3 * -6115190107282.775 +
                               Cn4 * -746593973850193.8) +
                      pbar1 * (      1262476007.214191 +
                               Cn1 * -142294131385.03415 +
                               Cn2 * -22922426626452.816 +
                               Cn3 * 335046059366707.6 +
                               Cn4 * 5.79422814279959e+16) +
                      pbar2 * (      30910600172.31796 +
                               Cn1 * 6154087422931.803 +
                               Cn2 * 139016045419384.06 +
                               Cn3 * -9499734044688050.0 +
                               Cn4 * -2.827987745108036e+17) +
                      pbar3 * (      28108598753.01581 +
                               Cn1 * 32793034225235.957 +
                               Cn2 * 2069005762528098.8 +
                               Cn3 * -7.479273933595238e+16 +
                               Cn4 * -6.594522906854527e+18) +
                      pbar4 * (      -2378804928939.88 +
                               Cn1 * -471220249244292.06 +
                               Cn2 * -1.052219288842699e+16 +
                               Cn3 * 3.9522325379706266e+17 +
                               Cn4 * 1.7499800635565085e+19))) +
        CL3 * (      (        (      7.574015972908629 +
                               Cn1 * 343.91721194151836 +
                               Cn2 * -108038.85014628865 +
                               Cn3 * 2678330.2570926514 +
                               Cn4 * 267292783.9471004) +
                      pbar1 * (      460.6346702849962 +
                               Cn1 * 3918.1924086049794 +
                               Cn2 * -7767509.49365114 +
                               Cn3 * -98783134.89637502 +
                               Cn4 * 11346796326.4266) +
                      pbar2 * (      -988.1614890380257 +
                               Cn1 * 2269944.970573906 +
                               Cn2 * 240349793.5979634 +
                               Cn3 * -7486310886.362898 +
                               Cn4 * -619436705607.8107) +
                      pbar3 * (      23423.520029048843 +
                               Cn1 * 5125562.644419886 +
                               Cn2 * -193990747.26194465 +
                               Cn3 * -35646311010.42748 +
                               Cn4 * 896700776940.7974) +
                      pbar4 * (      12444.102799119246 +
                               Cn1 * -186614009.31916875 +
                               Cn2 * -13860726208.748198 +
                               Cn3 * 835314945767.5145 +
                               Cn4 * 31502899782177.72)) +
               Cm1 * (        (      -1488.216220381066 +
                               Cn1 * -305958.4049806533 +
                               Cn2 * -11209047.473176138 +
                               Cn3 * 366667057.11490417 +
                               Cn4 * 23035996679.52564) +
                      pbar1 * (      -7669.42788380853 +
                               Cn1 * 5773294.9156450275 +
                               Cn2 * 428780069.7158508 +
                               Cn3 * -13379998747.499685 +
                               Cn4 * -1219677851033.5793) +
                      pbar2 * (      -263073.19418034254 +
                               Cn1 * -65549408.74815194 +
                               Cn2 * -1492056186.5470173 +
                               Cn3 * 111800308614.61804 +
                               Cn4 * -142285953436.41205) +
                      pbar3 * (      -914348.2639512608 +
                               Cn1 * -592201077.9763587 +
                               Cn2 * -37840939111.92145 +
                               Cn3 * 1316425968135.2278 +
                               Cn4 * 185844060798150.6) +
                      pbar4 * (      26618665.387957368 +
                               Cn1 * 3117299665.914461 +
                               Cn2 * 84078291079.73515 +
                               Cn3 * 14011859525738.26 +
                               Cn4 * 169552872404472.16)) +
               Cm2 * (        (      -3962.9188352667547 +
                               Cn1 * -293009.7742463246 +
                               Cn2 * 36543411.26444297 +
                               Cn3 * -1833024286.181003 +
                               Cn4 * -99378582680.64897) +
                      pbar1 * (      -478729.9104568126 +
                               Cn1 * -26280100.579868767 +
                               Cn2 * 6572828299.928256 +
                               Cn3 * 165060835588.37195 +
                               Cn4 * -10621485867349.34) +
                      pbar2 * (      -6881099.732088236 +
                               Cn1 * -2352719425.455182 +
                               Cn2 * -154993630804.35605 +
                               Cn3 * 5826350490878.236 +
                               Cn4 * 389210474016659.1) +
                      pbar3 * (      -6489390.564863466 +
                               Cn1 * -4536561031.101913 +
                               Cn2 * -451097330878.47626 +
                               Cn3 * 10588860073917.174 +
                               Cn4 * 672910387015495.1) +
                      pbar4 * (      457663830.0218784 +
                               Cn1 * 194604525909.8772 +
                               Cn2 * 11201288754936.518 +
                               Cn3 * -612645339418079.0 +
                               Cn4 * -2.799135058936903e+16)) +
               Cm3 * (        (      406026.06383052265 +
                               Cn1 * 91796562.57458039 +
                               Cn2 * 4295143512.040545 +
                               Cn3 * -84318691786.83551 +
                               Cn4 * -6792597675838.635) +
                      pbar1 * (      10367971.265521381 +
                               Cn1 * -1296320053.6608856 +
                               Cn2 * -182264768689.97083 +
                               Cn3 * 3628312294462.572 +
                               Cn4 * 506773030922020.4) +
                      pbar2 * (      222451219.1636199 +
                               Cn1 * 42420570977.31002 +
                               Cn2 * 730470137098.9679 +
                               Cn3 * -63781050202792.41 +
                               Cn4 * -919076169741012.5) +
                      pbar3 * (      139807604.4317322 +
                               Cn1 * 256251137525.0547 +
                               Cn2 * 17204799420626.463 +
                               Cn3 * -700896610869135.0 +
                               Cn4 * -7.004619894137254e+16) +
                      pbar4 * (      -18043671780.1442 +
                               Cn1 * -2870659301904.0806 +
                               Cn2 * -39095074675952.76 +
                               Cn3 * -1084628087017647.5 +
                               Cn4 * -4.332354934779742e+16)) +
               Cm4 * (        (      288123.7225348568 +
                               Cn1 * 15115036.611297864 +
                               Cn2 * -2780538956.5337744 +
                               Cn3 * 219073369203.36523 +
                               Cn4 * 7909319726197.674) +
                      pbar1 * (      36151668.6944617 +
                               Cn1 * 2555011147.5488963 +
                               Cn2 * -572286661047.8939 +
                               Cn3 * -18816397453722.29 +
                               Cn4 * 807441560415387.5) +
                      pbar2 * (      671892311.1165508 +
                               Cn1 * 220341875143.13248 +
                               Cn2 * 13486994962715.033 +
                               Cn3 * -565768841882373.6 +
                               Cn4 * -3.5235502375981868e+16) +
                      pbar3 * (      1318892874.965257 +
                               Cn1 * 292705922403.13696 +
                               Cn2 * 37756995870009.92 +
                               Cn3 * 392507699235187.3 +
                               Cn4 * -2.1146962352993268e+16) +
                      pbar4 * (      -41285858427.835014 +
                               Cn1 * -19148000123157.0 +
                               Cn2 * -1076780242874707.4 +
                               Cn3 * 6.473533121723519e+16 +
                               Cn4 * 3.1497732830028155e+18)) +
               Cm5 * (        (      -25313343.479782242 +
                               Cn1 * -6010124510.351513 +
                               Cn2 * -331472172743.6644 +
                               Cn3 * 4383277404060.813 +
                               Cn4 * 479893089732214.0) +
                      pbar1 * (      -914140470.4874842 +
                               Cn1 * 77252594807.74843 +
                               Cn2 * 13795642100298.59 +
                               Cn3 * -242283583204629.25 +
                               Cn4 * -3.724899397087343e+16) +
                      pbar2 * (      -19769520584.463825 +
                               Cn1 * -3746803032572.6177 +
                               Cn2 * -44584303082385.31 +
                               Cn3 * 5845169038975317.0 +
                               Cn4 * 4.3959338124958664e+16) +
                      pbar3 * (      -12118889096.978514 +
                               Cn1 * -19500154929840.12 +
                               Cn2 * -1245473357666242.0 +
                               Cn3 * 5.17874135885728e+16 +
                               Cn4 * 4.586914554705826e+18) +
                      pbar4 * (      1521014756115.7932 +
                               Cn1 * 277913844154142.5 +
                               Cn2 * 2446218136647275.0 +
                               Cn3 * -1.51988815556277e+17 +
                               Cn4 * 3.569142421790577e+18))));
    
    R[4] = (
              (      (        (      1.2455187608676301 +
                               Cn1 * 1571.2828909394532 +
                               Cn2 * 35573.54598478812 +
                               Cn3 * -2818633.3667174554 +
                               Cn4 * -65377477.67017491) +
                      pbar1 * (      -32.54619219463298 +
                               Cn1 * 3252.0905125524328 +
                               Cn2 * -463347.48941983085 +
                               Cn3 * 3104688.4589989358 +
                               Cn4 * 2211757028.9070506) +
                      pbar2 * (      282.66794047532744 +
                               Cn1 * -204188.30743219634 +
                               Cn2 * -10486232.927671082 +
                               Cn3 * 790368696.6680723 +
                               Cn4 * 23205707591.787605) +
                      pbar3 * (      2788.449240622573 +
                               Cn1 * 176618.93282126315 +
                               Cn2 * 27128218.710387323 +
                               Cn3 * -3027869509.5871463 +
                               Cn4 * -190668980316.49445) +
                      pbar4 * (      24325.730236880896 +
                               Cn1 * 19787194.37713975 +
                               Cn2 * 977437404.5464168 +
                               Cn3 * -79118792711.54494 +
                               Cn4 * -3234359946806.3403)) +
               Cm1 * (        (      -228.8275619509552 +
                               Cn1 * -29024.999871294713 +
                               Cn2 * -248696.46747413545 +
                               Cn3 * 74027317.57387643 +
                               Cn4 * 1195691735.3512626) +
                      pbar1 * (      -1858.7887892585966 +
                               Cn1 * -241096.582208725 +
                               Cn2 * -1444242.8900023657 +
                               Cn3 * 1075430548.8951864 +
                               Cn4 * 66201751071.974495) +
                      pbar2 * (      -6815.828152584878 +
                               Cn1 * -7094664.750661099 +
                               Cn2 * -280690753.70942473 +
                               Cn3 * 28585614028.32439 +
                               Cn4 * 963677240664.8833) +
                      pbar3 * (      -29994.65818274104 +
                               Cn1 * 44083762.29042647 +
                               Cn2 * 5351464104.321558 +
                               Cn3 * -156791548268.8884 +
                               Cn4 * -21391064178228.83) +
                      pbar4 * (      1554401.5525399377 +
                               Cn1 * 1306140824.0513256 +
                               Cn2 * 54792428595.12832 +
                               Cn3 * -5278987193354.16 +
                               Cn4 * -179931103509836.88)) +
               Cm2 * (        (      1157.0410176390862 +
                               Cn1 * -6150.061708843896 +
                               Cn2 * -11946199.192094928 +
                               Cn3 * 210647983.7117303 +
                               Cn4 * 35240147384.49366) +
                      pbar1 * (      -10623.624271973747 +
                               Cn1 * 4600456.429208327 +
                               Cn2 * 628867847.3170756 +
                               Cn3 * -922034217.5162702 +
                               Cn4 * -1293387474250.9707) +
                      pbar2 * (      -301371.9837570007 +
                               Cn1 * 142813359.3391643 +
                               Cn2 * 9410339520.576199 +
                               Cn3 * -383192468259.1474 +
                               Cn4 * -19691765154056.49) +
                      pbar3 * (      -2067711.4041290004 +
                               Cn1 * -380124982.40011066 +
                               Cn2 * -45372545181.035446 +
                               Cn3 * 74335024914.413 +
                               Cn4 * 84975083990995.78) +
                      pbar4 * (      -15495852.80748873 +
                               Cn1 * -15488264400.292797 +
                               Cn2 * -973328778274.8839 +
                               Cn3 * 36202470572099.445 +
                               Cn4 * 2074088716445481.5)) +
               Cm3 * (        (      66381.8756039967 +
                               Cn1 * 11231206.104601227 +
                               Cn2 * 119687206.44634295 +
                               Cn3 * -23922991240.39368 +
                               Cn4 * -318639981020.21185) +
                      pbar1 * (      411748.1464719808 +
                               Cn1 * 97771534.75468862 +
                               Cn2 * 7925789057.310271 +
                               Cn3 * -225407734511.90445 +
                               Cn4 * -29545446763247.375) +
                      pbar2 * (      8782524.755225167 +
                               Cn1 * 2416512022.900134 +
                               Cn2 * 106776957195.72617 +
                               Cn3 * -9719959112048.209 +
                               Cn4 * -337665992948191.06) +
                      pbar3 * (      9747819.968633393 +
                               Cn1 * -21036981856.255817 +
                               Cn2 * -2163831483576.4844 +
                               Cn3 * 42945922656117.16 +
                               Cn4 * 6670343259618090.0) +
                      pbar4 * (      -1266981078.7662106 +
                               Cn1 * -516614958642.4839 +
                               Cn2 * -21032598582974.676 +
                               Cn3 * 1804212607758201.2 +
                               Cn4 * 6.0006842289111096e+16)) +
               Cm4 * (        (      -72497.41344507817 +
                               Cn1 * 10475820.0966865 +
                               Cn2 * 1830630129.5704772 +
                               Cn3 * -50049860062.9602 +
                               Cn4 * -5711846731199.067) +
                      pbar1 * (      -547243.5140095237 +
                               Cn1 * -586001121.8788923 +
                               Cn2 * -71656546477.41261 +
                               Cn3 * -36472528743.90143 +
                               Cn4 * 128044565970522.02) +
                      pbar2 * (      11867118.65960601 +
                               Cn1 * -11168628020.65772 +
                               Cn2 * -1108572937299.461 +
                               Cn3 * 22459869598713.7 +
                               Cn4 * 2316695948724555.0) +
                      pbar3 * (      260725229.64153227 +
                               Cn1 * 39706460578.71278 +
                               Cn2 * 5165490169114.317 +
                               Cn3 * 61611551222002.95 +
                               Cn4 * -6789751482668092.0) +
                      pbar4 * (      2857261878.8786273 +
                               Cn1 * 1134250369910.4011 +
                               Cn2 * 98138068180339.39 +
                               Cn3 * -1457912218725548.0 +
                               Cn4 * -1.769205786039212e+17)) +
               Cm5 * (        (      -4622283.685911559 +
                               Cn1 * -729312460.4515951 +
                               Cn2 * -2351388689.682817 +
                               Cn3 * 1374783607294.9983 +
                               Cn4 * -1227476852836.7778) +
                      pbar1 * (      -35389117.51518674 +
                               Cn1 * -11432031590.107407 +
                               Cn2 * -761177785122.689 +
                               Cn3 * 18703191485756.293 +
                               Cn4 * 2072504137052882.5) +
                      pbar2 * (      -708563680.7184632 +
                               Cn1 * -205493776876.86172 +
                               Cn2 * -8872556295296.635 +
                               Cn3 * 701398683400654.9 +
                               Cn4 * 2.419047197264637e+16) +
                      pbar3 * (      -374740297.3874635 +
                               Cn1 * 1896918203307.3645 +
                               Cn2 * 166332160479686.3 +
                               Cn3 * -2987070104527542.5 +
                               Cn4 * -4.337360876843688e+17) +
                      pbar4 * (      103810322883.62318 +
                               Cn1 * 40952676125826.26 +
                               Cn2 * 1541721355290867.5 +
                               Cn3 * -1.2449704091826406e+17 +
                               Cn4 * -3.80031055604901e+18))) +
        CL1 * (      (        (      25.445585306629827 +
                               Cn1 * 1939.93377432952 +
                               Cn2 * -243096.23588001123 +
                               Cn3 * -5167479.411389859 +
                               Cn4 * 401977096.584264) +
                      pbar1 * (      34.12164460430546 +
                               Cn1 * 10105.753978633049 +
                               Cn2 * 4502946.419901476 +
                               Cn3 * -151674283.9816566 +
                               Cn4 * -27343792840.938396) +
                      pbar2 * (      -3989.43922506351 +
                               Cn1 * 2404336.5005565826 +
                               Cn2 * 225348737.8527921 +
                               Cn3 * -9747423552.58807 +
                               Cn4 * -540309504265.60376) +
                      pbar3 * (      -54027.05514721682 +
                               Cn1 * -710801.5044125877 +
                               Cn2 * 59163196.83755401 +
                               Cn3 * 38847418238.10806 +
                               Cn4 * 2191865657957.9634) +
                      pbar4 * (      -557215.6294112252 +
                               Cn1 * -237470733.85427338 +
                               Cn2 * -15546935685.362043 +
                               Cn3 * 1052364480398.7384 +
                               Cn4 * 53665896178961.72)) +
               Cm1 * (        (      2328.7527167212565 +
                               Cn1 * 373621.3039956499 +
                               Cn2 * 2439602.4432187625 +
                               Cn3 * -1016705640.7977457 +
                               Cn4 * -14483759746.838076) +
                      pbar1 * (      22909.32395708792 +
                               Cn1 * 3907540.4651714545 +
                               Cn2 * 67200056.4940638 +
                               Cn3 * -16807746967.366728 +
                               Cn4 * -1014293670660.9879) +
                      pbar2 * (      205931.89396141627 +
                               Cn1 * 118170364.10989988 +
                               Cn2 * 6104555878.914487 +
                               Cn3 * -401953788273.2246 +
                               Cn4 * -19796082874418.574) +
                      pbar3 * (      -412881.1626612603 +
                               Cn1 * -669665228.5448622 +
                               Cn2 * -60446580662.70851 +
                               Cn3 * 2432164355641.073 +
                               Cn4 * 268978958984396.2) +
                      pbar4 * (      -35189608.50220994 +
                               Cn1 * -19550547619.296463 +
                               Cn2 * -869500008081.3026 +
                               Cn3 * 73113897381251.53 +
                               Cn4 * 2888825694891588.5)) +
               Cm2 * (        (      -7912.161982203899 +
                               Cn1 * 1487708.729932496 +
                               Cn2 * 219900995.23104328 +
                               Cn3 * -6730198736.853797 +
                               Cn4 * -654683566553.9707) +
                      pbar1 * (      159533.721300133 +
                               Cn1 * -24735438.088730987 +
                               Cn2 * -5633021707.545242 +
                               Cn3 * -78223961345.21593 +
                               Cn4 * 10349707277622.998) +
                      pbar2 * (      2350521.427575609 +
                               Cn1 * -1787713112.7264805 +
                               Cn2 * -143543574236.00046 +
                               Cn3 * 5055211812545.569 +
                               Cn4 * 327966937829295.8) +
                      pbar3 * (      13707515.887282372 +
                               Cn1 * -3193863169.945498 +
                               Cn2 * 181426085324.90732 +
                               Cn3 * 16962948246192.266 +
                               Cn4 * -334086327234510.7) +
                      pbar4 * (      278655650.6301141 +
                               Cn1 * 151838531435.0912 +
                               Cn2 * 11573833349487.023 +
                               Cn3 * -421865225285632.2 +
                               Cn4 * -2.6863502987827984e+16)) +
               Cm3 * (        (      -734805.5889828624 +
                               Cn1 * -112979708.25339197 +
                               Cn2 * -153411262.3835285 +
                               Cn3 * 244656636710.38184 +
                               Cn4 * 709159991862.224) +
                      pbar1 * (      -6397552.346022835 +
                               Cn1 * -1280635933.806966 +
                               Cn2 * -64358551466.16631 +
                               Cn3 * 2998511025999.317 +
                               Cn4 * 295195002297355.25) +
                      pbar2 * (      -167132682.06864727 +
                               Cn1 * -43921237440.94121 +
                               Cn2 * -2132891529760.6018 +
                               Cn3 * 148606279417733.16 +
                               Cn4 * 6604748941643236.0) +
                      pbar3 * (      -147831442.7281985 +
                               Cn1 * 223184205585.89963 +
                               Cn2 * 19425891228947.72 +
                               Cn3 * -480523931812944.25 +
                               Cn4 * -6.9263494626025304e+16) +
                      pbar4 * (      20261065515.52114 +
                               Cn1 * 7337483125192.697 +
                               Cn2 * 295027457950702.94 +
                               Cn3 * -2.500687067380892e+16 +
                               Cn4 * -8.760908093685829e+17)) +
               Cm4 * (        (      -39138.077547652974 +
                               Cn1 * -298446128.89130867 +
                               Cn2 * -25394422783.344604 +
                               Cn3 * 1015125889373.2902 +
                               Cn4 * 76411946319521.11) +
                      pbar1 * (      -4280422.996214429 +
                               Cn1 * 2211608473.008672 +
                               Cn2 * 633602214737.794 +
                               Cn3 * 14020469366928.086 +
                               Cn4 * -906297043996439.1) +
                      pbar2 * (      -11852012.459392771 +
                               Cn1 * 120181156004.54666 +
                               Cn2 * 13166493235034.973 +
                               Cn3 * -244604353529713.16 +
                               Cn4 * -2.63764147896357e+16) +
                      pbar3 * (      -1298232247.7093506 +
                               Cn1 * 370644386229.7527 +
                               Cn2 * -25986839760651.125 +
                               Cn3 * -2798612307352610.0 +
                               Cn4 * 3861521271869450.5) +
                      pbar4 * (      -39889644333.89341 +
                               Cn1 * -8453025619693.299 +
                               Cn2 * -962745198096950.0 +
                               Cn3 * 9442040786031370.0 +
                               Cn4 * 1.5623467860596342e+18)) +
               Cm5 * (        (      45453898.41789895 +
                               Cn1 * 6439852146.725347 +
                               Cn2 * -57109608289.31424 +
                               Cn3 * -12008435590857.6 +
                               Cn4 * 199114044612867.53) +
                      pbar1 * (      447407495.5172196 +
                               Cn1 * 118801565628.97899 +
                               Cn2 * 5635380771558.137 +
                               Cn3 * -166644613688622.84 +
                               Cn4 * -1.6708396172676418e+16) +
                      pbar2 * (      13303088476.383823 +
                               Cn1 * 3442643024308.2124 +
                               Cn2 * 145391931342243.94 +
                               Cn3 * -1.0433748022002186e+16 +
                               Cn4 * -3.920575642153472e+17) +
                      pbar3 * (      19283439124.235374 +
                               Cn1 * -17197610605073.41 +
                               Cn2 * -1362587804380829.0 +
                               Cn3 * 2.3580626862739164e+16 +
                               Cn4 * 3.941961325045334e+18) +
                      pbar4 * (      -1567673220135.2627 +
                               Cn1 * -551073496877965.44 +
                               Cn2 * -1.930410899695198e+16 +
                               Cn3 * 1.6832187317850883e+18 +
                               Cn4 * 4.860765807084577e+19))) +
        CL2 * (      (        (      -55.46424137011178 +
                               Cn1 * -1957.2079092538524 +
                               Cn2 * 689411.4560907622 +
                               Cn3 * 8938068.403091865 +
                               Cn4 * -1083437318.7198353) +
                      pbar1 * (      1063.7467270315492 +
                               Cn1 * -43211.79911873476 +
                               Cn2 * -24479559.474213563 +
                               Cn3 * 455744732.5259887 +
                               Cn4 * 97606821441.6867) +
                      pbar2 * (      14992.619117686096 +
                               Cn1 * -6160235.651840919 +
                               Cn2 * -598212041.213089 +
                               Cn3 * 22935798772.10351 +
                               Cn4 * 1322573286824.7915) +
                      pbar3 * (      114421.7064690475 +
                               Cn1 * 7255900.254106456 +
                               Cn2 * 322049905.29629076 +
                               Cn3 * -154197871615.0065 +
                               Cn4 * -8232748817687.742) +
                      pbar4 * (      1035938.3583576843 +
                               Cn1 * 575968980.8584121 +
                               Cn2 * 43268720118.66952 +
                               Cn3 * -2634950492150.2183 +
                               Cn4 * -155240667132269.03)) +
               Cm1 * (        (      -6880.415855142291 +
                               Cn1 * -1098136.0729989028 +
                               Cn2 * -3217370.8716287087 +
                               Cn3 * 3048877816.4744215 +
                               Cn4 * 37049505870.30999) +
                      pbar1 * (      -84660.29470018693 +
                               Cn1 * -12458258.815842401 +
                               Cn2 * -111013237.3183199 +
                               Cn3 * 48088582421.41411 +
                               Cn4 * 2484344910779.764) +
                      pbar2 * (      -670129.2651813666 +
                               Cn1 * -257107418.28929833 +
                               Cn2 * -15828552887.407066 +
                               Cn3 * 795177056146.0045 +
                               Cn4 * 49481092540881.96) +
                      pbar3 * (      2099697.4727074862 +
                               Cn1 * 1998053728.4058554 +
                               Cn2 * 164971439331.15277 +
                               Cn3 * -7000956164796.484 +
                               Cn4 * -731964447797937.5) +
                      pbar4 * (      93732772.91168223 +
                               Cn1 * 46873763435.79876 +
                               Cn2 * 2262849935963.7505 +
                               Cn3 * -171941742361442.8 +
                               Cn4 * -7372175163073478.0)) +
               Cm2 * (        (      -1227.281544419491 +
                               Cn1 * -8348237.6396184 +
                               Cn2 * -674684378.0433335 +
                               Cn3 * 27860179206.167988 +
                               Cn4 * 2014920813452.0789) +
                      pbar1 * (      -701576.8974757708 +
                               Cn1 * 41764908.1716888 +
                               Cn2 * 19003532601.400417 +
                               Cn3 * 203321980551.19934 +
                               Cn4 * -40292781968043.61) +
                      pbar2 * (      -6583756.054175813 +
                               Cn1 * 4611932516.233816 +
                               Cn2 * 414537338503.71985 +
                               Cn3 * -13487833200199.434 +
                               Cn4 * -968556905051215.5) +
                      pbar3 * (      -20023889.292206403 +
                               Cn1 * 8580782940.003543 +
                               Cn2 * -867272121036.5471 +
                               Cn3 * -18377784201353.777 +
                               Cn4 * 3354894610103795.5) +
                      pbar4 * (      -551783711.939982 +
                               Cn1 * -384185299082.286 +
                               Cn2 * -34913457272315.715 +
                               Cn3 * 1303894068127828.0 +
                               Cn4 * 9.342939628276589e+16)) +
               Cm3 * (        (      1906809.9196366058 +
                               Cn1 * 282173089.32669574 +
                               Cn2 * -2039765542.620999 +
                               Cn3 * -639000187155.5907 +
                               Cn4 * 3542439317910.296) +
                      pbar1 * (      21655057.111522082 +
                               Cn1 * 4334632963.029355 +
                               Cn2 * 160847507382.7971 +
                               Cn3 * -10348222623785.408 +
                               Cn4 * -774950403957041.1) +
                      pbar2 * (      429754474.3892386 +
                               Cn1 * 106206697808.29156 +
                               Cn2 * 6130939818109.513 +
                               Cn3 * -332060118512928.7 +
                               Cn4 * -1.8459104689767132e+16) +
                      pbar3 * (      255361264.94254255 +
                               Cn1 * -687799735661.6289 +
                               Cn2 * -53487231275857.83 +
                               Cn3 * 1691880055091034.2 +
                               Cn4 * 2.0126402306314128e+17) +
                      pbar4 * (      -48284730834.99727 +
                               Cn1 * -18294953658806.777 +
                               Cn2 * -813599642426835.1 +
                               Cn3 * 6.2123332587946456e+16 +
                               Cn4 * 2.435841233552058e+18)) +
               Cm4 * (        (      1641883.308154597 +
                               Cn1 * 1105758561.8792355 +
                               Cn2 * 76304559481.20071 +
                               Cn3 * -3289759539629.0054 +
                               Cn4 * -223503351453462.56) +
                      pbar1 * (      41687089.426122375 +
                               Cn1 * -2776959344.371219 +
                               Cn2 * -1919780259244.4036 +
                               Cn3 * -35881293630984.14 +
                               Cn4 * 3502090209874613.0) +
                      pbar2 * (      154970226.19297156 +
                               Cn1 * -323862193816.3156 +
                               Cn2 * -38844365471656.086 +
                               Cn3 * 749768803909296.8 +
                               Cn4 * 8.179631493543872e+16) +
                      pbar3 * (      1534703729.755395 +
                               Cn1 * -1045394339207.9755 +
                               Cn2 * 94839514182332.44 +
                               Cn3 * 4826356296163665.0 +
                               Cn4 * -2.3962203047184797e+17) +
                      pbar4 * (      80013915100.34688 +
                               Cn1 * 23042862626178.984 +
                               Cn2 * 3036662338712900.0 +
                               Cn3 * -5.588500027404777e+16 +
                               Cn4 * -6.577705050827939e+18)) +
               Cm5 * (        (      -115000978.84923045 +
                               Cn1 * -15176286470.803843 +
                               Cn2 * 359388578609.65857 +
                               Cn3 * 30706140448403.01 +
                               Cn4 * -969262182563451.0) +
                      pbar1 * (      -1413969420.0841808 +
                               Cn1 * -364775321083.2309 +
                               Cn2 * -14167636581514.82 +
                               Cn3 * 620519124317324.5 +
                               Cn4 * 4.626714842628598e+16) +
                      pbar2 * (      -33155249774.442425 +
                               Cn1 * -8565594435637.41 +
                               Cn2 * -443057129444160.44 +
                               Cn3 * 2.403202630839636e+16 +
                               Cn4 * 1.188597366407844e+18) +
                      pbar3 * (      -46716543275.719086 +
                               Cn1 * 51641023505590.1 +
                               Cn2 * 3737613171845127.0 +
                               Cn3 * -1.0071899041882698e+17 +
                               Cn4 * -1.2204208665889993e+19) +
                      pbar4 * (      3641320415752.5615 +
                               Cn1 * 1392420865019185.5 +
                               Cn2 * 5.583879115314917e+16 +
                               Cn3 * -4.301798702330621e+18 +
                               Cn4 * -1.488650096958478e+20))) +
        CL3 * (      (        (      10.49446119431523 +
                               Cn1 * -3219.038829911403 +
                               Cn2 * -513473.31096400233 +
                               Cn3 * 2466780.221765769 +
                               Cn4 * 813293292.5567265) +
                      pbar1 * (      -1351.4134888896579 +
                               Cn1 * 34141.230015748144 +
                               Cn2 * 24003375.346025806 +
                               Cn3 * -321409053.86474407 +
                               Cn4 * -80518924453.99687) +
                      pbar2 * (      -12456.547337380705 +
                               Cn1 * 4409248.740565378 +
                               Cn2 * 425599223.7124036 +
                               Cn3 * -14563661674.16021 +
                               Cn4 * -865921466969.7828) +
                      pbar3 * (      -56900.55262268183 +
                               Cn1 * -7664881.147891031 +
                               Cn2 * -538041405.3328041 +
                               Cn3 * 134230946747.77823 +
                               Cn4 * 7045792059588.87) +
                      pbar4 * (      -441912.2992544336 +
                               Cn1 * -392938837.914989 +
                               Cn2 * -32987592865.33978 +
                               Cn3 * 1764272039588.8083 +
                               Cn4 * 117595338060320.88)) +
               Cm1 * (        (      4831.0274242936985 +
                               Cn1 * 763882.97093871 +
                               Cn2 * 220339.44009984974 +
                               Cn3 * -2164398022.991562 +
                               Cn4 * -23179623506.975018) +
                      pbar1 * (      64214.15632464209 +
                               Cn1 * 9531731.382203095 +
                               Cn2 * 129553190.64956224 +
                               Cn3 * -32331051417.10137 +
                               Cn4 * -1690385205809.7622) +
                      pbar2 * (      501508.0817236209 +
                               Cn1 * 152119887.50885528 +
                               Cn2 * 10636365598.909601 +
                               Cn3 * -417102466070.1111 +
                               Cn4 * -31334327716193.4) +
                      pbar3 * (      -1600753.3163676688 +
                               Cn1 * -1472766506.1960077 +
                               Cn2 * -122221716351.98708 +
                               Cn3 * 4758624552199.972 +
                               Cn4 * 508545360716736.8) +
                      pbar4 * (      -62694570.550595514 +
                               Cn1 * -29618246920.395805 +
                               Cn2 * -1524294125082.5789 +
                               Cn3 * 104715151340380.1 +
                               Cn4 * 4746669056178834.0)) +
               Cm2 * (        (      12302.975168141427 +
                               Cn1 * 7996196.878395128 +
                               Cn2 * 511734357.6505854 +
                               Cn3 * -23552680157.5434 +
                               Cn4 * -1496945670598.0002) +
                      pbar1 * (      696873.1296198717 +
                               Cn1 * -17836336.924689665 +
                               Cn2 * -15930345282.375929 +
                               Cn3 * -127367729198.98453 +
                               Cn4 * 36276458689313.41) +
                      pbar2 * (      5359758.707835006 +
                               Cn1 * -3172638584.46763 +
                               Cn2 * -304726808598.0915 +
                               Cn3 * 9331587906955.975 +
                               Cn4 * 708869083646460.5) +
                      pbar3 * (      3374234.408082305 +
                               Cn1 * -4981913839.027576 +
                               Cn2 * 873880372609.0236 +
                               Cn3 * -4710188335330.538 +
                               Cn4 * -3764504335396121.0) +
                      pbar4 * (      253689343.9113864 +
                               Cn1 * 265894241328.7239 +
                               Cn2 * 26782927525632.715 +
                               Cn3 * -998104696826710.2 +
                               Cn4 * -7.635990171232123e+16)) +
               Cm3 * (        (      -1294627.961820681 +
                               Cn1 * -185652617.9706727 +
                               Cn2 * 2622766303.507039 +
                               Cn3 * 437821035712.1931 +
                               Cn4 * -4889512945578.561) +
                      pbar1 * (      -16660330.00004024 +
                               Cn1 * -3420226120.555495 +
                               Cn2 * -119694064888.11728 +
                               Cn3 * 7753698125621.528 +
                               Cn4 * 536436697999252.5) +
                      pbar2 * (      -285413444.28252697 +
                               Cn1 * -66797940966.58381 +
                               Cn2 * -4340775206318.34 +
                               Cn3 * 192544590683022.62 +
                               Cn4 * 1.2574313869605736e+16) +
                      pbar3 * (      -114769769.27725734 +
                               Cn1 * 527050963380.5776 +
                               Cn2 * 39560989086630.13 +
                               Cn3 * -1302310614563502.8 +
                               Cn4 * -1.463419099557052e+17) +
                      pbar4 * (      30466808244.70042 +
                               Cn1 * 11947983753320.291 +
                               Cn2 * 570835872141043.1 +
                               Cn3 * -3.973858428346572e+16 +
                               Cn4 * -1.6740730820178662e+18)) +
               Cm4 * (        (      -1887327.3684932017 +
                               Cn1 * -912424015.477186 +
                               Cn2 * -57028903619.55124 +
                               Cn3 * 2507122627704.2383 +
                               Cn4 * 162805854233431.7) +
                      pbar1 * (      -47528136.40827455 +
                               Cn1 * 814825749.5125364 +
                               Cn2 * 1516584356031.012 +
                               Cn3 * 22533205173873.453 +
                               Cn4 * -3142620211637383.0) +
                      pbar2 * (      -215672877.65399417 +
                               Cn1 * 228259260062.71298 +
                               Cn2 * 28948397569843.375 +
                               Cn3 * -558963022238485.7 +
                               Cn4 * -6.2083009084119144e+16) +
                      pbar3 * (      -116439985.1088752 +
                               Cn1 * 631901167341.0438 +
                               Cn2 * -86805574161431.55 +
                               Cn3 * -1639165387951952.5 +
                               Cn4 * 3.007865363744085e+17) +
                      pbar4 * (      -40609192963.79144 +
                               Cn1 * -16996701850089.098 +
                               Cn2 * -2394745597919002.5 +
                               Cn3 * 5.3993767350087256e+16 +
                               Cn4 * 5.860502125589524e+18)) +
               Cm5 * (        (      78056014.35410628 +
                               Cn1 * 9726003926.703085 +
                               Cn2 * -349084980206.80164 +
                               Cn3 * -21098386771823.773 +
                               Cn4 * 868865095558597.0) +
                      pbar1 * (      1078887857.451567 +
                               Cn1 * 277843794493.35004 +
                               Cn2 * 10228720372404.61 +
                               Cn3 * -487989672046183.25 +
                               Cn4 * -3.3058059288501348e+16) +
                      pbar2 * (      21568044426.42836 +
                               Cn1 * 5525279022424.356 +
                               Cn2 * 325103284116783.1 +
                               Cn3 * -1.4350369260711276e+16 +
                               Cn4 * -8.515983062658931e+17) +
                      pbar3 * (      28086633521.93816 +
                               Cn1 * -39541161064667.375 +
                               Cn2 * -2779301233629863.0 +
                               Cn3 * 8.446127603044717e+16 +
                               Cn4 * 9.24337464512132e+18) +
                      pbar4 * (      -2261684641986.9272 +
                               Cn1 * -923345609646310.4 +
                               Cn2 * -4.052162294708934e+16 +
                               Cn3 * 2.8194985687540357e+18 +
                               Cn4 * 1.0854839220556112e+20))));
    
    L[0] = (
              (      (        (      -4.492983210463115 +
                               Cn1 * 138.5305898999188 +
                               Cn2 * 16737.5918092919 +
                               Cn3 * -1836638.982433552 +
                               Cn4 * 69895909.15103422) +
                      pbar1 * (      16.846109217768053 +
                               Cn1 * 5040.810516478376 +
                               Cn2 * -339026.09722435503 +
                               Cn3 * 28508643.765248865 +
                               Cn4 * -678400039.8057613) +
                      pbar2 * (      608.9054845849358 +
                               Cn1 * -161097.4268445196 +
                               Cn2 * 1287036.8770361114 +
                               Cn3 * 1680205704.105332 +
                               Cn4 * -88327222653.55806) +
                      pbar3 * (      237.4398269085457 +
                               Cn1 * 437403.0484224588 +
                               Cn2 * 26426011.924845383 +
                               Cn3 * -7360377524.798566 +
                               Cn4 * 94355919863.36456) +
                      pbar4 * (      -62481.33760698791 +
                               Cn1 * 14695155.571949603 +
                               Cn2 * 214051293.13333854 +
                               Cn3 * -152580824522.87396 +
                               Cn4 * 6062722841012.02)) +
               Cm1 * (        (      -19.452865164710413 +
                               Cn1 * -5632.699056706681 +
                               Cn2 * -621631.6898134756 +
                               Cn3 * 34049415.780274116 +
                               Cn4 * 1242421023.8520288) +
                      pbar1 * (      -273.71405786227666 +
                               Cn1 * 79390.08863368655 +
                               Cn2 * 9220765.617625449 +
                               Cn3 * -1005853838.6727655 +
                               Cn4 * 4604405424.518141) +
                      pbar2 * (      -11369.40871603508 +
                               Cn1 * -1815485.7311140778 +
                               Cn2 * 565576656.4026161 +
                               Cn3 * 20398268744.678864 +
                               Cn4 * -2877642564074.159) +
                      pbar3 * (      65012.871012789205 +
                               Cn1 * -18152294.16070981 +
                               Cn2 * -1624643800.963049 +
                               Cn3 * 197740766911.562 +
                               Cn4 * -1282136798305.2805) +
                      pbar4 * (      584482.9010972523 +
                               Cn1 * 232523251.81460527 +
                               Cn2 * -46076812533.53973 +
                               Cn3 * -2790918734068.085 +
                               Cn4 * 307702296109541.56)) +
               Cm2 * (        (      748.1337874638104 +
                               Cn1 * -40586.3664415374 +
                               Cn2 * -13954385.365965335 +
                               Cn3 * 831317572.1844124 +
                               Cn4 * -4745956588.65144) +
                      pbar1 * (      -978.2947936637945 +
                               Cn1 * 3290729.32948779 +
                               Cn2 * 4516929.55664088 +
                               Cn3 * -33051270059.74185 +
                               Cn4 * 836098392531.5371) +
                      pbar2 * (      -396711.95195109525 +
                               Cn1 * 54577905.188604236 +
                               Cn2 * 5209423106.703974 +
                               Cn3 * -564864854411.5908 +
                               Cn4 * 10218824178197.11) +
                      pbar3 * (      -326177.6509725444 +
                               Cn1 * -691527494.3220111 +
                               Cn2 * 7426097765.00321 +
                               Cn3 * 6611855660120.169 +
                               Cn4 * -145169953585573.22) +
                      pbar4 * (      45166281.73130018 +
                               Cn1 * -3778812534.6412168 +
                               Cn2 * -770788741305.0801 +
                               Cn3 * 39011969711678.01 +
                               Cn4 * 1050997798061594.1)) +
               Cm3 * (        (      4945.724734829092 +
                               Cn1 * 1645925.7847400443 +
                               Cn2 * -55329850.83778257 +
                               Cn3 * -8232841003.795474 +
                               Cn4 * 153483710439.57425) +
                      pbar1 * (      129648.34404774591 +
                               Cn1 * 50978725.042979196 +
                               Cn2 * -6568643800.638239 +
                               Cn3 * -190429566599.80768 +
                               Cn4 * 21287264270894.24) +
                      pbar2 * (      -1959195.4763416788 +
                               Cn1 * -8268093.909262883 +
                               Cn2 * 13736680790.365389 +
                               Cn3 * -3765975741441.138 +
                               Cn4 * 340858406913058.0) +
                      pbar3 * (      -27174000.690157503 +
                               Cn1 * -5684662721.696786 +
                               Cn2 * 1128425276989.331 +
                               Cn3 * 16345902844792.96 +
                               Cn4 * -2989991360453568.0) +
                      pbar4 * (      313579963.13468343 +
                               Cn1 * 3293447913.3954444 +
                               Cn2 * -4611356314371.24 +
                               Cn3 * 448558248857842.56 +
                               Cn4 * -2.8364715217081612e+16)) +
               Cm4 * (        (      -62678.02609675944 +
                               Cn1 * 1495538.557331557 +
                               Cn2 * 1385220927.2283485 +
                               Cn3 * -48290367058.984436 +
                               Cn4 * -998051334249.5059) +
                      pbar1 * (      195659.0686654108 +
                               Cn1 * -428473822.8336179 +
                               Cn2 * 1642765030.1719992 +
                               Cn3 * 3671634287755.3574 +
                               Cn4 * -83237683924950.83) +
                      pbar2 * (      43254688.29678794 +
                               Cn1 * -2480311098.470748 +
                               Cn2 * -837458213348.5586 +
                               Cn3 * 32334518137944.68 +
                               Cn4 * 917287033271901.2) +
                      pbar3 * (      26929929.76885492 +
                               Cn1 * 82581013894.04474 +
                               Cn2 * -1457727944566.3997 +
                               Cn3 * -739061352401614.0 +
                               Cn4 * 1.7598169995480594e+16) +
                      pbar4 * (      -4979794684.259125 +
                               Cn1 * 65277346173.07103 +
                               Cn2 * 112691888112805.52 +
                               Cn3 * -1450040362891091.8 +
                               Cn4 * -3.1294887585136237e+17)) +
               Cm5 * (        (      -569297.9573919752 +
                               Cn1 * -123111027.8836357 +
                               Cn2 * 11931123582.981153 +
                               Cn3 * 530617076460.22144 +
                               Cn4 * -29169587481008.48) +
                      pbar1 * (      -9339107.070567798 +
                               Cn1 * -6532616232.712482 +
                               Cn2 * 574312169018.0511 +
                               Cn3 * 36311878273481.77 +
                               Cn4 * -2325508393385598.0) +
                      pbar2 * (      323896634.30361456 +
                               Cn1 * 21825802241.0005 +
                               Cn2 * -7870316263902.568 +
                               Cn3 * 159375900922702.9 +
                               Cn4 * 295115619008655.94) +
                      pbar3 * (      1974357680.2348888 +
                               Cn1 * 942035361497.8203 +
                               Cn2 * -101304213333246.7 +
                               Cn3 * -5502299978074260.0 +
                               Cn4 * 3.669776549669234e+17) +
                      pbar4 * (      -39622488692.584526 +
                               Cn1 * -3432087081471.5 +
                               Cn2 * 1052571825517885.4 +
                               Cn3 * -1.1011039537750576e+16 +
                               Cn4 * -1.2410241118331753e+18))) +
        CL1 * (      (        (      14.424261134190452 +
                               Cn1 * 238.33182840717538 +
                               Cn2 * -306320.2713710955 +
                               Cn3 * 12870363.947536279 +
                               Cn4 * -30356846.598305468) +
                      pbar1 * (      -73.25117119684795 +
                               Cn1 * -7882.063661220275 +
                               Cn2 * 4905057.952988281 +
                               Cn3 * -51755136.79400937 +
                               Cn4 * -5026196712.578343) +
                      pbar2 * (      -2826.5240361723186 +
                               Cn1 * 987882.1286562665 +
                               Cn2 * 11983747.740375733 +
                               Cn3 * -12288673301.658365 +
                               Cn4 * 715219932649.0974) +
                      pbar3 * (      3271.105033269791 +
                               Cn1 * -2266828.857458848 +
                               Cn2 * -590246695.8510767 +
                               Cn3 * 31518382615.911556 +
                               Cn4 * 1280468388866.7732) +
                      pbar4 * (      328932.3723152821 +
                               Cn1 * -124804503.13371012 +
                               Cn2 * -1034129314.2810622 +
                               Cn3 * 1310793392395.3801 +
                               Cn4 * -60228114607193.8)) +
               Cm1 * (        (      -49.751540130936306 +
                               Cn1 * 70201.77349247079 +
                               Cn2 * 3340816.8760062438 +
                               Cn3 * -674813521.8254442 +
                               Cn4 * 16180115504.764147) +
                      pbar1 * (      3680.9075604909413 +
                               Cn1 * 842264.0110171112 +
                               Cn2 * -186492770.33895835 +
                               Cn3 * 3300337397.250003 +
                               Cn4 * 433674117117.0222) +
                      pbar2 * (      73731.94592112752 +
                               Cn1 * 486167.80198901065 +
                               Cn2 * -2724364904.1055794 +
                               Cn3 * -19847033267.451115 +
                               Cn4 * 11134458399351.832) +
                      pbar3 * (      -663467.3259536813 +
                               Cn1 * 55653163.02030782 +
                               Cn2 * 23256637952.465343 +
                               Cn3 * -1590227511402.4673 +
                               Cn4 * -25985883412022.26) +
                      pbar4 * (      -3326812.582815546 +
                               Cn1 * -1155259082.552843 +
                               Cn2 * 270646353475.65652 +
                               Cn3 * 14924045043710.531 +
                               Cn4 * -1885955900990261.2)) +
               Cm2 * (        (      -5845.818356346642 +
                               Cn1 * 23361.1653721219 +
                               Cn2 * 184378697.2777851 +
                               Cn3 * -5587356544.275403 +
                               Cn4 * -138911663396.13834) +
                      pbar1 * (      51702.7836300893 +
                               Cn1 * -11960415.734650979 +
                               Cn2 * -1878912269.3276734 +
                               Cn3 * 146483432952.3123 +
                               Cn4 * 95752386036.88927) +
                      pbar2 * (      2177212.738663265 +
                               Cn1 * -405568073.3857265 +
                               Cn2 * -40447035444.107086 +
                               Cn3 * 4143522218035.314 +
                               Cn4 * -125711172751042.05) +
                      pbar3 * (      532873.746485315 +
                               Cn1 * 4029249092.8386226 +
                               Cn2 * 130030061953.04735 +
                               Cn3 * -41397578652594.266 +
                               Cn4 * 414942835993337.25) +
                      pbar4 * (      -247410550.7660619 +
                               Cn1 * 39234930625.38094 +
                               Cn2 * 4469616462038.447 +
                               Cn3 * -338081002748433.75 +
                               Cn4 * 1068714214465678.2)) +
               Cm3 * (        (      -58666.43395705312 +
                               Cn1 * -21272024.168691095 +
                               Cn2 * 671499543.464333 +
                               Cn3 * 163601667152.8154 +
                               Cn4 * -6890643126783.729) +
                      pbar1 * (      -1389236.6128521801 +
                               Cn1 * -729182063.8666103 +
                               Cn2 * 82912319854.00447 +
                               Cn3 * 1493515857194.1924 +
                               Cn4 * -250997481394517.75) +
                      pbar2 * (      36099589.81985889 +
                               Cn1 * 3193545339.2552323 +
                               Cn2 * -717604361773.2148 +
                               Cn3 * 8879668004156.068 +
                               Cn4 * -76464818963793.56) +
                      pbar3 * (      251675168.51526552 +
                               Cn1 * 73713724229.4029 +
                               Cn2 * -12943358291479.75 +
                               Cn3 * -47240887540863.836 +
                               Cn4 * 3.4631608109807668e+16) +
                      pbar4 * (      -4142117120.9381957 +
                               Cn1 * -238097396274.23538 +
                               Cn2 * 87713277131075.11 +
                               Cn3 * -2931981892052580.5 +
                               Cn4 * 9.771388754034294e+16)) +
               Cm4 * (        (      516782.15701124736 +
                               Cn1 * 1687193.8057265535 +
                               Cn2 * -16154575488.18165 +
                               Cn3 * 364059517873.378 +
                               Cn4 * 16130095826866.61) +
                      pbar1 * (      -5549383.682635596 +
                               Cn1 * 2352612820.083056 +
                               Cn2 * 147752256108.91788 +
                               Cn3 * -21204479682624.9 +
                               Cn4 * 153794714740713.56) +
                      pbar2 * (      -287319442.6006198 +
                               Cn1 * 19753655171.8911 +
                               Cn2 * 7057392010685.233 +
                               Cn3 * -252436384834166.25 +
                               Cn4 * -3609569266151482.5) +
                      pbar3 * (      74901421.03477366 +
                               Cn1 * -559922714971.8875 +
                               Cn2 * -5777431851229.029 +
                               Cn3 * 5267153185480737.0 +
                               Cn4 * -8.701814505248152e+16) +
                      pbar4 * (      32579399544.71806 +
                               Cn1 * -1274592673136.373 +
                               Cn2 * -832272238746396.4 +
                               Cn3 * 1.4754415766808592e+16 +
                               Cn4 * 1.851175429491378e+18)) +
               Cm5 * (        (      6431444.314037884 +
                               Cn1 * 1561100544.5657566 +
                               Cn2 * -113470007549.1176 +
                               Cn3 * -10654872115970.027 +
                               Cn4 * 595728841989204.2) +
                      pbar1 * (      94224487.05640201 +
                               Cn1 * 72560755004.03894 +
                               Cn2 * -6551926475623.063 +
                               Cn3 * -266093887275558.44 +
                               Cn4 * 2.258956340718414e+16) +
                      pbar2 * (      -4259504612.482598 +
                               Cn1 * -402220727384.6208 +
                               Cn2 * 112417807880283.73 +
                               Cn3 * -309523687023050.1 +
                               Cn4 * -1.7393553985481344e+17) +
                      pbar3 * (      -18164264376.378788 +
                               Cn1 * -9831588410140.684 +
                               Cn2 * 1111239357127492.1 +
                               Cn3 * 3.95773276356943e+16 +
                               Cn4 * -3.7209887687049477e+18) +
                      pbar4 * (      452609427249.7408 +
                               Cn1 * 46080014977241.805 +
                               Cn2 * -1.3062284977868578e+16 +
                               Cn3 * 7.994307982138403e+16 +
                               Cn4 * 1.950118821432187e+19))) +
        CL2 * (      (        (      -25.37586759106799 +
                               Cn1 * -455.9918242866623 +
                               Cn2 * 679984.403202656 +
                               Cn3 * -37509165.46340054 +
                               Cn4 * 377917513.1547775) +
                      pbar1 * (      315.121725197669 +
                               Cn1 * 44461.150806710546 +
                               Cn2 * -16419915.917589974 +
                               Cn3 * 53483677.26957454 +
                               Cn4 * 23333936453.453835) +
                      pbar2 * (      8551.178074227975 +
                               Cn1 * -2576589.511567002 +
                               Cn2 * 50534637.31065825 +
                               Cn3 * 27524196399.564445 +
                               Cn4 * -1796907151777.7678) +
                      pbar3 * (      -25644.904163490337 +
                               Cn1 * 2244774.406064789 +
                               Cn2 * 2104858763.1442537 +
                               Cn3 * -55927370773.7951 +
                               Cn4 * -5425521647790.453) +
                      pbar4 * (      -682819.4875910202 +
                               Cn1 * 328208996.9977792 +
                               Cn2 * -6324514817.520001 +
                               Cn3 * -3020863181610.8936 +
                               Cn4 * 156780931569354.1)) +
               Cm1 * (        (      240.53883287738708 +
                               Cn1 * -189553.02471551386 +
                               Cn2 * -6532879.350003309 +
                               Cn3 * 1832990951.0346835 +
                               Cn4 * -55355273824.10114) +
                      pbar1 * (      -6462.355448216047 +
                               Cn1 * -3559821.589896668 +
                               Cn2 * 436813182.44805855 +
                               Cn3 * -3788063107.3028803 +
                               Cn4 * -1014579267892.4058) +
                      pbar2 * (      -262098.88808879815 +
                               Cn1 * 26744268.339508265 +
                               Cn2 * 3708218755.1953464 +
                               Cn3 * -127585656957.2634 +
                               Cn4 * -10629805210721.707) +
                      pbar3 * (      1184649.740642472 +
                               Cn1 * 502018.4982110137 +
                               Cn2 * -49246700891.51754 +
                               Cn3 * 3329843976502.257 +
                               Cn4 * 48264624094087.92) +
                      pbar4 * (      16660674.737258088 +
                               Cn1 * 336418800.9620934 +
                               Cn2 * -402128353873.40564 +
                               Cn3 * -20152282221583.52 +
                               Cn4 * 3026400760636831.0)) +
               Cm2 * (        (      10273.43754711618 +
                               Cn1 * -446125.9267647057 +
                               Cn2 * -328074789.76441485 +
                               Cn3 * 14522076918.265738 +
                               Cn4 * -12371052793.65224) +
                      pbar1 * (      -170437.98961741762 +
                               Cn1 * 1504162.584054733 +
                               Cn2 * 7157252384.366796 +
                               Cn3 * -177555361380.6028 +
                               Cn4 * -11082732202023.334) +
                      pbar2 * (      -2243179.9303388163 +
                               Cn1 * 1163218464.8657668 +
                               Cn2 * 12206554138.248795 +
                               Cn3 * -9804677786960.393 +
                               Cn4 * 529560122066509.75) +
                      pbar3 * (      3333486.4590424187 +
                               Cn1 * -5458479117.451938 +
                               Cn2 * -701474269463.4797 +
                               Cn3 * 70525283308707.67 +
                               Cn4 * 777896522178522.1) +
                      pbar4 * (      304129289.34253037 +
                               Cn1 * -129409659856.59947 +
                               Cn2 * -1594992910888.9211 +
                               Cn3 * 907595514164074.6 +
                               Cn4 * -3.0833430662647116e+16)) +
               Cm3 * (        (      113517.95060043785 +
                               Cn1 * 57863269.17215351 +
                               Cn2 * -1878595669.512732 +
                               Cn3 * -484494353505.55035 +
                               Cn4 * 22662900060812.777) +
                      pbar1 * (      2369317.7915458377 +
                               Cn1 * 1865553591.209455 +
                               Cn2 * -182945710262.45108 +
                               Cn3 * -2650045586977.008 +
                               Cn4 * 508725031468010.6) +
                      pbar2 * (      -66419969.828824535 +
                               Cn1 * -16194353679.859625 +
                               Cn2 * 2505553742376.44 +
                               Cn3 * 37208636112923.21 +
                               Cn4 * -4851350429977565.0) +
                      pbar3 * (      -447381917.49296683 +
                               Cn1 * -176865888366.67093 +
                               Cn2 * 27798931525009.832 +
                               Cn3 * -1742527113733.7505 +
                               Cn4 * -7.130806117271455e+16) +
                      pbar4 * (      7397601314.834226 +
                               Cn1 * 1321661181286.0713 +
                               Cn2 * -270051890949199.6 +
                               Cn3 * 1921206747400439.5 +
                               Cn4 * 1.7980232296988595e+17)) +
               Cm4 * (        (      -810960.6563552392 +
                               Cn1 * 35770601.23825082 +
                               Cn2 * 28173458922.816673 +
                               Cn3 * -1014044589404.9274 +
                               Cn4 * -5586560679494.493) +
                      pbar1 * (      16025540.37734263 +
                               Cn1 * -2682210579.578737 +
                               Cn2 * -570555669311.5405 +
                               Cn3 * 29064475280786.895 +
                               Cn4 * 729178025017649.9) +
                      pbar2 * (      383122902.36054313 +
                               Cn1 * -74755903539.11008 +
                               Cn2 * -8967713837353.33 +
                               Cn3 * 697710450654120.1 +
                               Cn4 * -1.7328669673738404e+16) +
                      pbar3 * (      -433566510.86340934 +
                               Cn1 * 857718199713.7627 +
                               Cn2 * 46671297986467.5 +
                               Cn3 * -8818954540160694.0 +
                               Cn4 * 2.2434579049017496e+16) +
                      pbar4 * (      -49813419226.27648 +
                               Cn1 * 7430396811955.903 +
                               Cn2 * 1135925511741032.8 +
                               Cn3 * -5.7370613485291336e+16 +
                               Cn4 * -1.0954664717820392e+18)) +
               Cm5 * (        (      -13500330.426896196 +
                               Cn1 * -4202884291.0417147 +
                               Cn2 * 271118300949.19672 +
                               Cn3 * 32066389377417.93 +
                               Cn4 * -1799691778396108.8) +
                      pbar1 * (      -151061020.22533363 +
                               Cn1 * -172900343874.5605 +
                               Cn2 * 14035764579302.732 +
                               Cn3 * 518818516174943.2 +
                               Cn4 * -4.461710247971187e+16) +
                      pbar2 * (      8549695755.054323 +
                               Cn1 * 1501703316758.9038 +
                               Cn2 * -308139497121979.8 +
                               Cn3 * -3088212547301174.5 +
                               Cn4 * 7.185048187598241e+17) +
                      pbar3 * (      31766786860.018864 +
                               Cn1 * 22112772466772.44 +
                               Cn2 * -2362706005566415.5 +
                               Cn3 * -7.782532685312242e+16 +
                               Cn4 * 7.708809795743735e+18) +
                      pbar4 * (      -877945103072.7465 +
                               Cn1 * -155546075016879.28 +
                               Cn2 * 3.3659141441137972e+16 +
                               Cn3 * 1.2839495296621429e+17 +
                               Cn4 * -6.898752664361418e+19))) +
        CL3 * (      (        (      15.053078752497223 +
                               Cn1 * -214.6658178809651 +
                               Cn2 * -388465.3353784562 +
                               Cn3 * 29248414.330871273 +
                               Cn4 * -588077783.8165143) +
                      pbar1 * (      -277.78645417882444 +
                               Cn1 * -46303.89630535017 +
                               Cn2 * 12970972.518202148 +
                               Cn3 * -32616686.75960799 +
                               Cn4 * -20542238650.008842) +
                      pbar2 * (      -6597.147555794238 +
                               Cn1 * 1825656.8669033872 +
                               Cn2 * -69810083.06447008 +
                               Cn3 * -18193097887.195374 +
                               Cn4 * 1260058684766.4434) +
                      pbar3 * (      23466.818516918338 +
                               Cn1 * 233736.37899317904 +
                               Cn2 * -1714276021.334192 +
                               Cn3 * 32507065752.96497 +
                               Cn4 * 4553243813146.544) +
                      pbar4 * (      460670.5617965368 +
                               Cn1 * -231259174.67859295 +
                               Cn2 * 7741537303.834798 +
                               Cn3 * 1988179197841.6492 +
                               Cn4 * -109784214493592.31)) +
               Cm1 * (        (      -187.3888991687689 +
                               Cn1 * 125871.28240639494 +
                               Cn2 * 4269271.002822808 +
                               Cn3 * -1226836577.8892872 +
                               Cn4 * 38690837659.58008) +
                      pbar1 * (      2697.139330102697 +
                               Cn1 * 2837731.19052811 +
                               Cn2 * -278540976.62077135 +
                               Cn3 * 1881156072.829395 +
                               Cn4 * 616311735907.2161) +
                      pbar2 * (      216087.01707185237 +
                               Cn1 * -28557987.928015105 +
                               Cn2 * -1531773842.39123 +
                               Cn3 * 133436860912.88646 +
                               Cn4 * 2607246051923.1543) +
                      pbar3 * (      -572238.2971626659 +
                               Cn1 * -45851320.74824232 +
                               Cn2 * 29899326728.854256 +
                               Cn3 * -2073074390776.1077 +
                               Cn4 * -24885764239164.67) +
                      pbar4 * (      -15795700.118103655 +
                               Cn1 * 886700830.8816222 +
                               Cn2 * 177317943310.9909 +
                               Cn3 * 7899954183728.979 +
                               Cn4 * -1527469300280819.2)) +
               Cm2 * (        (      -5225.121362767436 +
                               Cn1 * 502778.7301902622 +
                               Cn2 * 156937528.23777476 +
                               Cn3 * -10572817130.702887 +
                               Cn4 * 207421159456.0782) +
                      pbar1 * (      130890.48627360228 +
                               Cn1 * 10555223.843451919 +
                               Cn2 * -5952602666.904118 +
                               Cn3 * 62338806817.8327 +
                               Cn4 * 11502544798981.836) +
                      pbar2 * (      459749.32844070345 +
                               Cn1 * -882860106.7967767 +
                               Cn2 * 30437037780.2298 +
                               Cn3 * 6564475173306.265 +
                               Cn4 * -453012495607490.3) +
                      pbar3 * (      -4316302.209825997 +
                               Cn1 * 1698071106.0557399 +
                               Cn2 * 655423310708.9347 +
                               Cn3 * -35431358970214.51 +
                               Cn4 * -1290263707631899.0) +
                      pbar4 * (      -102906525.87460768 +
                               Cn1 * 104503727202.31992 +
                               Cn2 * -2883639012601.416 +
                               Cn3 * -654282601767393.9 +
                               Cn4 * 3.238861938998265e+16)) +
               Cm3 * (        (      -60205.86451020887 +
                               Cn1 * -39911785.46002049 +
                               Cn2 * 1314437588.1133943 +
                               Cn3 * 339217231664.3371 +
                               Cn4 * -16474082904688.635) +
                      pbar1 * (      -1160632.4733913986 +
                               Cn1 * -1307241368.4905372 +
                               Cn2 * 118448113824.14691 +
                               Cn3 * 1550354414263.0356 +
                               Cn4 * -313967979795230.2) +
                      pbar2 * (      29739253.422914207 +
                               Cn1 * 15305058479.623804 +
                               Cn2 * -1998087772157.0645 +
                               Cn3 * -47689975013011.53 +
                               Cn4 * 5080165530651707.0) +
                      pbar3 * (      233560078.08639362 +
                               Cn1 * 120262501459.86447 +
                               Cn2 * -17412087785226.734 +
                               Cn3 * 15290601944011.86 +
                               Cn4 * 4.482942804663111e+16) +
                      pbar4 * (      -3376824473.4893136 +
                               Cn1 * -1334752213453.4739 +
                               Cn2 * 207654160884195.25 +
                               Cn3 * 1257210014685253.5 +
                               Cn4 * -2.945614138719819e+17)) +
               Cm4 * (        (      348210.44423624134 +
                               Cn1 * -42472540.23686054 +
                               Cn2 * -13193964184.647722 +
                               Cn3 * 749703042083.3649 +
                               Cn4 * -13261659687553.863) +
                      pbar1 * (      -11614988.650066203 +
                               Cn1 * 505037559.8095958 +
                               Cn2 * 482055049046.454 +
                               Cn3 * -11447525343071.049 +
                               Cn4 * -926381413377102.0) +
                      pbar2 * (      -129228975.90649678 +
                               Cn1 * 64794840520.74244 +
                               Cn2 * 2030612896066.174 +
                               Cn3 * -503202773804821.9 +
                               Cn4 * 2.3035370658986452e+16) +
                      pbar3 * (      393794189.69923794 +
                               Cn1 * -346356632050.7659 +
                               Cn2 * -47616939606875.414 +
                               Cn3 * 4232975587952591.0 +
                               Cn4 * 6.979754016907138e+16) +
                      pbar4 * (      21684615746.165985 +
                               Cn1 * -7314565608724.073 +
                               Cn2 * -344312903806615.5 +
                               Cn3 * 4.8228711767895944e+16 +
                               Cn4 * -7.494266918632701e+17)) +
               Cm5 * (        (      7846268.935559706 +
                               Cn1 * 2893541753.100636 +
                               Cn2 * -178610366941.61758 +
                               Cn3 * -22512155725711.715 +
                               Cn4 * 1274325404643239.5) +
                      pbar1 * (      70960310.03384168 +
                               Cn1 * 117102234450.86552 +
                               Cn2 * -9007208235825.639 +
                               Cn3 * -315854908854554.25 +
                               Cn4 * 2.7307979456019464e+16) +
                      pbar2 * (      -4605584458.058375 +
                               Cn1 * -1316921539420.7058 +
                               Cn2 * 224201706534885.88 +
                               Cn3 * 3654930951949490.5 +
                               Cn4 * -5.966116911948278e+17) +
                      pbar3 * (      -16396103819.209593 +
                               Cn1 * -14366559208923.643 +
                               Cn2 * 1469456139341287.5 +
                               Cn3 * 4.7961695689818904e+16 +
                               Cn4 * -4.841545713459229e+18) +
                      pbar4 * (      469102343770.4571 +
                               Cn1 * 135218459187080.33 +
                               Cn2 * -2.38418257192483e+16 +
                               Cn3 * -2.6539216132956077e+17 +
                               Cn4 * 5.649603465276686e+19))));
    
    L[1] = (
              (      (        (      -3.9411417101967854 +
                               Cn1 * 291.4279318322384 +
                               Cn2 * -32236.62406208156 +
                               Cn3 * 798611.9470511022 +
                               Cn4 * 29582833.612614177) +
                      pbar1 * (      -14.15714425711438 +
                               Cn1 * -954.7531938406911 +
                               Cn2 * 1491217.764451202 +
                               Cn3 * -43810872.08388787 +
                               Cn4 * -1281179602.8887558) +
                      pbar2 * (      50.84925682860624 +
                               Cn1 * -26373.640114988637 +
                               Cn2 * 16494497.07499973 +
                               Cn3 * -1141096339.171847 +
                               Cn4 * 5006465749.94624) +
                      pbar3 * (      3434.322434331407 +
                               Cn1 * 824910.9057041731 +
                               Cn2 * -155256457.8326156 +
                               Cn3 * 2250578015.7667584 +
                               Cn4 * 137291271049.53427) +
                      pbar4 * (      -14844.65404723074 +
                               Cn1 * -1001041.2410948143 +
                               Cn2 * -906084575.6410592 +
                               Cn3 * 110218244113.2503 +
                               Cn4 * -2096983441140.4395)) +
               Cm1 * (        (      -4.837443474825568 +
                               Cn1 * 9445.923795109138 +
                               Cn2 * -1924129.2678811764 +
                               Cn3 * -36102427.054844186 +
                               Cn4 * 6124104674.194697) +
                      pbar1 * (      471.52141327974516 +
                               Cn1 * 105682.496509096 +
                               Cn2 * -8647835.934404252 +
                               Cn3 * -273171544.28346103 +
                               Cn4 * 3770174919.0707483) +
                      pbar2 * (      -33934.67596613704 +
                               Cn1 * -3701214.97087137 +
                               Cn2 * 1340346603.4858375 +
                               Cn3 * -19241049607.421204 +
                               Cn4 * -2569977581263.7954) +
                      pbar3 * (      -20927.255376210764 +
                               Cn1 * -20423344.17546101 +
                               Cn2 * 1657035071.840116 +
                               Cn3 * 11720743016.297998 +
                               Cn4 * 14518635126.173737) +
                      pbar4 * (      1824364.3646803251 +
                               Cn1 * 372942983.20843065 +
                               Cn2 * -106641900747.72398 +
                               Cn3 * 1668335383073.006 +
                               Cn4 * 215687164350406.9)) +
               Cm2 * (        (      -72.50121314221795 +
                               Cn1 * -60635.70537381245 +
                               Cn2 * 11919370.407674711 +
                               Cn3 * -374441528.84219587 +
                               Cn4 * -5005340044.635735) +
                      pbar1 * (      14725.560793036566 +
                               Cn1 * 8062037.481538727 +
                               Cn2 * -1111591432.1790874 +
                               Cn3 * 5535341026.996783 +
                               Cn4 * 1418615924069.7053) +
                      pbar2 * (      57391.75803882122 +
                               Cn1 * 11547082.445434095 +
                               Cn2 * -6340349737.230498 +
                               Cn3 * 285564472951.23254 +
                               Cn4 * 4577331231485.953) +
                      pbar3 * (      -1429743.8664901187 +
                               Cn1 * -878350662.658852 +
                               Cn2 * 98908679740.57133 +
                               Cn3 * -238060516970.2647 +
                               Cn4 * -64828693771379.734) +
                      pbar4 * (      11197792.831514085 +
                               Cn1 * -1550166439.639592 +
                               Cn2 * 220206158717.9176 +
                               Cn3 * -9769762862811.475 +
                               Cn4 * 24321801918844.6)) +
               Cm3 * (        (      -3117.909738365304 +
                               Cn1 * -4087441.994936041 +
                               Cn2 * 492776397.71946347 +
                               Cn3 * 17061963601.60694 +
                               Cn4 * -1835182134178.5623) +
                      pbar1 * (      -192010.41053668733 +
                               Cn1 * 3552705.467718582 +
                               Cn2 * 328119149.4291658 +
                               Cn3 * 240248214494.91656 +
                               Cn4 * -8729263430573.026) +
                      pbar2 * (      6998220.883670426 +
                               Cn1 * 1995158855.4531965 +
                               Cn2 * -384599416033.15717 +
                               Cn3 * -589957692113.8372 +
                               Cn4 * 1015837447052463.2) +
                      pbar3 * (      18300426.986791145 +
                               Cn1 * 1265396991.2679627 +
                               Cn2 * -164032631474.9713 +
                               Cn3 * -22667636588325.195 +
                               Cn4 * 1076190525679068.0) +
                      pbar4 * (      -404536787.84025085 +
                               Cn1 * -217514249150.9991 +
                               Cn2 * 34717829627721.32 +
                               Cn3 * 219328104510972.78 +
                               Cn4 * -9.910594357420242e+16)) +
               Cm4 * (        (      -6237.900692114016 +
                               Cn1 * -746281.0461958826 +
                               Cn2 * 233085491.61043227 +
                               Cn3 * 14786506155.57713 +
                               Cn4 * -1484007160344.9695) +
                      pbar1 * (      -780367.3447573964 +
                               Cn1 * -784402748.6370173 +
                               Cn2 * 92771511022.46213 +
                               Cn3 * -332879135483.18524 +
                               Cn4 * -106507541236939.81) +
                      pbar2 * (      282348.71353279985 +
                               Cn1 * -448934976.11569864 +
                               Cn2 * 207337869285.42627 +
                               Cn3 * -4157511114382.206 +
                               Cn4 * -575152957370306.8) +
                      pbar3 * (      84485817.01202196 +
                               Cn1 * 83453935561.98273 +
                               Cn2 * -8381516860080.297 +
                               Cn3 * 35785511391048.96 +
                               Cn4 * 1447177373984252.5) +
                      pbar4 * (      -1478961942.5582325 +
                               Cn1 * 222499286317.86627 +
                               Cn2 * 2717073420818.6743 +
                               Cn3 * -1731596102800236.5 +
                               Cn4 * 4.4291407087251544e+16)) +
               Cm5 * (        (      -101923.30157522885 +
                               Cn1 * 253699676.4284533 +
                               Cn2 * -21261443296.48418 +
                               Cn3 * -1330780993108.8354 +
                               Cn4 * 103353956913317.06) +
                      pbar1 * (      13145021.687301263 +
                               Cn1 * -2099650583.218345 +
                               Cn2 * 198397279513.71622 +
                               Cn3 * -24494277995322.77 +
                               Cn4 * 740317951990661.9) +
                      pbar2 * (      -362849912.92364043 +
                               Cn1 * -165727762508.71408 +
                               Cn2 * 24218036531874.62 +
                               Cn3 * 317503256781464.94 +
                               Cn4 * -7.515784895550506e+16) +
                      pbar3 * (      -1502239579.1712036 +
                               Cn1 * 240715207867.0595 +
                               Cn2 * -20537879512452.145 +
                               Cn3 * 2394855658555685.0 +
                               Cn4 * -1.0458850061666995e+17) +
                      pbar4 * (      23441240184.484833 +
                               Cn1 * 19204549675339.98 +
                               Cn2 * -2421528890834756.5 +
                               Cn3 * -4.8450912326380744e+16 +
                               Cn4 * 7.893898907157295e+18))) +
        CL1 * (      (        (      0.9416463959831871 +
                               Cn1 * -379.65390223951255 +
                               Cn2 * 82455.88407430534 +
                               Cn3 * -9356975.24613719 +
                               Cn4 * 257057157.74437055) +
                      pbar1 * (      301.4189074671276 +
                               Cn1 * 100389.47433689717 +
                               Cn2 * -15243750.2927931 +
                               Cn3 * 139994892.95139375 +
                               Cn4 * 23041280960.15678) +
                      pbar2 * (      -2557.2646082519354 +
                               Cn1 * -863629.1743358878 +
                               Cn2 * 86608105.23276526 +
                               Cn3 * 7602667643.545279 +
                               Cn4 * -365070087771.37445) +
                      pbar3 * (      -18968.03546598088 +
                               Cn1 * -13489205.993026549 +
                               Cn2 * 1461453267.565993 +
                               Cn3 * 7843162539.942535 +
                               Cn4 * -2593472435388.622) +
                      pbar4 * (      352934.4523198648 +
                               Cn1 * 117592125.24869192 +
                               Cn2 * -12917675304.635706 +
                               Cn3 * -914487898065.6989 +
                               Cn4 * 53290823585431.49)) +
               Cm1 * (        (      8.97670981519846 +
                               Cn1 * -121103.07982168985 +
                               Cn2 * 10221085.239530355 +
                               Cn3 * 829511452.755613 +
                               Cn4 * -54296156344.8334) +
                      pbar1 * (      914.1106396092214 +
                               Cn1 * 2294473.523165574 +
                               Cn2 * -336762905.9629761 +
                               Cn3 * 5286199272.636334 +
                               Cn4 * 592976196269.5712) +
                      pbar2 * (      183662.06665710197 +
                               Cn1 * 4957017.268792275 +
                               Cn2 * -4881662644.551265 +
                               Cn3 * 61388386571.038414 +
                               Cn4 * 11949882118665.268) +
                      pbar3 * (      -656003.125255129 +
                               Cn1 * -126728452.40806346 +
                               Cn2 * 31807449975.54221 +
                               Cn3 * -432260084174.3351 +
                               Cn4 * -81755195850785.19) +
                      pbar4 * (      -4619679.6340689445 +
                               Cn1 * -456595883.0161972 +
                               Cn2 * 320446381585.9685 +
                               Cn3 * -9541836741210.998 +
                               Cn4 * -833157114098685.9)) +
               Cm2 * (        (      -1765.5672136622748 +
                               Cn1 * -755969.0444690452 +
                               Cn2 * 124084945.31463742 +
                               Cn3 * 4406863773.960985 +
                               Cn4 * -438756470391.3937) +
                      pbar1 * (      -78338.81851918588 +
                               Cn1 * -84493607.86914289 +
                               Cn2 * 9495741463.135267 +
                               Cn3 * 30327410334.53205 +
                               Cn4 * -15930534671725.611) +
                      pbar2 * (      1674608.45479565 +
                               Cn1 * 861761926.7292985 +
                               Cn2 * -113354033365.41115 +
                               Cn3 * -2537118048235.1675 +
                               Cn4 * 251549289483612.12) +
                      pbar3 * (      11137125.704095913 +
                               Cn1 * 8627454722.19065 +
                               Cn2 * -830031966035.5044 +
                               Cn3 * -5350172987760.915 +
                               Cn4 * 880144619579726.0) +
                      pbar4 * (      -298722991.98893213 +
                               Cn1 * -75256257375.87474 +
                               Cn2 * 12983042448347.244 +
                               Cn3 * 153102012739052.66 +
                               Cn4 * -2.918756478069478e+16)) +
               Cm3 * (        (      -68416.91896596363 +
                               Cn1 * 35418613.05141002 +
                               Cn2 * -1614126531.1732328 +
                               Cn3 * -256211957506.68243 +
                               Cn4 * 13416328879654.389) +
                      pbar1 * (      649326.2673523474 +
                               Cn1 * -846563605.329621 +
                               Cn2 * 101277600853.5442 +
                               Cn3 * -2447521584766.387 +
                               Cn4 * -121997050174959.22) +
                      pbar2 * (      -22308245.600961573 +
                               Cn1 * -6800418898.1230545 +
                               Cn2 * 1240989943172.5469 +
                               Cn3 * 13405752352960.764 +
                               Cn4 * -4709593864541691.0) +
                      pbar3 * (      13783511.17337275 +
                               Cn1 * 89323892570.98613 +
                               Cn2 * -12480862437993.455 +
                               Cn3 * 206094452687853.2 +
                               Cn4 * 2.015376993706204e+16) +
                      pbar4 * (      146238413.3985247 +
                               Cn1 * 639209843286.0109 +
                               Cn2 * -90786627023774.28 +
                               Cn3 * -909587920918098.8 +
                               Cn4 * 4.017472253797036e+17)) +
               Cm4 * (        (      293592.65924964566 +
                               Cn1 * 115413521.09821871 +
                               Cn2 * -20193284422.388634 +
                               Cn3 * -284933815397.218 +
                               Cn4 * 51526349619605.195) +
                      pbar1 * (      3563115.915540931 +
                               Cn1 * 8264324749.36864 +
                               Cn2 * -847659306807.7004 +
                               Cn3 * -2461588239550.122 +
                               Cn4 * 1292518535146740.8) +
                      pbar2 * (      -168304399.06886625 +
                               Cn1 * -84406202221.55603 +
                               Cn2 * 12161469723843.877 +
                               Cn3 * 106932591264716.66 +
                               Cn4 * -2.085316868542661e+16) +
                      pbar3 * (      -691703525.0861607 +
                               Cn1 * -806984830124.3162 +
                               Cn2 * 72288110586637.11 +
                               Cn3 * 108801890763580.16 +
                               Cn4 * -3.1962294849760572e+16) +
                      pbar4 * (      28371011707.72654 +
                               Cn1 * 6100331484493.617 +
                               Cn2 * -1274182113784089.2 +
                               Cn3 * 4444800437220833.0 +
                               Cn4 * 2.221045843525471e+18)) +
               Cm5 * (        (      7923155.107396528 +
                               Cn1 * -2004987604.2985482 +
                               Cn2 * -14387688536.502277 +
                               Cn3 * 18236647950726.207 +
                               Cn4 * -684531299537628.2) +
                      pbar1 * (      -73188502.6776237 +
                               Cn1 * 70520733211.78673 +
                               Cn2 * -7994799570885.058 +
                               Cn3 * 209722370419507.34 +
                               Cn4 * 6979460601655548.0) +
                      pbar2 * (      569383845.1143789 +
                               Cn1 * 561918516592.963 +
                               Cn2 * -54466170069069.08 +
                               Cn3 * -2507885529543056.5 +
                               Cn4 * 3.2478778585511936e+17) +
                      pbar3 * (      4942698295.197308 +
                               Cn1 * -9302503947769.107 +
                               Cn2 * 1093188757410104.0 +
                               Cn3 * -1.6270049978652546e+16 +
                               Cn4 * -1.2205099343139205e+18) +
                      pbar4 * (      8217337316.208348 +
                               Cn1 * -57961169555022.89 +
                               Cn2 * 4657312958660499.0 +
                               Cn3 * 2.461145997260812e+17 +
                               Cn4 * -2.9547007005313114e+19))) +
        CL2 * (      (        (      5.695701843065234 +
                               Cn1 * 982.6894677498165 +
                               Cn2 * -366060.2720483255 +
                               Cn3 * 26894438.4630281 +
                               Cn4 * -520846504.4160072) +
                      pbar1 * (      -506.2250424752391 +
                               Cn1 * -184871.67016102013 +
                               Cn2 * 28180854.406167302 +
                               Cn3 * -535241283.4636364 +
                               Cn4 * -30478165518.911575) +
                      pbar2 * (      10688.334923004606 +
                               Cn1 * 28883.67354707245 +
                               Cn2 * -91172407.16118707 +
                               Cn3 * -7605651825.922872 +
                               Cn4 * 289428525205.3085) +
                      pbar3 * (      10373.90339986598 +
                               Cn1 * 32901120.610372916 +
                               Cn2 * -2985839977.789789 +
                               Cn3 * -12410740631.442244 +
                               Cn4 * 4722154427621.045) +
                      pbar4 * (      -975730.3287873111 +
                               Cn1 * -136338588.10598907 +
                               Cn2 * 22207766274.723278 +
                               Cn3 * 1237519594639.047 +
                               Cn4 * -74355181629197.39)) +
               Cm1 * (        (      -64.35749152683027 +
                               Cn1 * 226242.04915195107 +
                               Cn2 * -13318195.700561108 +
                               Cn3 * -2009219803.8751984 +
                               Cn4 * 105475616604.95224) +
                      pbar1 * (      -2621.3298440830317 +
                               Cn1 * -9635422.931557674 +
                               Cn2 * 1114454828.0879705 +
                               Cn3 * -2449012393.4852257 +
                               Cn4 * -2263779274773.113) +
                      pbar2 * (      -469770.64335036295 +
                               Cn1 * 71398898.13992985 +
                               Cn2 * 1316412490.6259632 +
                               Cn3 * -238962880015.2226 +
                               Cn4 * -4332609767014.069) +
                      pbar3 * (      2058793.0942569713 +
                               Cn1 * 574047142.8506216 +
                               Cn2 * -101856327531.1511 +
                               Cn3 * 227679965932.12848 +
                               Cn4 * 273999663436519.4) +
                      pbar4 * (      13535013.83145229 +
                               Cn1 * -5968658603.202006 +
                               Cn2 * 80622609717.76447 +
                               Cn3 * 35276465043802.32 +
                               Cn4 * -235194268386588.56)) +
               Cm2 * (        (      3235.4657047988057 +
                               Cn1 * 2538131.130652977 +
                               Cn2 * -348421024.00644535 +
                               Cn3 * -12653906468.982224 +
                               Cn4 * 1180540738041.5955) +
                      pbar1 * (      98780.33150252835 +
                               Cn1 * 172671181.0920136 +
                               Cn2 * -17884430600.969692 +
                               Cn3 * -84693907236.51636 +
                               Cn4 * 31056395840988.477) +
                      pbar2 * (      -3503974.1785390107 +
                               Cn1 * -2151960004.7444286 +
                               Cn2 * 284449647057.64 +
                               Cn3 * 4402637062714.384 +
                               Cn4 * -512869792527697.06) +
                      pbar3 * (      -21766202.218101203 +
                               Cn1 * -16954070881.113192 +
                               Cn2 * 1492087548603.248 +
                               Cn3 * 9424911136653.453 +
                               Cn4 * -1173323374597663.0) +
                      pbar4 * (      676539450.7727659 +
                               Cn1 * 176405195468.76215 +
                               Cn2 * -31325752378756.75 +
                               Cn3 * -171755344063918.22 +
                               Cn4 * 5.9124591221803144e+16)) +
               Cm3 * (        (      219264.06433546764 +
                               Cn1 * -61348243.11325457 +
                               Cn2 * -828752512.786474 +
                               Cn3 * 655223408767.9736 +
                               Cn4 * -23406585209287.418) +
                      pbar1 * (      -1485425.507508174 +
                               Cn1 * 3191610822.666947 +
                               Cn2 * -347318059548.64276 +
                               Cn3 * 2698974076236.056 +
                               Cn4 * 620611955731400.2) +
                      pbar2 * (      60454922.23035323 +
                               Cn1 * -13948548173.11165 +
                               Cn2 * 649070763097.5637 +
                               Cn3 * 2678086541573.055 +
                               Cn4 * 3052841466011452.0) +
                      pbar3 * (      -140062880.83893645 +
                               Cn1 * -299911690836.69977 +
                               Cn2 * 39586515143608.73 +
                               Cn3 * -200346486635985.72 +
                               Cn4 * -8.1243516038025e+16) +
                      pbar4 * (      -1853953265.3687794 +
                               Cn1 * 1249850056337.204 +
                               Cn2 * -86341741621566.69 +
                               Cn3 * -2897478833242385.5 +
                               Cn4 * -1.7778961289217955e+17)) +
               Cm4 * (        (      -551061.5080436614 +
                               Cn1 * -377290612.14012617 +
                               Cn2 * 56186956653.83293 +
                               Cn3 * 939592037379.2739 +
                               Cn4 * -142208273178803.84) +
                      pbar1 * (      -1815925.5970631447 +
                               Cn1 * -16994581780.338783 +
                               Cn2 * 1581143820209.714 +
                               Cn3 * 9808414879406.36 +
                               Cn4 * -2616164425402254.5) +
                      pbar2 * (      304381410.91024315 +
                               Cn1 * 236271767497.56985 +
                               Cn2 * -32078175841065.242 +
                               Cn3 * -249159111387979.06 +
                               Cn4 * 5.095310067592498e+16) +
                      pbar3 * (      1457174701.8674226 +
                               Cn1 * 1478028827228.105 +
                               Cn2 * -120901226944425.5 +
                               Cn3 * 61695742812805.05 +
                               Cn4 * 2143946670095679.0) +
                      pbar4 * (      -61337750730.570435 +
                               Cn1 * -16287027818614.646 +
                               Cn2 * 3187943713932551.0 +
                               Cn3 * -1.2869774624670152e+16 +
                               Cn4 * -5.083003265852602e+18)) +
               Cm5 * (        (      -22816425.350724276 +
                               Cn1 * 3038793350.1176333 +
                               Cn2 * 363086881326.62787 +
                               Cn3 * -46990245791528.43 +
                               Cn4 * 1082837846282826.1) +
                      pbar1 * (      166810530.7364703 +
                               Cn1 * -242153303511.7615 +
                               Cn2 * 25939318896742.13 +
                               Cn3 * -268158391398171.78 +
                               Cn4 * -4.012132022340798e+16) +
                      pbar2 * (      -1880832578.7592933 +
                               Cn1 * 934564840418.6737 +
                               Cn2 * -131660890916557.77 +
                               Cn3 * 2887264624684999.5 +
                               Cn4 * -1.64891587384314e+17) +
                      pbar3 * (      -5218881190.095439 +
                               Cn1 * 28107153495120.61 +
                               Cn2 * -3288674394921000.5 +
                               Cn3 * 1.3902852184483372e+16 +
                               Cn4 * 5.555633457195438e+18) +
                      pbar4 * (      99513616528.95305 +
                               Cn1 * -84126467411069.94 +
                               Cn2 * 1.191941246017288e+16 +
                               Cn3 * -1.1371282960301475e+17 +
                               Cn4 * 8.890118294545071e+18))) +
        CL3 * (      (        (      -6.122438456127769 +
                               Cn1 * -1257.8386723653016 +
                               Cn2 * 378749.36712515034 +
                               Cn3 * -20771307.888287693 +
                               Cn4 * 199676488.2504234) +
                      pbar1 * (      230.3960153602719 +
                               Cn1 * 86476.165015649 +
                               Cn2 * -14247378.662241178 +
                               Cn3 * 436327985.8219193 +
                               Cn4 * 7658874849.524599) +
                      pbar2 * (      -8510.050326363287 +
                               Cn1 * 1102142.2836520239 +
                               Cn2 * -40854912.22999694 +
                               Cn3 * 1232189958.4735975 +
                               Cn4 * 119563610676.85152) +
                      pbar3 * (      10554.498577399698 +
                               Cn1 * -22094513.42580258 +
                               Cn2 * 1750093038.462203 +
                               Cn3 * 5996948490.198706 +
                               Cn4 * -2384607986183.346) +
                      pbar4 * (      634006.0712015519 +
                               Cn1 * 12251295.80848123 +
                               Cn2 * -6936596695.384979 +
                               Cn3 * -461308696723.2475 +
                               Cn4 * 21085691978802.08)) +
               Cm1 * (        (      23.34786013492117 +
                               Cn1 * -109532.27026021863 +
                               Cn2 * 4213906.488933887 +
                               Cn3 * 1239801258.1260722 +
                               Cn4 * -56606026291.323746) +
                      pbar1 * (      473.1654750840187 +
                               Cn1 * 7833163.71236476 +
                               Cn2 * -811432303.2634681 +
                               Cn3 * -4399538159.108355 +
                               Cn4 * 1833054159454.8105) +
                      pbar2 * (      357288.13403232174 +
                               Cn1 * -88201542.88599016 +
                               Cn2 * 3164227731.568565 +
                               Cn3 * 247242031405.97623 +
                               Cn4 * -8103989887014.895) +
                      pbar3 * (      -1519054.2773546302 +
                               Cn1 * -440342832.4196109 +
                               Cn2 * 71617129324.26842 +
                               Cn3 * 287715824466.77576 +
                               Cn4 * -208331174329328.6) +
                      pbar4 * (      -12402553.780025903 +
                               Cn1 * 7282005396.978538 +
                               Cn2 * -374804195958.84906 +
                               Cn3 * -32443864502511.82 +
                               Cn4 * 1130211917926096.5)) +
               Cm2 * (        (      -1237.7573694210967 +
                               Cn1 * -1852119.7924917394 +
                               Cn2 * 216380391.85056394 +
                               Cn3 * 9434133719.63728 +
                               Cn4 * -764070309838.2368) +
                      pbar1 * (      -35921.52398860245 +
                               Cn1 * -97268412.55170593 +
                               Cn2 * 9574233310.509607 +
                               Cn3 * 46534249046.88459 +
                               Cn4 * -16504512733051.994) +
                      pbar2 * (      1975612.9128482703 +
                               Cn1 * 1318223959.7308037 +
                               Cn2 * -170765391539.7221 +
                               Cn3 * -2218636070890.253 +
                               Cn4 * 263089270315583.88) +
                      pbar3 * (      11666360.575956969 +
                               Cn1 * 9428844125.473272 +
                               Cn2 * -765989458161.3859 +
                               Cn3 * -2805153203447.9165 +
                               Cn4 * 271460895351968.78) +
                      pbar4 * (      -410654118.9352888 +
                               Cn1 * -103357681499.18579 +
                               Cn2 * 18998350742431.023 +
                               Cn3 * 19433762841273.258 +
                               Cn4 * -3.0802361357165268e+16)) +
               Cm3 * (        (      -148771.69568542182 +
                               Cn1 * 25964087.5189768 +
                               Cn2 * 2519752117.19142 +
                               Cn3 * -421783815121.28424 +
                               Cn4 * 11025915101852.72) +
                      pbar1 * (      1195711.5946736906 +
                               Cn1 * -2452189211.328266 +
                               Cn2 * 252140217257.46375 +
                               Cn3 * -187505319073.49692 +
                               Cn4 * -519550260919870.06) +
                      pbar2 * (      -54407789.75122724 +
                               Cn1 * 22271278739.94468 +
                               Cn2 * -1725683612207.3835 +
                               Cn3 * -25538595914583.586 +
                               Cn4 * 1247545588766928.5) +
                      pbar3 * (      104943474.21993932 +
                               Cn1 * 214250606562.04358 +
                               Cn2 * -27368404699784.5 +
                               Cn3 * -3003533873852.3677 +
                               Cn4 * 6.2645757499216136e+16) +
                      pbar4 * (      2843788094.2418895 +
                               Cn1 * -1957312452621.5593 +
                               Cn2 * 158806025463507.7 +
                               Cn3 * 4625212720867109.0 +
                               Cn4 * -1.767111913376556e+17)) +
               Cm4 * (        (      235223.84649725075 +
                               Cn1 * 277280804.6335853 +
                               Cn2 * -36916202977.6682 +
                               Cn3 * -736419136349.88 +
                               Cn4 * 95102235602489.42) +
                      pbar1 * (      -1084710.0791186993 +
                               Cn1 * 9780935751.154366 +
                               Cn2 * -841189467702.1741 +
                               Cn3 * -7199219320016.893 +
                               Cn4 * 1446107115300842.5) +
                      pbar2 * (      -140182816.4373996 +
                               Cn1 * -158837069114.693 +
                               Cn2 * 20589840380211.523 +
                               Cn3 * 154085470072379.9 +
                               Cn4 * -3.0795836278041264e+16) +
                      pbar3 * (      -835230294.4933206 +
                               Cn1 * -772627991293.8226 +
                               Cn2 * 57069361467402.07 +
                               Cn3 * -323829721565384.5 +
                               Cn4 * 3.7742352394121416e+16) +
                      pbar4 * (      35547217141.03436 +
                               Cn1 * 10486368172481.512 +
                               Cn2 * -2017715567105651.2 +
                               Cn3 * 1.1238336585889122e+16 +
                               Cn4 * 2.9251069071691996e+18)) +
               Cm5 * (        (      15376423.75883068 +
                               Cn1 * -947798729.4898508 +
                               Cn2 * -378872007822.62537 +
                               Cn3 * 30502860108566.848 +
                               Cn4 * -426793638707806.0) +
                      pbar1 * (      -119185900.4528573 +
                               Cn1 * 177632275266.8068 +
                               Cn2 * -18295243968746.688 +
                               Cn3 * 72644420835138.33 +
                               Cn4 * 3.3458248729297764e+16) +
                      pbar2 * (      2152586864.638454 +
                               Cn1 * -1527910307342.6016 +
                               Cn2 * 176209302807662.03 +
                               Cn3 * -174555787787571.62 +
                               Cn4 * -1.2009644581721106e+17) +
                      pbar3 * (      3404097004.700623 +
                               Cn1 * -19346123896291.453 +
                               Cn2 * 2223046600104773.8 +
                               Cn3 * 996835102901453.0 +
                               Cn4 * -4.3201059583465743e+18) +
                      pbar4 * (      -180337293833.44202 +
                               Cn1 * 139206263633456.45 +
                               Cn2 * -1.5109938637872658e+16 +
                               Cn3 * -1.4215390474696264e+17 +
                               Cn4 * 1.5779585149148539e+19))));
    
    L[2] = (
              (      (        (      -5.579497116348753 +
                               Cn1 * 578.1182380356792 +
                               Cn2 * -43836.251105145064 +
                               Cn3 * -692508.8297514837 +
                               Cn4 * 103897524.09548585) +
                      pbar1 * (      28.387150154695576 +
                               Cn1 * 27676.22592910956 +
                               Cn2 * -2626894.513762008 +
                               Cn3 * -35104245.29389764 +
                               Cn4 * 5675807619.905014) +
                      pbar2 * (      525.6815209199119 +
                               Cn1 * -93825.17136568256 +
                               Cn2 * 8885854.024057081 +
                               Cn3 * 215906080.67475462 +
                               Cn4 * -37264998242.28236) +
                      pbar3 * (      1485.8070545931043 +
                               Cn1 * -2038970.748631297 +
                               Cn2 * 239344021.43552825 +
                               Cn3 * 1378148437.6635277 +
                               Cn4 * -568218232143.0272) +
                      pbar4 * (      -55534.90611537862 +
                               Cn1 * 10805682.959746419 +
                               Cn2 * -757083826.4558417 +
                               Cn3 * -35911605892.0601 +
                               Cn4 * 3208767153556.7783)) +
               Cm1 * (        (      -22.367609340184742 +
                               Cn1 * -25980.681538849232 +
                               Cn2 * 1793704.493576762 +
                               Cn3 * 54383246.078727864 +
                               Cn4 * -4085681805.8266115) +
                      pbar1 * (      -420.0934447883645 +
                               Cn1 * 444515.5294510512 +
                               Cn2 * -32253625.167386007 +
                               Cn3 * -1173954259.2314208 +
                               Cn4 * 92675999576.99626) +
                      pbar2 * (      -11734.735420082718 +
                               Cn1 * 15025135.658089519 +
                               Cn2 * -1375751174.424842 +
                               Cn3 * -10819270479.098476 +
                               Cn4 * 2209839001033.534) +
                      pbar3 * (      -17990.23379748129 +
                               Cn1 * -47427346.56175779 +
                               Cn2 * 3895353829.1991677 +
                               Cn3 * 145105193146.0369 +
                               Cn4 * -11965062707984.914) +
                      pbar4 * (      1221946.0555379086 +
                               Cn1 * -1129018057.9963999 +
                               Cn2 * 96459877178.08073 +
                               Cn3 * 1015760682138.9484 +
                               Cn4 * -151916529220191.7)) +
               Cm2 * (        (      464.3995375680357 +
                               Cn1 * -125402.230635663 +
                               Cn2 * 10180495.462025 +
                               Cn3 * 70785297.68572727 +
                               Cn4 * -25297857885.992786) +
                      pbar1 * (      20870.36746472311 +
                               Cn1 * -17772883.256220743 +
                               Cn2 * 1809263052.7786214 +
                               Cn3 * 15386220505.651302 +
                               Cn4 * -3842352372706.115) +
                      pbar2 * (      -709971.5940691626 +
                               Cn1 * 132168048.91686918 +
                               Cn2 * -5858756280.869223 +
                               Cn3 * -175282358301.79468 +
                               Cn4 * 13200546503244.65) +
                      pbar3 * (      -669822.5067680326 +
                               Cn1 * 1098445216.8131003 +
                               Cn2 * -160800373888.98325 +
                               Cn3 * 871861539147.3628 +
                               Cn4 * 346870324336243.2) +
                      pbar4 * (      65148898.25087327 +
                               Cn1 * -13105907111.895308 +
                               Cn2 * 539970282451.686 +
                               Cn3 * 14411670290145.447 +
                               Cn4 * -435939366794777.56)) +
               Cm3 * (        (      -13112.389941786245 +
                               Cn1 * 6615550.239953292 +
                               Cn2 * -324714800.7305547 +
                               Cn3 * -22660991760.655598 +
                               Cn4 * 1078798219108.2876) +
                      pbar1 * (      -224910.75881673655 +
                               Cn1 * -126896865.81967665 +
                               Cn2 * 19176986439.3948 +
                               Cn3 * 180372444706.47177 +
                               Cn4 * -48309836894050.04) +
                      pbar2 * (      10908467.718462093 +
                               Cn1 * -5703097476.040764 +
                               Cn2 * 441494077322.9934 +
                               Cn3 * 9342400903243.38 +
                               Cn4 * -924929742445757.0) +
                      pbar3 * (      18433457.417527426 +
                               Cn1 * 19066377581.566944 +
                               Cn2 * -2507109172244.3477 +
                               Cn3 * -26032413584992.8 +
                               Cn4 * 6376118442011448.0) +
                      pbar4 * (      -757607541.2964827 +
                               Cn1 * 432575619338.14966 +
                               Cn2 * -34987721403819.03 +
                               Cn3 * -815254982031049.6 +
                               Cn4 * 8.196340850351206e+16)) +
               Cm4 * (        (      -27943.114955433844 +
                               Cn1 * 19157295.833875902 +
                               Cn2 * -1939268761.9708652 +
                               Cn3 * -11059127690.841242 +
                               Cn4 * 4534626188472.924) +
                      pbar1 * (      -2151097.6660964293 +
                               Cn1 * 1515913963.9078717 +
                               Cn2 * -157104554758.8144 +
                               Cn3 * -1018292735741.2999 +
                               Cn4 * 330120292292182.4) +
                      pbar2 * (      64266457.89287383 +
                               Cn1 * -14220864523.33334 +
                               Cn2 * 810885595967.7517 +
                               Cn3 * 6651384501966.166 +
                               Cn4 * -1191645525543301.5) +
                      pbar3 * (      78112831.28862464 +
                               Cn1 * -95482550684.13849 +
                               Cn2 * 14789262782747.904 +
                               Cn3 * -148750686095332.9 +
                               Cn4 * -3.087666183732244e+16) +
                      pbar4 * (      -5911143130.245999 +
                               Cn1 * 1314793093754.5073 +
                               Cn2 * -64504297911824.56 +
                               Cn3 * -105947002831324.05 +
                               Cn4 * -1.3957809266405146e+16)) +
               Cm5 * (        (      1054302.9868410144 +
                               Cn1 * -365915738.9580917 +
                               Cn2 * 9519588673.188513 +
                               Cn3 * 1692130281867.271 +
                               Cn4 * -52771864765435.49) +
                      pbar1 * (      24272794.394261807 +
                               Cn1 * 9982206576.030071 +
                               Cn2 * -1765919046773.8616 +
                               Cn3 * -4938582314927.259 +
                               Cn4 * 4141830940213839.5) +
                      pbar2 * (      -991588927.2890124 +
                               Cn1 * 417970055859.15094 +
                               Cn2 * -29796549232838.742 +
                               Cn3 * -882007828798452.1 +
                               Cn4 * 7.2007368232857544e+16) +
                      pbar3 * (      -1486876046.1060936 +
                               Cn1 * -1572332954466.3757 +
                               Cn2 * 236701768001414.84 +
                               Cn3 * 475529584152621.7 +
                               Cn4 * -5.6145048119085306e+17) +
                      pbar4 * (      64313774770.024796 +
                               Cn1 * -32556346782766.23 +
                               Cn2 * 2613709586669432.5 +
                               Cn3 * 7.798639036586286e+16 +
                               Cn4 * -7.42308538999107e+18))) +
        CL1 * (      (        (      -2.1746503005041045 +
                               Cn1 * -2810.909476589457 +
                               Cn2 * 309785.1554599177 +
                               Cn3 * 839175.4385746486 +
                               Cn4 * -648809773.1476518) +
                      pbar1 * (      60.73645637509382 +
                               Cn1 * -126034.5715255349 +
                               Cn2 * 15051440.430924179 +
                               Cn3 * 237710414.5699646 +
                               Cn4 * -36577920308.894066) +
                      pbar2 * (      891.3894129086625 +
                               Cn1 * 1701405.0124759784 +
                               Cn2 * -231217524.9989118 +
                               Cn3 * -518663086.6552621 +
                               Cn4 * 586064833793.2286) +
                      pbar3 * (      -9846.230931518034 +
                               Cn1 * 7135793.586987463 +
                               Cn2 * -1278906106.5139363 +
                               Cn3 * -7149196391.893416 +
                               Cn4 * 3768683583852.643) +
                      pbar4 * (      153354.71667872643 +
                               Cn1 * -168745496.17754078 +
                               Cn2 * 17293737572.034573 +
                               Cn3 * 307023287668.13745 +
                               Cn4 * -49622160866254.08)) +
               Cm1 * (        (      -170.9134889934009 +
                               Cn1 * 98333.45820707167 +
                               Cn2 * -2637779.4094183706 +
                               Cn3 * -499028922.576078 +
                               Cn4 * 21109717763.227444) +
                      pbar1 * (      -151.64610698472862 +
                               Cn1 * -8382142.643945458 +
                               Cn2 * 914939983.8102465 +
                               Cn3 * 15986111891.720604 +
                               Cn4 * -2199075693828.9263) +
                      pbar2 * (      -84708.44972539399 +
                               Cn1 * -24970995.449974142 +
                               Cn2 * 4730807277.414989 +
                               Cn3 * -70205774615.88937 +
                               Cn4 * -5948348262440.213) +
                      pbar3 * (      769896.8565065069 +
                               Cn1 * 884400309.3951955 +
                               Cn2 * -100151363063.34694 +
                               Cn3 * -2074803587272.5518 +
                               Cn4 * 259818178947039.12) +
                      pbar4 * (      4488631.154955441 +
                               Cn1 * 453243635.3263081 +
                               Cn2 * -201153608638.46344 +
                               Cn3 * 9395068843673.145 +
                               Cn4 * -12327281297612.375)) +
               Cm2 * (        (      -1536.3830141623841 +
                               Cn1 * 2585967.577623847 +
                               Cn2 * -237841447.57409254 +
                               Cn3 * -3957524087.884639 +
                               Cn4 * 630819402468.6118) +
                      pbar1 * (      -158828.9753467852 +
                               Cn1 * 100306730.19296685 +
                               Cn2 * -10554430738.701399 +
                               Cn3 * -112587356114.38638 +
                               Cn4 * 24034971085805.33) +
                      pbar2 * (      5417931.9580748165 +
                               Cn1 * -2321672489.894835 +
                               Cn2 * 179464624923.1263 +
                               Cn3 * 2606049779296.3203 +
                               Cn4 * -410450876949624.44) +
                      pbar3 * (      -3926327.7352724867 +
                               Cn1 * -2235628478.473486 +
                               Cn2 * 861061678750.5344 +
                               Cn3 * -15762664269748.732 +
                               Cn4 * -2015901949918354.5) +
                      pbar4 * (      -485736017.76086134 +
                               Cn1 * 197057850392.03616 +
                               Cn2 * -13823354661786.57 +
                               Cn3 * -236615044363172.94 +
                               Cn4 * 2.65033993755968e+16)) +
               Cm3 * (        (      136200.90670542984 +
                               Cn1 * -31463041.648182098 +
                               Cn2 * -458516080.9052383 +
                               Cn3 * 198783605119.88748 +
                               Cn4 * -4515091344283.96) +
                      pbar1 * (      3010517.998501118 +
                               Cn1 * 2329514241.2097335 +
                               Cn2 * -332224836256.21136 +
                               Cn3 * -3343008151141.1704 +
                               Cn4 * 777125258447981.9) +
                      pbar2 * (      -58209889.82433158 +
                               Cn1 * 18485396005.783276 +
                               Cn2 * -1367693732449.1707 +
                               Cn3 * -25254757348693.984 +
                               Cn4 * 2587231779960303.5) +
                      pbar3 * (      -258098567.90774024 +
                               Cn1 * -329962264641.74084 +
                               Cn2 * 42339525140600.83 +
                               Cn3 * 544917423537816.3 +
                               Cn4 * -1.049100231047875e+17) +
                      pbar4 * (      2922842359.4177017 +
                               Cn1 * -589434791070.9144 +
                               Cn2 * 66171106053183.945 +
                               Cn3 * 286716142154261.0 +
                               Cn4 * -1.4226887509535366e+17)) +
               Cm4 * (        (      214133.6949525618 +
                               Cn1 * -315774087.22328126 +
                               Cn2 * 29896425056.197136 +
                               Cn3 * 435019174044.96954 +
                               Cn4 * -76098139445572.39) +
                      pbar1 * (      17784349.24701686 +
                               Cn1 * -9484405788.822569 +
                               Cn2 * 1008149867123.0312 +
                               Cn3 * 7572976491371.918 +
                               Cn4 * -2211644574250071.8) +
                      pbar2 * (      -561359759.0576192 +
                               Cn1 * 229139759397.67737 +
                               Cn2 * -17592854499677.438 +
                               Cn3 * -187019215313710.25 +
                               Cn4 * 3.577599475087787e+16) +
                      pbar3 * (      265565792.75786227 +
                               Cn1 * 213118143790.2472 +
                               Cn2 * -85369847361821.81 +
                               Cn3 * 2103265515277225.2 +
                               Cn4 * 1.8439733164392115e+17) +
                      pbar4 * (      47603396254.039764 +
                               Cn1 * -18852936226180.195 +
                               Cn2 * 1327187072992361.8 +
                               Cn3 * 1.3112557280604412e+16 +
                               Cn4 * -1.8625236793107397e+18)) +
               Cm5 * (        (      -10952742.142500937 +
                               Cn1 * 1357531472.3629951 +
                               Cn2 * 147774694702.3083 +
                               Cn3 * -14290353417449.516 +
                               Cn4 * 82418457538724.11) +
                      pbar1 * (      -272076253.47943234 +
                               Cn1 * -163680473755.1141 +
                               Cn2 * 25516461850096.016 +
                               Cn3 * 159141385297770.4 +
                               Cn4 * -5.794848745104072e+16) +
                      pbar2 * (      6915680558.894039 +
                               Cn1 * -1429993158827.2114 +
                               Cn2 * 73124882775608.2 +
                               Cn3 * 3570157084638742.5 +
                               Cn4 * -1.9701447740075834e+17) +
                      pbar3 * (      15777715736.617073 +
                               Cn1 * 25159309773197.633 +
                               Cn2 * -3432002310859944.5 +
                               Cn3 * -2.6517490353291656e+16 +
                               Cn4 * 8.261682487077555e+18) +
                      pbar4 * (      -350352266237.2673 +
                               Cn1 * 44832451561081.31 +
                               Cn2 * -4163249509612856.5 +
                               Cn3 * -1.5725176522907334e+17 +
                               Cn4 * 1.9123062772365165e+19))) +
        CL2 * (      (        (      9.4856799110028 +
                               Cn1 * -367.60710870096966 +
                               Cn2 * -103459.73753604095 +
                               Cn3 * 5757530.464203094 +
                               Cn4 * 294785064.6688107) +
                      pbar1 * (      -185.30859965721376 +
                               Cn1 * 141969.70299421117 +
                               Cn2 * -17760753.332778443 +
                               Cn3 * -250720883.73749053 +
                               Cn4 * 39258675268.84845) +
                      pbar2 * (      -8254.232976197323 +
                               Cn1 * -799250.9376945153 +
                               Cn2 * 395437075.7020311 +
                               Cn3 * -10931420159.490921 +
                               Cn4 * -787359448312.2308) +
                      pbar3 * (      66239.89850501769 +
                               Cn1 * -7205415.603858223 +
                               Cn2 * 1511266937.828998 +
                               Cn3 * -6435848626.615732 +
                               Cn4 * -4917355025042.136) +
                      pbar4 * (      -103151.44386274631 +
                               Cn1 * 154820768.4189926 +
                               Cn2 * -27534209378.818417 +
                               Cn3 * 298726233015.17755 +
                               Cn4 * 62704600782243.625)) +
               Cm1 * (        (      595.2259375748483 +
                               Cn1 * -161041.13962250052 +
                               Cn2 * -1390793.1140277542 +
                               Cn3 * 1064104739.785617 +
                               Cn4 * -35891293736.24587) +
                      pbar1 * (      -4683.161840311286 +
                               Cn1 * 22148589.353141908 +
                               Cn2 * -2372775663.5365086 +
                               Cn3 * -47914531735.826515 +
                               Cn4 * 5963320841426.435) +
                      pbar2 * (      288682.55755537236 +
                               Cn1 * -77404351.07207885 +
                               Cn2 * 268526178.7627847 +
                               Cn3 * 445701021696.7806 +
                               Cn4 * -13788751411695.945) +
                      pbar3 * (      -2265885.5249791937 +
                               Cn1 * -2220593146.850302 +
                               Cn2 * 257758795079.60614 +
                               Cn3 * 5868244436648.761 +
                               Cn4 * -713483530921684.1) +
                      pbar4 * (      -13454810.449794939 +
                               Cn1 * 11325425983.98058 +
                               Cn2 * -542891119048.6866 +
                               Cn3 * -58223294602892.82 +
                               Cn4 * 2852070880775573.5)) +
               Cm2 * (        (      2874.051208111099 +
                               Cn1 * -5705514.231187477 +
                               Cn2 * 517118876.44177663 +
                               Cn3 * 10290814471.92524 +
                               Cn4 * -1492001524048.0483) +
                      pbar1 * (      342863.9616735479 +
                               Cn1 * -140499704.04874668 +
                               Cn2 * 14687888332.88335 +
                               Cn3 * 161071189618.74396 +
                               Cn4 * -34527838728715.61) +
                      pbar2 * (      -10185282.512598367 +
                               Cn1 * 4812330324.6302805 +
                               Cn2 * -412222053184.2226 +
                               Cn3 * -3240497429913.561 +
                               Cn4 * 897576246428239.2) +
                      pbar3 * (      10368462.733507939 +
                               Cn1 * -4221875132.935057 +
                               Cn2 * -968893490049.5881 +
                               Cn3 * 46642006190402.94 +
                               Cn4 * 2263236347172245.5) +
                      pbar4 * (      922630403.3267354 +
                               Cn1 * -383082945656.9628 +
                               Cn2 * 29861449714631.066 +
                               Cn3 * 252778071193645.38 +
                               Cn4 * -5.427969762640562e+16)) +
               Cm3 * (        (      -309396.61983689544 +
                               Cn1 * 30162066.047449116 +
                               Cn2 * 4879166432.315451 +
                               Cn3 * -398737867805.2902 +
                               Cn4 * 3230908483020.7207) +
                      pbar1 * (      -5943718.027107545 +
                               Cn1 * -6618834543.483713 +
                               Cn2 * 879802340717.147 +
                               Cn3 * 11900947585495.535 +
                               Cn4 * -2135779624985823.0) +
                      pbar2 * (      82601001.93510339 +
                               Cn1 * 10300728858.736534 +
                               Cn2 * -917771889817.6348 +
                               Cn3 * -61198611392235.41 +
                               Cn4 * 4310079495276225.0) +
                      pbar3 * (      676436773.300253 +
                               Cn1 * 889350478000.3634 +
                               Cn2 * -109806407524664.64 +
                               Cn3 * -1880797657268078.8 +
                               Cn4 * 2.890174062211989e+17) +
                      pbar4 * (      -3369399474.801381 +
                               Cn1 * -3678454151993.1255 +
                               Cn2 * 230668169251414.56 +
                               Cn3 * 1.4184117691514038e+16 +
                               Cn4 * -7.995116461136614e+17)) +
               Cm4 * (        (      -610698.4520914657 +
                               Cn1 * 782138944.5648985 +
                               Cn2 * -70783992336.97652 +
                               Cn3 * -1208377979803.0122 +
                               Cn4 * 186241559976281.0) +
                      pbar1 * (      -35204428.63149412 +
                               Cn1 * 13730336712.824987 +
                               Cn2 * -1489717189408.2124 +
                               Cn3 * -10735321830290.777 +
                               Cn4 * 3367404026760458.0) +
                      pbar2 * (      1106862635.8243139 +
                               Cn1 * -505350374467.6652 +
                               Cn2 * 42057072658948.1 +
                               Cn3 * 259602198890892.9 +
                               Cn4 * -8.335425717728406e+16) +
                      pbar3 * (      -1381946011.3786433 +
                               Cn1 * 420148154528.29083 +
                               Cn2 * 102767445230359.47 +
                               Cn3 * -5602465366524509.0 +
                               Cn4 * -2.1343541055295296e+17) +
                      pbar4 * (      -90572315139.80812 +
                               Cn1 * 38800896973512.0 +
                               Cn2 * -3018918246039578.0 +
                               Cn3 * -1.1335852278487132e+16 +
                               Cn4 * 4.243289569155459e+18)) +
               Cm5 * (        (      23097914.89732839 +
                               Cn1 * 341177847.81882757 +
                               Cn2 * -642549533488.0844 +
                               Cn3 * 28345351734649.82 +
                               Cn4 * 371232068060215.56) +
                      pbar1 * (      553783860.015197 +
                               Cn1 * 468342425037.3106 +
                               Cn2 * -66627459941381.7 +
                               Cn3 * -674564237114479.4 +
                               Cn4 * 1.575441834717798e+17) +
                      pbar2 * (      -11736330422.076078 +
                               Cn1 * -745624281379.2571 +
                               Cn2 * 139585419970214.48 +
                               Cn3 * 1276729202060019.2 +
                               Cn4 * -3.4319741264642816e+17) +
                      pbar3 * (      -38670808820.328415 +
                               Cn1 * -68301769472751.19 +
                               Cn2 * 8775049149803110.0 +
                               Cn3 * 1.1191555164024778e+17 +
                               Cn4 * -2.257165107351448e+19) +
                      pbar4 * (      506345403711.8363 +
                               Cn1 * 291461072971269.06 +
                               Cn2 * -2.0165707386341452e+16 +
                               Cn3 * -8.298427796982998e+17 +
                               Cn4 * 4.561255179224806e+19))) +
        CL3 * (      (        (      -7.532604339306609 +
                               Cn1 * 1587.8472141536358 +
                               Cn2 * -45308.67220817123 +
                               Cn3 * -5585619.939968083 +
                               Cn4 * 22620601.76909852) +
                      pbar1 * (      32.32395850027677 +
                               Cn1 * -15891.671618099457 +
                               Cn2 * 4511524.342833484 +
                               Cn3 * -46890282.46220439 +
                               Cn4 * -4955988706.523403) +
                      pbar2 * (      8909.750519279733 +
                               Cn1 * -972928.9860891894 +
                               Cn2 * -201260464.11227557 +
                               Cn3 * 12478168275.766438 +
                               Cn4 * 270857278267.7864) +
                      pbar3 * (      -63298.63590194007 +
                               Cn1 * 869934.3648845569 +
                               Cn2 * -393000796.31669116 +
                               Cn3 * 17879617807.173306 +
                               Cn4 * 1511361065232.2249) +
                      pbar4 * (      -43152.72885501679 +
                               Cn1 * 12320893.091221502 +
                               Cn2 * 12085580411.26888 +
                               Cn3 * -649995845009.1533 +
                               Cn4 * -16609998696511.027)) +
               Cm1 * (        (      -477.33545478507267 +
                               Cn1 * 86718.00438494206 +
                               Cn2 * 2827731.910921656 +
                               Cn3 * -627922664.8578818 +
                               Cn4 * 18387095075.21946) +
                      pbar1 * (      5901.043655877756 +
                               Cn1 * -15195093.32301015 +
                               Cn2 * 1575254714.9625585 +
                               Cn3 * 36541049630.77243 +
                               Cn4 * -4124034803431.4043) +
                      pbar2 * (      -205368.74815996038 +
                               Cn1 * 104296511.22203977 +
                               Cn2 * -4899981398.352705 +
                               Cn3 * -416469271307.30237 +
                               Cn4 * 21691457637134.312) +
                      pbar3 * (      1703663.9816531297 +
                               Cn1 * 1457328768.3050835 +
                               Cn2 * -169955457170.30048 +
                               Cn3 * -4249080960118.7725 +
                               Cn4 * 493033302420876.1) +
                      pbar4 * (      6953567.955614485 +
                               Cn1 * -12215742585.998861 +
                               Cn2 * 791492403084.4209 +
                               Cn3 * 53207743492235.03 +
                               Cn4 * -3163143190479580.5)) +
               Cm2 * (        (      -2275.911663755922 +
                               Cn1 * 3594568.367766317 +
                               Cn2 * -321690690.5684322 +
                               Cn3 * -6898267830.378056 +
                               Cn4 * 966627900522.8403) +
                      pbar1 * (      -205871.68506654847 +
                               Cn1 * 53770165.084758736 +
                               Cn2 * -5522036457.771482 +
                               Cn3 * -56778042532.93824 +
                               Cn4 * 13363318787951.953) +
                      pbar2 * (      5556621.998718967 +
                               Cn1 * -2690435912.4697547 +
                               Cn2 * 249725734632.70056 +
                               Cn3 * 615660885341.768 +
                               Cn4 * -519895992584651.7) +
                      pbar3 * (      -4734224.808406505 +
                               Cn1 * 6038918636.975869 +
                               Cn2 * 207649774008.0531 +
                               Cn3 * -33495947391394.168 +
                               Cn4 * -438157967877658.25) +
                      pbar4 * (      -522293415.57160133 +
                               Cn1 * 202318304175.7567 +
                               Cn2 * -17116635927641.104 +
                               Cn3 * -4639064756114.201 +
                               Cn4 * 2.8700296301519276e+16)) +
               Cm3 * (        (      192582.46851201673 +
                               Cn1 * -1080577.9305964422 +
                               Cn2 * -4606121373.98995 +
                               Cn3 * 223644766389.07712 +
                               Cn4 * 969856804558.4442) +
                      pbar1 * (      3424015.767546124 +
                               Cn1 * 4634145832.900848 +
                               Cn2 * -590815714134.3068 +
                               Cn3 * -9774912600514.412 +
                               Cn4 * 1492552769701297.0) +
                      pbar2 * (      -42971300.83127076 +
                               Cn1 * -27479887384.483273 +
                               Cn2 * 2305301917027.8677 +
                               Cn3 * 90757794088893.78 +
                               Cn4 * -7392784748619615.0) +
                      pbar3 * (      -466547084.5900896 +
                               Cn1 * -607380338262.3976 +
                               Cn2 * 72707071329607.19 +
                               Cn3 * 1518450688079755.8 +
                               Cn4 * -2.0202463804745523e+17) +
                      pbar4 * (      1906395435.599146 +
                               Cn1 * 4354035677020.0283 +
                               Cn2 * -312310629113487.06 +
                               Cn3 * -1.5738803365436764e+16 +
                               Cn4 * 1.0390483617545964e+18)) +
               Cm4 * (        (      463051.9092746522 +
                               Cn1 * -511512654.52294856 +
                               Cn2 * 44983353192.56148 +
                               Cn3 * 834087514301.2458 +
                               Cn4 * -120684553112037.08) +
                      pbar1 * (      21261541.776505634 +
                               Cn1 * -5646528576.714161 +
                               Cn2 * 604281008437.4171 +
                               Cn3 * 4300957373325.9873 +
                               Cn4 * -1418849885746052.5) +
                      pbar2 * (      -639403718.6206893 +
                               Cn1 * 298790870215.2188 +
                               Cn2 * -26116801723411.92 +
                               Cn3 * -68215538956091.63 +
                               Cn4 * 5.021361880648778e+16) +
                      pbar3 * (      956963696.3163204 +
                               Cn1 * -594864251216.8403 +
                               Cn2 * -26912585776834.93 +
                               Cn3 * 3801312122309798.0 +
                               Cn4 * 4.5984295540168936e+16) +
                      pbar4 * (      51349553799.22893 +
                               Cn1 * -21668012636262.027 +
                               Cn2 * 1800308399943080.0 +
                               Cn3 * -3902096428171322.5 +
                               Cn4 * -2.3844205973633884e+18)) +
               Cm5 * (        (      -13214222.86216794 +
                               Cn1 * -1807119970.431176 +
                               Cn2 * 534522914880.94995 +
                               Cn3 * -15714704881409.617 +
                               Cn4 * -480526372847660.2) +
                      pbar1 * (      -343799997.00013447 +
                               Cn1 * -326778624512.0986 +
                               Cn2 * 44497967907873.39 +
                               Cn3 * 586029873266569.9 +
                               Cn4 * -1.0959244960526366e+17) +
                      pbar2 * (      6695574887.723226 +
                               Cn1 * 2052436833554.7214 +
                               Cn2 * -218090083061223.4 +
                               Cn3 * -4876626117896836.0 +
                               Cn4 * 5.751264660637744e+17) +
                      pbar3 * (      25694699608.19837 +
                               Cn1 * 46855537227594.93 +
                               Cn2 * -5775312916841196.0 +
                               Cn3 * -9.762956535713026e+16 +
                               Cn4 * 1.5757612835416533e+19) +
                      pbar4 * (      -280856909054.88684 +
                               Cn1 * -341820525550748.44 +
                               Cn2 * 2.542846205334005e+16 +
                               Cn3 * 1.0711539739907936e+18 +
                               Cn4 * -7.079456216250247e+19))));
    
    L[3] = (
              (      (        (      -2.0237949435171636 +
                               Cn1 * -1136.8727884709892 +
                               Cn2 * 71627.36581028624 +
                               Cn3 * 1828358.688178239 +
                               Cn4 * -140494775.46874034) +
                      pbar1 * (      176.13568359549654 +
                               Cn1 * -34590.001729025425 +
                               Cn2 * 1776057.0630347282 +
                               Cn3 * 62576652.87433461 +
                               Cn4 * -3918218961.917468) +
                      pbar2 * (      -664.9416851299068 +
                               Cn1 * 399131.8258478106 +
                               Cn2 * -24409412.52164992 +
                               Cn3 * -854086534.0773894 +
                               Cn4 * 52647406551.001396) +
                      pbar3 * (      -7397.844636699593 +
                               Cn1 * 2192428.6121070934 +
                               Cn2 * -134118699.36977898 +
                               Cn3 * -3505316097.628054 +
                               Cn4 * 299134967085.3556) +
                      pbar4 * (      48423.869093912836 +
                               Cn1 * -33568254.05142322 +
                               Cn2 * 1975664648.440814 +
                               Cn3 * 82106818446.09247 +
                               Cn4 * -4290970142904.2646)) +
               Cm1 * (        (      -91.49769216928507 +
                               Cn1 * 11077.381790058615 +
                               Cn2 * -736173.0980886843 +
                               Cn3 * -23006524.7645726 +
                               Cn4 * 1918864392.910952) +
                      pbar1 * (      -181.3095318284703 +
                               Cn1 * -422295.0152749391 +
                               Cn2 * 53056874.27407785 +
                               Cn3 * 436884781.1480704 +
                               Cn4 * -122976315209.0153) +
                      pbar2 * (      46898.85979250856 +
                               Cn1 * -14337191.097555317 +
                               Cn2 * 1018388366.1937929 +
                               Cn3 * 18873338316.090176 +
                               Cn4 * -2317406172562.0234) +
                      pbar3 * (      -68331.89108615421 +
                               Cn1 * 48994351.39335445 +
                               Cn2 * -5064583736.905195 +
                               Cn3 * -52895707316.03144 +
                               Cn4 * 13518450381779.19) +
                      pbar4 * (      -3615588.620724523 +
                               Cn1 * 998695480.2603829 +
                               Cn2 * -79247163860.41023 +
                               Cn3 * -651253219552.9396 +
                               Cn4 * 188484804827595.66)) +
               Cm2 * (        (      -848.1017773296431 +
                               Cn1 * 364284.72835653956 +
                               Cn2 * -21608982.694732685 +
                               Cn3 * -596744623.818207 +
                               Cn4 * 39802876548.25074) +
                      pbar1 * (      -69092.69068631365 +
                               Cn1 * 15804511.886879137 +
                               Cn2 * -469422088.67674 +
                               Cn3 * -31338020045.773422 +
                               Cn4 * 889561627609.3905) +
                      pbar2 * (      759126.12271057 +
                               Cn1 * -268972590.7111696 +
                               Cn2 * 14419590895.668415 +
                               Cn3 * 474663409380.1522 +
                               Cn4 * -30160028680614.81) +
                      pbar3 * (      3500075.643103636 +
                               Cn1 * -966511992.4672171 +
                               Cn2 * 31296857873.64779 +
                               Cn3 * 1695231291206.552 +
                               Cn4 * -34909709795701.168) +
                      pbar4 * (      -59754899.38765499 +
                               Cn1 * 23443140267.15735 +
                               Cn2 * -1290469556786.7515 +
                               Cn3 * -42808422237120.46 +
                               Cn4 * 2711934966368374.0)) +
               Cm3 * (        (      -4300.539568771901 +
                               Cn1 * 987241.6527697867 +
                               Cn2 * -15930942.099837145 +
                               Cn3 * 1018302600.747588 +
                               Cn4 * -188934151116.0883) +
                      pbar1 * (      522821.9872840664 +
                               Cn1 * 80494324.5835492 +
                               Cn2 * -14340922174.71173 +
                               Cn3 * -84054780016.27863 +
                               Cn4 * 31056949838427.355) +
                      pbar2 * (      -22847552.770446375 +
                               Cn1 * 6036660882.757234 +
                               Cn2 * -338831953845.2672 +
                               Cn3 * -8649120975956.769 +
                               Cn4 * 792390526114663.4) +
                      pbar3 * (      2379585.2234263895 +
                               Cn1 * -14237089383.602718 +
                               Cn2 * 1346500397141.928 +
                               Cn3 * 20578520285995.5 +
                               Cn4 * -3248599867618844.0) +
                      pbar4 * (      1815181342.6999707 +
                               Cn1 * -467353894080.45966 +
                               Cn2 * 27906823717689.082 +
                               Cn3 * 543519607111480.9 +
                               Cn4 * -6.672903920079641e+16)) +
               Cm4 * (        (      60738.78848770272 +
                               Cn1 * -26921256.42689851 +
                               Cn2 * 1729975744.7143564 +
                               Cn3 * 42309032234.2528 +
                               Cn4 * -3074050578154.627) +
                      pbar1 * (      4952539.458888407 +
                               Cn1 * -1205534540.5175576 +
                               Cn2 * 32015769579.57444 +
                               Cn3 * 2534899421782.5825 +
                               Cn4 * -56667558986428.02) +
                      pbar2 * (      -68075940.59792648 +
                               Cn1 * 23448382039.234634 +
                               Cn2 * -1261953775921.3613 +
                               Cn3 * -41119951517734.97 +
                               Cn4 * 2666132715920668.5) +
                      pbar3 * (      -243213739.15305004 +
                               Cn1 * 79462555587.92651 +
                               Cn2 * -2080103372385.7764 +
                               Cn3 * -167456296375899.12 +
                               Cn4 * 1272009502744050.8) +
                      pbar4 * (      5357692275.861315 +
                               Cn1 * -2082146766045.6494 +
                               Cn2 * 116760538635566.2 +
                               Cn3 * 3777230571862546.5 +
                               Cn4 * -2.5786615675340368e+17)) +
               Cm5 * (        (      372565.6523475701 +
                               Cn1 * -111602954.38743946 +
                               Cn2 * 6486108195.152264 +
                               Cn3 * -78353979375.7783 +
                               Cn4 * 6705727806539.053) +
                      pbar1 * (      -48447252.08696526 +
                               Cn1 * -3922485133.7836733 +
                               Cn2 * 899670341564.7018 +
                               Cn3 * 3848608674515.7554 +
                               Cn4 * -1836762507169064.2) +
                      pbar2 * (      1837127793.1664917 +
                               Cn1 * -472057269541.96686 +
                               Cn2 * 23674498485185.492 +
                               Cn3 * 725460177015469.6 +
                               Cn4 * -5.6283044852473544e+16) +
                      pbar3 * (      102359187.70912714 +
                               Cn1 * 939409982982.5953 +
                               Cn2 * -77904110519210.02 +
                               Cn3 * -1413846279922281.5 +
                               Cn4 * 1.606901247861911e+17) +
                      pbar4 * (      -146176687776.38687 +
                               Cn1 * 38200309897903.17 +
                               Cn2 * -2011526032646921.5 +
                               Cn3 * -5.521558865175585e+16 +
                               Cn4 * 4.873884362170735e+18))) +
        CL1 * (      (        (      -20.131783177320102 +
                               Cn1 * 4804.953400830695 +
                               Cn2 * -290277.99080274254 +
                               Cn3 * -9514940.363237828 +
                               Cn4 * 633471636.7183079) +
                      pbar1 * (      -438.3003746747034 +
                               Cn1 * 95513.23078750524 +
                               Cn2 * -1311015.675538928 +
                               Cn3 * -245097158.67755952 +
                               Cn4 * 6662524909.932516) +
                      pbar2 * (      748.6923774260927 +
                               Cn1 * -1920661.643710318 +
                               Cn2 * 197753392.58458582 +
                               Cn3 * 5568579663.024962 +
                               Cn4 * -492006827592.42334) +
                      pbar3 * (      4436.584825782968 +
                               Cn1 * -3221556.019153766 +
                               Cn2 * 355822811.9926355 +
                               Cn3 * -473680681.8370283 +
                               Cn4 * -1257773164577.8687) +
                      pbar4 * (      -14028.656959823045 +
                               Cn1 * 162135244.03770074 +
                               Cn2 * -14240257087.28319 +
                               Cn3 * -625244554881.9855 +
                               Cn4 * 35680940885597.66)) +
               Cm1 * (        (      -656.8566884212911 +
                               Cn1 * 171275.63285514928 +
                               Cn2 * -6769213.361245313 +
                               Cn3 * -193177402.48040316 +
                               Cn4 * 11809100120.543425) +
                      pbar1 * (      -622.2645620837326 +
                               Cn1 * 5551387.8130722 +
                               Cn2 * -497557318.4177703 +
                               Cn3 * -8173751286.663157 +
                               Cn4 * 1167813708836.712) +
                      pbar2 * (      -197085.54879900685 +
                               Cn1 * 60951056.23628805 +
                               Cn2 * -4674433469.332781 +
                               Cn3 * -80856131221.3789 +
                               Cn4 * 10224585378055.24) +
                      pbar3 * (      1099288.9199864161 +
                               Cn1 * -583959066.2880008 +
                               Cn2 * 42798164268.402466 +
                               Cn3 * 862351210097.0688 +
                               Cn4 * -131167188646455.42) +
                      pbar4 * (      18350347.7335308 +
                               Cn1 * -4033115293.984427 +
                               Cn2 * 377618133973.028 +
                               Cn3 * -2966982678832.544 +
                               Cn4 * -874393593721271.2)) +
               Cm2 * (        (      5719.679618185769 +
                               Cn1 * -2042394.7490185343 +
                               Cn2 * 138233264.77113765 +
                               Cn3 * 4258105432.2445264 +
                               Cn4 * -293151006698.73456) +
                      pbar1 * (      398879.99039378046 +
                               Cn1 * -67272349.62460995 +
                               Cn2 * -774782348.2702429 +
                               Cn3 * 187909673437.40585 +
                               Cn4 * 263081426972.91187) +
                      pbar2 * (      -6716961.112387861 +
                               Cn1 * 2168365454.9919996 +
                               Cn2 * -131654777652.16086 +
                               Cn3 * -4678741297587.504 +
                               Cn4 * 316419448288849.4) +
                      pbar3 * (      -4853652.695532879 +
                               Cn1 * 1448817926.513827 +
                               Cn2 * 80515323856.02805 +
                               Cn3 * -3473127255111.3286 +
                               Cn4 * -166569195033000.44) +
                      pbar4 * (      470267393.063577 +
                               Cn1 * -176356799361.4261 +
                               Cn2 * 10656498121972.145 +
                               Cn3 * 430693665871173.9 +
                               Cn4 * -2.5992471327026896e+16)) +
               Cm3 * (        (      154448.86088201858 +
                               Cn1 * -45932618.77567033 +
                               Cn2 * 2476635411.3207965 +
                               Cn3 * 32613218750.52407 +
                               Cn4 * -3177240267727.672) +
                      pbar1 * (      -5014181.216570792 +
                               Cn1 * -1181313603.5562298 +
                               Cn2 * 152614767956.7632 +
                               Cn3 * 1926747250913.4866 +
                               Cn4 * -363236132016039.75) +
                      pbar2 * (      174430235.6243281 +
                               Cn1 * -37944940802.667404 +
                               Cn2 * 1714142355783.6907 +
                               Cn3 * 53867838231718.25 +
                               Cn4 * -4022348146038663.5) +
                      pbar3 * (      -218692665.70681483 +
                               Cn1 * 200556986019.54123 +
                               Cn2 * -13870016716955.518 +
                               Cn3 * -378818588965431.25 +
                               Cn4 * 4.008711850861238e+16) +
                      pbar4 * (      -13845911726.838238 +
                               Cn1 * 2886642493902.92 +
                               Cn2 * -141125030649846.72 +
                               Cn3 * -2112251938574706.2 +
                               Cn4 * 3.271840476704518e+17)) +
               Cm4 * (        (      -396164.0685158664 +
                               Cn1 * 161228111.22531748 +
                               Cn2 * -11419158153.382383 +
                               Cn3 * -356126011831.0347 +
                               Cn4 * 23579970949189.332) +
                      pbar1 * (      -31089360.4594963 +
                               Cn1 * 5832122727.099224 +
                               Cn2 * 87473742330.64392 +
                               Cn3 * -18139014605364.316 +
                               Cn4 * -59778661299102.15) +
                      pbar2 * (      648252691.9194177 +
                               Cn1 * -203120796557.5135 +
                               Cn2 * 11754904330333.244 +
                               Cn3 * 440668572741302.7 +
                               Cn4 * -2.8970477155784296e+16) +
                      pbar3 * (      -25538132.539292313 +
                               Cn1 * -168526525090.0949 +
                               Cn2 * -8193396636412.724 +
                               Cn3 * 839896882067232.9 +
                               Cn4 * 1.1764772634461906e+16) +
                      pbar4 * (      -44972442021.73651 +
                               Cn1 * 16890738537774.719 +
                               Cn2 * -994527244593323.4 +
                               Cn3 * -4.19350091542584e+16 +
                               Cn4 * 2.623167800895167e+18)) +
               Cm5 * (        (      -8512702.899407418 +
                               Cn1 * 2824843114.086684 +
                               Cn2 * -188368918996.87622 +
                               Cn3 * -1035828047612.7457 +
                               Cn4 * 213336875498282.0) +
                      pbar1 * (      485312093.0018022 +
                               Cn1 * 65135557882.3314 +
                               Cn2 * -10248997122363.613 +
                               Cn3 * -115019794117055.9 +
                               Cn4 * 2.4007423868653196e+16) +
                      pbar2 * (      -15567364087.782784 +
                               Cn1 * 3313520266743.674 +
                               Cn2 * -119429060778922.44 +
                               Cn3 * -5070527589329638.0 +
                               Cn4 * 2.835401330665861e+17) +
                      pbar3 * (      15047264437.441463 +
                               Cn1 * -14290675094177.38 +
                               Cn2 * 902616537839694.8 +
                               Cn3 * 2.755581110633627e+16 +
                               Cn4 * -2.407528987430176e+18) +
                      pbar4 * (      1201366658396.583 +
                               Cn1 * -263801355120724.62 +
                               Cn2 * 1.0043580213002862e+16 +
                               Cn3 * 3.1658097242435386e+17 +
                               Cn4 * -2.363930560036886e+19))) +
        CL2 * (      (        (      2.812954452732338 +
                               Cn1 * -2247.8046858233997 +
                               Cn2 * 283319.01384327526 +
                               Cn3 * 8243336.364992453 +
                               Cn4 * -681019963.3116736) +
                      pbar1 * (      841.7079023078606 +
                               Cn1 * -67373.70716709079 +
                               Cn2 * -9150125.75795886 +
                               Cn3 * 292920874.6171387 +
                               Cn4 * 10078599580.24439) +
                      pbar2 * (      1542.1348200347816 +
                               Cn1 * 3347786.097062535 +
                               Cn2 * -395990749.31950927 +
                               Cn3 * -11018244847.52927 +
                               Cn4 * 1018276798760.4232) +
                      pbar3 * (      19072.7897903219 +
                               Cn1 * -3894433.653767784 +
                               Cn2 * -290856599.64543897 +
                               Cn3 * 36556813284.87769 +
                               Cn4 * 1675710037261.2815) +
                      pbar4 * (      -100591.95516893231 +
                               Cn1 * -287020988.9487554 +
                               Cn2 * 25359451918.700016 +
                               Cn3 * 1307767268616.9065 +
                               Cn4 * -62017079360990.32)) +
               Cm1 * (        (      2011.249941563985 +
                               Cn1 * -462519.7002975712 +
                               Cn2 * 18206334.86725745 +
                               Cn3 * 537411006.2576019 +
                               Cn4 * -35259381888.48247) +
                      pbar1 * (      -5980.8339776334 +
                               Cn1 * -10983999.922818216 +
                               Cn2 * 895098531.0253924 +
                               Cn3 * 20445749148.16641 +
                               Cn4 * -2283142949515.523) +
                      pbar2 * (      449164.8222064954 +
                               Cn1 * -115822852.72186948 +
                               Cn2 * 5693594376.131401 +
                               Cn3 * 176580795899.37234 +
                               Cn4 * -9468838102216.672) +
                      pbar3 * (      -1962928.3010488662 +
                               Cn1 * 1138847087.9250023 +
                               Cn2 * -77953476761.11339 +
                               Cn3 * -2060497917047.977 +
                               Cn4 * 299640666614028.2) +
                      pbar4 * (      -44012290.63889492 +
                               Cn1 * 6755787941.003825 +
                               Cn2 * -445309049164.13727 +
                               Cn3 * 15213143214545.346 +
                               Cn4 * 722879873634392.9)) +
               Cm2 * (        (      -2180.106588516502 +
                               Cn1 * 1439312.9775264899 +
                               Cn2 * -146614914.38629636 +
                               Cn3 * -5450842040.39146 +
                               Cn4 * 340781371312.41455) +
                      pbar1 * (      -772589.3630055711 +
                               Cn1 * 75871832.96430135 +
                               Cn2 * 7544807185.782702 +
                               Cn3 * -318994044476.9897 +
                               Cn4 * -11147969309069.45) +
                      pbar2 * (      12572525.074224478 +
                               Cn1 * -4109460494.2057915 +
                               Cn2 * 265064338420.3435 +
                               Cn3 * 9691277128367.674 +
                               Cn4 * -660757873875837.1) +
                      pbar3 * (      -5135109.758517453 +
                               Cn1 * 3870918761.00985 +
                               Cn2 * -563070117107.1915 +
                               Cn3 * -6870661330783.72 +
                               Cn4 * 850379223808160.0) +
                      pbar4 * (      -856312317.2978878 +
                               Cn1 * 336478889690.7547 +
                               Cn2 * -20145504050880.02 +
                               Cn3 * -972705865832917.2 +
                               Cn4 * 5.101312181639998e+16)) +
               Cm3 * (        (      -497619.4810259878 +
                               Cn1 * 128990366.25021398 +
                               Cn2 * -6617576651.717934 +
                               Cn3 * -104046650657.85432 +
                               Cn4 * 9786734362569.463) +
                      pbar1 * (      13890354.472297808 +
                               Cn1 * 2474131114.9775004 +
                               Cn2 * -323736968304.9986 +
                               Cn3 * -5273140937896.425 +
                               Cn4 * 833721222672588.8) +
                      pbar2 * (      -374621585.40643966 +
                               Cn1 * 73971461200.55917 +
                               Cn2 * -2274893514318.6055 +
                               Cn3 * -108322108014291.03 +
                               Cn4 * 4725956571539220.0) +
                      pbar3 * (      383697746.57687414 +
                               Cn1 * -444706710724.3692 +
                               Cn2 * 30179457397241.832 +
                               Cn3 * 1013568800674456.6 +
                               Cn4 * -1.0435667798853576e+17) +
                      pbar4 * (      30157751550.71087 +
                               Cn1 * -5352317810699.288 +
                               Cn2 * 172840153735700.5 +
                               Cn3 * 1156344331972650.5 +
                               Cn4 * -2.96337632328039e+17)) +
               Cm4 * (        (      225831.0468346075 +
                               Cn1 * -139437028.55760944 +
                               Cn2 * 12633301852.080736 +
                               Cn3 * 551107449168.0886 +
                               Cn4 * -28625965253804.004) +
                      pbar1 * (      57635846.45103991 +
                               Cn1 * -6877683816.487423 +
                               Cn2 * -652171645381.8787 +
                               Cn3 * 33492999314542.57 +
                               Cn4 * 849345033799093.2) +
                      pbar2 * (      -1242914852.8796296 +
                               Cn1 * 389719022650.817 +
                               Cn2 * -23438349947698.95 +
                               Cn3 * -938941084022379.4 +
                               Cn4 * 6.035437114577077e+16) +
                      pbar3 * (      1649711093.5546756 +
                               Cn1 * -192302712777.50592 +
                               Cn2 * 46708939718916.17 +
                               Cn3 * -1167403479257751.0 +
                               Cn4 * -3.167100215845904e+16) +
                      pbar4 * (      81863670857.2376 +
                               Cn1 * -33052526162146.516 +
                               Cn2 * 1907168518679455.2 +
                               Cn3 * 1.0013803027814955e+17 +
                               Cn4 * -5.446604501926446e+18)) +
               Cm5 * (        (      28952865.430536393 +
                               Cn1 * -8119179572.718818 +
                               Cn2 * 500362904271.42804 +
                               Cn3 * 4408229104951.756 +
                               Cn4 * -666861277037168.0) +
                      pbar1 * (      -1255797054.782501 +
                               Cn1 * -145851991672.36636 +
                               Cn2 * 23156176357154.645 +
                               Cn3 * 341498890891946.6 +
                               Cn4 * -5.852714125646908e+16) +
                      pbar2 * (      33289692521.866016 +
                               Cn1 * -6512580911707.893 +
                               Cn2 * 152414152837732.1 +
                               Cn3 * 1.0105407312249758e+16 +
                               Cn4 * -3.158337901978379e+17) +
                      pbar3 * (      -29952949721.73702 +
                               Cn1 * 32925228598553.195 +
                               Cn2 * -2087026298485352.2 +
                               Cn3 * -7.390336308177045e+16 +
                               Cn4 * 6.619006802940453e+18) +
                      pbar4 * (      -2571306427145.4185 +
                               Cn1 * 502896330934772.5 +
                               Cn2 * -1.184482941883833e+16 +
                               Cn3 * -4.541326664015174e+17 +
                               Cn4 * 2.1158587868558017e+19))) +
        CL3 * (      (        (      9.130059742742072 +
                               Cn1 * -704.626753647443 +
                               Cn2 * -94288.77923278433 +
                               Cn3 * -1979298.065288728 +
                               Cn4 * 237404204.38596117) +
                      pbar1 * (      -467.20935728724317 +
                               Cn1 * 4633.987971880975 +
                               Cn2 * 7804844.712134302 +
                               Cn3 * -99824149.40897879 +
                               Cn4 * -11462115215.794626) +
                      pbar2 * (      -1383.4004974495053 +
                               Cn1 * -2103593.5794163295 +
                               Cn2 * 232680403.09537607 +
                               Cn3 * 7143900323.044728 +
                               Cn4 * -604083401971.6296) +
                      pbar3 * (      -22914.278100665746 +
                               Cn1 * 4966626.154473068 +
                               Cn2 * 194976934.35217467 +
                               Cn3 * -35267693264.9125 +
                               Cn4 * -907062593918.6505) +
                      pbar4 * (      29059.30024621195 +
                               Cn1 * 172965407.65186736 +
                               Cn2 * -13055950439.730583 +
                               Cn3 * -809604675546.934 +
                               Cn4 * 29924527946832.93)) +
               Cm1 * (        (      -1349.9292640215924 +
                               Cn1 * 278735.019681147 +
                               Cn2 * -10228602.778827284 +
                               Cn3 * -315617679.4891062 +
                               Cn4 * 20852212311.605953) +
                      pbar1 * (      7085.595887737024 +
                               Cn1 * 5856334.455477179 +
                               Cn2 * -431840298.72520125 +
                               Cn3 * -13503699345.238304 +
                               Cn4 * 1228808051471.714) +
                      pbar2 * (      -328816.15005665243 +
                               Cn1 * 76758718.36157252 +
                               Cn2 * -1943520790.6606588 +
                               Cn3 * -131251466059.51466 +
                               Cn4 * 925410500606.611) +
                      pbar3 * (      984638.4620963854 +
                               Cn1 * -599584856.2606337 +
                               Cn2 * 38204475390.993225 +
                               Cn3 * 1315223662129.841 +
                               Cn4 * -186239429037872.53) +
                      pbar4 * (      32072696.014165245 +
                               Cn1 * -4081822072.454466 +
                               Cn2 * 126971891176.10854 +
                               Cn3 * -12221856803284.494 +
                               Cn4 * 62750994640697.75)) +
               Cm2 * (        (      -3767.751047881536 +
                               Cn1 * 309805.44944028306 +
                               Cn2 * 34594864.601095796 +
                               Cn3 * 1764718577.7525127 +
                               Cn4 * -93676459805.03194) +
                      pbar1 * (      481863.4656097326 +
                               Cn1 * -26212996.055779405 +
                               Cn2 * -6595784669.845377 +
                               Cn3 * 166018835134.48846 +
                               Cn4 * 10598393105038.037) +
                      pbar2 * (      -7156566.375802569 +
                               Cn1 * 2339572826.9688478 +
                               Cn2 * -152870661590.1307 +
                               Cn3 * -5784035237533.801 +
                               Cn4 * 385625299718932.5) +
                      pbar3 * (      6542550.42059594 +
                               Cn1 * -4445372253.023979 +
                               Cn2 * 448324819711.7774 +
                               Cn3 * 10092884201992.148 +
                               Cn4 * -642575085106754.8) +
                      pbar4 * (      485633023.6515275 +
                               Cn1 * -194352501020.9255 +
                               Cn2 * 10991686310342.3 +
                               Cn3 * 614881730433944.1 +
                               Cn4 * -2.7715114721519996e+16)) +
               Cm3 * (        (      366315.8089399486 +
                               Cn1 * -84224913.17950661 +
                               Cn2 * 4018001826.892533 +
                               Cn3 * 70180458977.29285 +
                               Cn4 * -6153951974628.436) +
                      pbar1 * (      -10265619.659651347 +
                               Cn1 * -1323158503.2365909 +
                               Cn2 * 183907845290.30872 +
                               Cn3 * 3675306982659.244 +
                               Cn4 * -511184224423443.0) +
                      pbar2 * (      242319974.56246564 +
                               Cn1 * -45413187617.445786 +
                               Cn2 * 846057879089.0391 +
                               Cn3 * 68720151093780.74 +
                               Cn4 * -1197253454580222.5) +
                      pbar3 * (      -157918734.59159234 +
                               Cn1 * 257361692998.6701 +
                               Cn2 * -17338258595038.135 +
                               Cn3 * -694173695576115.8 +
                               Cn4 * 7.020326468454045e+16) +
                      pbar4 * (      -19669038748.664906 +
                               Cn1 * 3132964084295.182 +
                               Cn2 * -50532906994899.47 +
                               Cn3 * 605292796908542.5 +
                               Cn4 * -1.2492468839630002e+16)) +
               Cm4 * (        (      237019.42891581779 +
                               Cn1 * -13540026.08243347 +
                               Cn2 * -2613871357.6665983 +
                               Cn3 * -216074687151.44644 +
                               Cn4 * 7320638524588.565) +
                      pbar1 * (      -36649958.352040924 +
                               Cn1 * 2542195972.8720303 +
                               Cn2 * 576541984034.8186 +
                               Cn3 * -18945386709722.297 +
                               Cn4 * -806199227354568.9) +
                      pbar2 * (      719088525.0356718 +
                               Cn1 * -219930217074.24738 +
                               Cn2 * 13219859713728.96 +
                               Cn3 * 561892782668322.8 +
                               Cn4 * -3.4742294658300868e+16) +
                      pbar3 * (      -1338676579.1030893 +
                               Cn1 * 276871058471.3389 +
                               Cn2 * -37318994381126.266 +
                               Cn3 * 482159924110638.1 +
                               Cn4 * 1.6846176927354204e+16) +
                      pbar4 * (      -45450751613.55271 +
                               Cn1 * 19242198738173.152 +
                               Cn2 * -1054671023446150.2 +
                               Cn3 * -6.524030093897783e+16 +
                               Cn4 * 3.126636992108391e+18)) +
               Cm5 * (        (      -22612495.723735478 +
                               Cn1 * 5533863935.099214 +
                               Cn2 * -314404059534.45087 +
                               Cn3 * -3511754041371.4863 +
                               Cn4 * 439442593508338.8) +
                      pbar1 * (      912495180.54743 +
                               Cn1 * 79053674644.87259 +
                               Cn2 * -13955864205054.451 +
                               Cn3 * -245126332623761.28 +
                               Cn4 * 3.7634004258653816e+16) +
                      pbar2 * (      -21237067395.590843 +
                               Cn1 * 3923179953310.56 +
                               Cn2 * -50385817414575.1 +
                               Cn3 * -6108831847835409.0 +
                               Cn4 * 5.87610610698735e+16) +
                      pbar3 * (      13310582615.946268 +
                               Cn1 * -19444747627458.793 +
                               Cn2 * 1253628021325605.5 +
                               Cn3 * 5.063627382858382e+16 +
                               Cn4 * -4.5860853939904e+18) +
                      pbar4 * (      1639263362505.5356 +
                               Cn1 * -294043890344671.8 +
                               Cn2 * 3077273173306351.0 +
                               Cn3 * 1.8171081996753955e+17 +
                               Cn4 * 1.6088608545873316e+18))));
    
    L[4] = (
              (      (        (      1.1687488015135177 +
                               Cn1 * -1570.58149483272 +
                               Cn2 * 37819.3356133603 +
                               Cn3 * 2712983.449907872 +
                               Cn4 * -66507784.77117975) +
                      pbar1 * (      34.23853149980568 +
                               Cn1 * 3143.744173777344 +
                               Cn2 * 432473.61958064325 +
                               Cn3 * 4030944.7012048922 +
                               Cn4 * -2131571253.7524452) +
                      pbar2 * (      320.0883150940021 +
                               Cn1 * 203222.0046761531 +
                               Cn2 * -11705884.505015226 +
                               Cn3 * -737423509.4796108 +
                               Cn4 * 24679523130.289814) +
                      pbar3 * (      -3247.7753263837276 +
                               Cn1 * 187484.87309617567 +
                               Cn2 * -20228546.19890128 +
                               Cn3 * -3134997901.8265057 +
                               Cn4 * 173085377471.3715) +
                      pbar4 * (      22957.616292448158 +
                               Cn1 * -19648121.45627985 +
                               Cn2 * 1063375030.6728299 +
                               Cn3 * 74096773469.72554 +
                               Cn4 * -3333183007837.491)) +
               Cm1 * (        (      -233.4521521119998 +
                               Cn1 * 29069.739927060473 +
                               Cn2 * -78625.13316951938 +
                               Cn3 * -82025571.69181596 +
                               Cn4 * 1091063235.147488) +
                      pbar1 * (      1936.273468703938 +
                               Cn1 * -244426.50417160758 +
                               Cn2 * 153834.67274309805 +
                               Cn3 * 1127809155.4237828 +
                               Cn4 * -64321279455.38173) +
                      pbar2 * (      -4555.195921723673 +
                               Cn1 * 7043763.036408335 +
                               Cn2 * -366720289.5462774 +
                               Cn3 * -25022081339.746265 +
                               Cn4 * 1062022410656.3108) +
                      pbar3 * (      11489.290799330067 +
                               Cn1 * 44365043.9991388 +
                               Cn2 * -5085175420.730423 +
                               Cn3 * -162679636843.95068 +
                               Cn4 * 20898671031165.027) +
                      pbar4 * (      1404558.0413152126 +
                               Cn1 * -1299099688.8254433 +
                               Cn2 * 61811243787.68417 +
                               Cn3 * 4962206606164.61 +
                               Cn4 * -188310003885891.7)) +
               Cm2 * (        (      1193.5543509679626 +
                               Cn1 * 8166.21042172233 +
                               Cn2 * -12464343.804372873 +
                               Cn3 * -221758519.6615163 +
                               Cn4 * 36553312126.058334) +
                      pbar1 * (      9893.353927078866 +
                               Cn1 * 4604474.348661296 +
                               Cn2 * -610754387.9434633 +
                               Cn3 * -883182475.5035496 +
                               Cn4 * 1230492507255.8083) +
                      pbar2 * (      -320774.0007272787 +
                               Cn1 * -143423420.0996335 +
                               Cn2 * 9627230740.346813 +
                               Cn3 * 384145602647.16437 +
                               Cn4 * -20147576636031.45) +
                      pbar3 * (      2224582.007494292 +
                               Cn1 * -379138523.8879051 +
                               Cn2 * 42176036446.70998 +
                               Cn3 * 69107251055.01804 +
                               Cn4 * -74058933561478.05) +
                      pbar4 * (      -14337425.024822762 +
                               Cn1 * 15497284614.286898 +
                               Cn2 * -981649686931.1373 +
                               Cn3 * -35866939160270.65 +
                               Cn4 * 2087620780749126.5)) +
               Cm3 * (        (      67870.23621976415 +
                               Cn1 * -11226489.687313577 +
                               Cn2 * 65348741.76567503 +
                               Cn3 * 26037564736.225964 +
                               Cn4 * -268851583627.7911) +
                      pbar1 * (      -444182.05373830494 +
                               Cn1 * 98091473.21824257 +
                               Cn2 * -7362187267.218893 +
                               Cn3 * -236511516910.57245 +
                               Cn4 * 28379785461717.496) +
                      pbar2 * (      8142006.041726363 +
                               Cn1 * -2387810750.5027494 +
                               Cn2 * 133207113509.62314 +
                               Cn3 * 8681213768056.478 +
                               Cn4 * -371118739971231.6) +
                      pbar3 * (      -3235744.6901210546 +
                               Cn1 * -21056405670.116222 +
                               Cn2 * 2063818811348.3118 +
                               Cn3 * 44346132381281.89 +
                               Cn4 * -6447855435436129.0) +
                      pbar4 * (      -1227816136.421176 +
                               Cn1 * 512488606267.23114 +
                               Cn2 * -23180133271858.688 +
                               Cn3 * -1707463725071711.2 +
                               Cn4 * 6.285143987825942e+16)) +
               Cm4 * (        (      -75766.36274273605 +
                               Cn1 * -10813067.095734248 +
                               Cn2 * 1859992797.8617167 +
                               Cn3 * 52966351365.121956 +
                               Cn4 * -5851881016066.487) +
                      pbar1 * (      594435.4263154485 +
                               Cn1 * -586880816.2856821 +
                               Cn2 * 70105565890.3915 +
                               Cn3 * -46539455341.38247 +
                               Cn4 * -122301569065220.0) +
                      pbar2 * (      13930310.575115632 +
                               Cn1 * 11354697574.535404 +
                               Cn2 * -1116722327538.5288 +
                               Cn3 * -23588728140786.215 +
                               Cn4 * 2347416901667937.0) +
                      pbar3 * (      -271384872.7738524 +
                               Cn1 * 39581850233.002594 +
                               Cn2 * -4899387875491.465 +
                               Cn3 * 63434867862277.664 +
                               Cn4 * 5812006894145010.0) +
                      pbar4 * (      2709267857.3980236 +
                               Cn1 * -1148359240977.7434 +
                               Cn2 * 97711889452584.97 +
                               Cn3 * 1521533516513810.0 +
                               Cn4 * -1.7618345215360925e+17)) +
               Cm5 * (        (      -4722738.375906478 +
                               Cn1 * 728757190.9980862 +
                               Cn2 * 1473789270.2630808 +
                               Cn3 * -1507254892179.9182 +
                               Cn4 * -5561421323680.8125) +
                      pbar1 * (      38015992.827120945 +
                               Cn1 * -11412155642.912472 +
                               Cn2 * 716520765581.8063 +
                               Cn3 * 19220061100110.902 +
                               Cn4 * -1973589231919414.5) +
                      pbar2 * (      -671780295.8713036 +
                               Cn1 * 202258415800.9964 +
                               Cn2 * -10692515968169.945 +
                               Cn3 * -630508649354391.8 +
                               Cn4 * 2.673279459472808e+16) +
                      pbar3 * (      -103629407.8149514 +
                               Cn1 * 1894253333127.301 +
                               Cn2 * -158984539526203.8 +
                               Cn3 * -3061253048777371.0 +
                               Cn4 * 4.168436146669204e+17) +
                      pbar4 * (      101854180238.33449 +
                               Cn1 * -40517022826468.234 +
                               Cn2 * 1691157601086606.5 +
                               Cn3 * 1.1775380952462848e+17 +
                               Cn4 * -4.0212056001905956e+18))) +
        CL1 * (      (        (      25.977864685019785 +
                               Cn1 * -1940.4732294185042 +
                               Cn2 * -256501.88295482524 +
                               Cn3 * 5775720.019910389 +
                               Cn4 * 408442114.9312002) +
                      pbar1 * (      -53.11395453736752 +
                               Cn1 * 10854.253712681044 +
                               Cn2 * -4211040.15168074 +
                               Cn3 * -157023007.64826268 +
                               Cn4 * 26592309968.602245) +
                      pbar2 * (      -4229.813588099212 +
                               Cn1 * -2402764.1991516836 +
                               Cn2 * 232511599.18402585 +
                               Cn3 * 9441501284.339436 +
                               Cn4 * -548349238334.84186) +
                      pbar3 * (      59758.32466304059 +
                               Cn1 * -776066.9143587175 +
                               Cn2 * -137221321.5958083 +
                               Cn3 * 39401656375.81126 +
                               Cn4 * -1994856366674.3452) +
                      pbar4 * (      -562288.6784833126 +
                               Cn1 * 236834108.17501774 +
                               Cn2 * -15866309909.64287 +
                               Cn3 * -1021333976333.3232 +
                               Cn4 * 53750950715736.31)) +
               Cm1 * (        (      2354.6082632643443 +
                               Cn1 * -374216.59395083727 +
                               Cn2 * 1490285.0052604256 +
                               Cn3 * 1063172934.1364576 +
                               Cn4 * -13927907193.880793) +
                      pbar1 * (      -23720.649826722998 +
                               Cn1 * 3913799.410505422 +
                               Cn2 * -55364776.91498048 +
                               Cn3 * -17077640001.997118 +
                               Cn4 * 994734177965.9846) +
                      pbar2 * (      194327.55225111244 +
                               Cn1 * -117643835.01966216 +
                               Cn2 * 6563714746.368454 +
                               Cn3 * 381612114666.9844 +
                               Cn4 * -20306203445723.098) +
                      pbar3 * (      635777.8283292744 +
                               Cn1 * -668876495.2521081 +
                               Cn2 * 57460757784.85133 +
                               Cn3 * 2455090918880.4214 +
                               Cn4 * -262759053178672.06) +
                      pbar4 * (      -34851131.939545386 +
                               Cn1 * 19470969547.856594 +
                               Cn2 * -901506566048.0696 +
                               Cn3 * -71217148765000.58 +
                               Cn4 * 2918258343487579.5)) +
               Cm2 * (        (      -8227.254190195375 +
                               Cn1 * -1505527.3400518273 +
                               Cn2 * 223750490.3773152 +
                               Cn3 * 6781862225.577694 +
                               Cn4 * -662174167060.5297) +
                      pbar1 * (      -152037.96508708148 +
                               Cn1 * -24966067.380152136 +
                               Cn2 * 5486720938.475257 +
                               Cn3 * -77521158647.12766 +
                               Cn4 * -9913408203575.66) +
                      pbar2 * (      2543195.207586377 +
                               Cn1 * 1801449128.510402 +
                               Cn2 * -145235734371.1519 +
                               Cn3 * -5085409134527.448 +
                               Cn4 * 330620283995377.94) +
                      pbar3 * (      -15514740.21382117 +
                               Cn1 * -3185452916.1390586 +
                               Cn2 * -151185122567.59732 +
                               Cn3 * 16915950937130.568 +
                               Cn4 * 243806660228239.5) +
                      pbar4 * (      268358856.84096426 +
                               Cn1 * -152893965811.0469 +
                               Cn2 * 11593231919144.355 +
                               Cn3 * 422046277063817.44 +
                               Cn4 * -2.6783606004927256e+16)) +
               Cm3 * (        (      -743061.8994080082 +
                               Cn1 * 113169561.52262546 +
                               Cn2 * 158566414.39426836 +
                               Cn3 * -257744812450.0123 +
                               Cn4 * 431417553590.3832) +
                      pbar1 * (      6724473.526220897 +
                               Cn1 * -1277195528.9872885 +
                               Cn2 * 59582827391.79699 +
                               Cn3 * 3051647967712.915 +
                               Cn4 * -285883764681106.6) +
                      pbar2 * (      -164481913.27117494 +
                               Cn1 * 43589342273.326355 +
                               Cn2 * -2277075336829.7026 +
                               Cn3 * -142262095945126.28 +
                               Cn4 * 6787348489640481.0) +
                      pbar3 * (      72960216.90073651 +
                               Cn1 * 222540383318.98267 +
                               Cn2 * -18410371148670.656 +
                               Cn3 * -486195208711988.4 +
                               Cn4 * 6.70599924027985e+16) +
                      pbar4 * (      20257346594.42036 +
                               Cn1 * -7293181557865.677 +
                               Cn2 * 305338932048959.0 +
                               Cn3 * 2.4400743635093096e+16 +
                               Cn4 * -8.88594110727602e+17)) +
               Cm4 * (        (      -7420.836365468414 +
                               Cn1 * 301564640.2594554 +
                               Cn2 * -25641241677.090183 +
                               Cn3 * -1032376680355.091 +
                               Cn4 * 77166975417381.1) +
                      pbar1 * (      3847130.0355049446 +
                               Cn1 * 2247999281.669053 +
                               Cn2 * -621692791631.224 +
                               Cn3 * 13954720855053.88 +
                               Cn4 * 868818462492886.6) +
                      pbar2 * (      -35678856.18063905 +
                               Cn1 * -123018390589.51965 +
                               Cn2 * 13250980376760.316 +
                               Cn3 * 256212793173863.66 +
                               Cn4 * -2.6501837306450468e+16) +
                      pbar3 * (      1415129246.5359156 +
                               Cn1 * 369161544399.94977 +
                               Cn2 * 23573879970912.45 +
                               Cn3 * -2799767672930395.0 +
                               Cn4 * 3902492920718835.5) +
                      pbar4 * (      -38162423068.71812 +
                               Cn1 * 8714865910339.712 +
                               Cn2 * -956945352940253.8 +
                               Cn3 * -1.0290423256730626e+16 +
                               Cn4 * 1.5359374505739174e+18)) +
               Cm5 * (        (      45977761.66319389 +
                               Cn1 * -6461664857.758483 +
                               Cn2 * -79425736940.41455 +
                               Cn3 * 12884299788853.895 +
                               Cn4 * 224182826701190.12) +
                      pbar1 * (      -473899706.8051062 +
                               Cn1 * 118145381925.59708 +
                               Cn2 * -5267239791725.507 +
                               Cn3 * -168332470714218.12 +
                               Cn4 * 1.5983139939252606e+16) +
                      pbar2 * (      13221201502.239273 +
                               Cn1 * -3402109814825.8716 +
                               Cn2 * 155503067994730.88 +
                               Cn3 * 9964718657061694.0 +
                               Cn4 * -4.067588248760532e+17) +
                      pbar3 * (      -13883103414.358011 +
                               Cn1 * -17120187094948.371 +
                               Cn2 * 1291502681689481.8 +
                               Cn3 * 2.384779424992216e+16 +
                               Cn4 * -3.7920876045270625e+18) +
                      pbar4 * (      -1577149624181.6191 +
                               Cn1 * 546126430964747.2 +
                               Cn2 * -2.006726278881402e+16 +
                               Cn3 * -1.6382918502720801e+18 +
                               Cn4 * 4.978380801604921e+19))) +
        CL2 * (      (        (      -56.50750974301278 +
                               Cn1 * 1945.524639924296 +
                               Cn2 * 713314.8451786189 +
                               Cn3 * -9972950.26329809 +
                               Cn4 * -1094006200.005593) +
                      pbar1 * (      -1021.7274135631557 +
                               Cn1 * -45008.71852528294 +
                               Cn2 * 23850027.840064522 +
                               Cn3 * 465983016.44430196 +
                               Cn4 * -95933553388.87434) +
                      pbar2 * (      15453.331027841568 +
                               Cn1 * 6169636.6242859755 +
                               Cn2 * -610868408.6713679 +
                               Cn3 * -22409249530.434513 +
                               Cn4 * 1335379784317.1392) +
                      pbar3 * (      -127718.23537700225 +
                               Cn1 * 7422606.456448411 +
                               Cn2 * -143844400.40177146 +
                               Cn3 * -155292217466.35815 +
                               Cn4 * 7775891757830.263) +
                      pbar4 * (      1059159.5935614184 +
                               Cn1 * -575880161.5448291 +
                               Cn2 * 43609894905.485855 +
                               Cn3 * 2578896863640.7705 +
                               Cn4 * -154763276796991.53)) +
               Cm1 * (        (      -6924.758922300608 +
                               Cn1 * 1100002.6954076728 +
                               Cn2 * -1592919.7170372664 +
                               Cn3 * -3131161143.2600408 +
                               Cn4 * 36091829348.23567) +
                      pbar1 * (      86475.41311508231 +
                               Cn1 * -12434785.139451396 +
                               Cn2 * 86248072.31458677 +
                               Cn3 * 48496597094.53321 +
                               Cn4 * -2445973404734.065) +
                      pbar2 * (      -650441.1318037243 +
                               Cn1 * 255693544.05770853 +
                               Cn2 * -16592241499.525118 +
                               Cn3 * -760714934022.2084 +
                               Cn4 * 50372349004519.18) +
                      pbar3 * (      -2600563.816539052 +
                               Cn1 * 1990542352.9060328 +
                               Cn2 * -158372960093.63406 +
                               Cn3 * -7016014777762.865 +
                               Cn4 * 718049768377411.0) +
                      pbar4 * (      93435073.1558145 +
                               Cn1 * -46657621522.19617 +
                               Cn2 * 2311457358899.069 +
                               Cn3 * 168585362157391.12 +
                               Cn4 * -7409681979474361.0)) +
               Cm2 * (        (      -476.63246482593433 +
                               Cn1 * 8398833.063460486 +
                               Cn2 * -683088639.8509223 +
                               Cn3 * -27969192160.597004 +
                               Cn4 * 2028165606696.3384) +
                      pbar1 * (      686768.3259658789 +
                               Cn1 * 42718350.67506598 +
                               Cn2 * -18711773820.25494 +
                               Cn3 * 199506687180.44263 +
                               Cn4 * 39453787476878.07) +
                      pbar2 * (      -7100202.852349944 +
                               Cn1 * -4662448430.723727 +
                               Cn2 * 418386710304.7471 +
                               Cn3 * 13613648976470.43 +
                               Cn4 * -973521154638415.4) +
                      pbar3 * (      24008136.262420394 +
                               Cn1 * 8520850548.409288 +
                               Cn2 * 801012120685.1766 +
                               Cn3 * -18022313899335.05 +
                               Cn4 * -3157514410557812.5) +
                      pbar4 * (      -521319584.3862735 +
                               Cn1 * 388624762099.7519 +
                               Cn2 * -34921002282450.766 +
                               Cn3 * -1309895813370060.0 +
                               Cn4 * 9.307826208869104e+16)) +
               Cm3 * (        (      1920625.4246594235 +
                               Cn1 * -282998460.05167794 +
                               Cn2 * -2596670579.5672493 +
                               Cn3 * 663612283570.8875 +
                               Cn4 * 4058594790455.9863) +
                      pbar1 * (      -22393000.022612143 +
                               Cn1 * 4315431233.918542 +
                               Cn2 * -151083213306.52487 +
                               Cn3 * -10421658861816.459 +
                               Cn4 * 757851510925086.8) +
                      pbar2 * (      426641556.7765975 +
                               Cn1 * -105231022169.37967 +
                               Cn2 * 6382034614269.277 +
                               Cn3 * 320342119547869.0 +
                               Cn4 * -1.8811321644862956e+16) +
                      pbar3 * (      -87635233.24664861 +
                               Cn1 * -684850718222.9294 +
                               Cn2 * 51316476245963.04 +
                               Cn3 * 1696941788662424.8 +
                               Cn4 * -1.9673009028702963e+17) +
                      pbar4 * (      -48501771775.92702 +
                               Cn1 * 18170442146590.73 +
                               Cn2 * -831006687564071.2 +
                               Cn3 * -6.097976922315267e+16 +
                               Cn4 * 2.4595509719090954e+18)) +
               Cm4 * (        (      1559895.5442895538 +
                               Cn1 * -1114565271.6191757 +
                               Cn2 * 76886248039.08226 +
                               Cn3 * 3326691906519.008 +
                               Cn4 * -224708429965579.16) +
                      pbar1 * (      -41043472.278236434 +
                               Cn1 * -2922777160.1120367 +
                               Cn2 * 1896880396621.7458 +
                               Cn3 * -35398869085460.32 +
                               Cn4 * -3430104744756242.0) +
                      pbar2 * (      223862604.99936724 +
                               Cn1 * 333229832743.9814 +
                               Cn2 * -39062230100250.305 +
                               Cn3 * -783151654875299.0 +
                               Cn4 * 8.192444666517482e+16) +
                      pbar3 * (      -1778274014.2296038 +
                               Cn1 * -1036487769363.6792 +
                               Cn2 * -89496899347658.31 +
                               Cn3 * 4794481466294519.0 +
                               Cn4 * 2.2185050690236774e+17) +
                      pbar4 * (      74707739897.20567 +
                               Cn1 * -23941143481566.246 +
                               Cn2 * 3019865996283524.0 +
                               Cn3 * 5.8448147918718456e+16 +
                               Cn4 * -6.490288740441779e+18)) +
               Cm5 * (        (      -115794070.02785304 +
                               Cn1 * 15271975879.00638 +
                               Cn2 * 400231708287.3226 +
                               Cn3 * -32456226658167.7 +
                               Cn4 * -1017761739777621.4) +
                      pbar1 * (      1475055315.2422502 +
                               Cn1 * -362336316348.3438 +
                               Cn2 * 13421609434086.553 +
                               Cn3 * 621301015882172.6 +
                               Cn4 * -4.49835753821639e+16) +
                      pbar2 * (      -33253019017.493908 +
                               Cn1 * 8440557568692.741 +
                               Cn2 * -461306433666649.2 +
                               Cn3 * -2.3075321736216304e+16 +
                               Cn4 * 1.2193164891545953e+18) +
                      pbar3 * (      34590365382.39004 +
                               Cn1 * 51363916407008.04 +
                               Cn2 * -3589923572811883.5 +
                               Cn3 * -1.0093439903477272e+17 +
                               Cn4 * 1.1917722389503732e+19) +
                      pbar4 * (      3686103451384.133 +
                               Cn1 * -1377832768225128.2 +
                               Cn2 * 5.725739933204989e+16 +
                               Cn3 * 4.2097025895828275e+18 +
                               Cn4 * -1.515643563658013e+20))) +
        CL3 * (      (        (      11.106525606128724 +
                               Cn1 * 3235.2186098382294 +
                               Cn2 * -526415.4578021283 +
                               Cn3 * -1932434.0263809378 +
                               Cn4 * 817976267.0149103) +
                      pbar1 * (      1326.1356108638695 +
                               Cn1 * 35503.28203538715 +
                               Cn2 * -23620046.024081845 +
                               Cn3 * -327696008.40546435 +
                               Cn4 * 79449805686.7926) +
                      pbar2 * (      -12724.959712276835 +
                               Cn1 * -4423384.171107621 +
                               Cn2 * 432373034.1181964 +
                               Cn3 * 14285206437.702888 +
                               Cn4 * -871565869444.8282) +
                      pbar3 * (      65290.62243915188 +
                               Cn1 * -7803106.741938741 +
                               Cn2 * 425819177.8090187 +
                               Cn3 * 134962852396.5369 +
                               Cn4 * -6752983432066.443) +
                      pbar4 * (      -460553.28272127797 +
                               Cn1 * 393810860.60607713 +
                               Cn2 * -33061812788.499546 +
                               Cn3 * -1733792371749.5295 +
                               Cn4 * 116996517831758.9)) +
               Cm1 * (        (      4855.078423190195 +
                               Cn1 * -765421.024920078 +
                               Cn2 * -660123.3311443847 +
                               Cn3 * 2209814512.0587482 +
                               Cn4 * -22614290727.168076) +
                      pbar1 * (      -65352.215502909195 +
                               Cn1 * 9493890.411541514 +
                               Cn2 * -114888646.15410204 +
                               Cn3 * -32512039299.858204 +
                               Cn4 * 1670974090444.41) +
                      pbar2 * (      490268.0880062761 +
                               Cn1 * -151051662.37392646 +
                               Cn2 * 11044270676.804468 +
                               Cn3 * 399445987033.1612 +
                               Cn4 * -31872775427002.543) +
                      pbar3 * (      1905198.927332055 +
                               Cn1 * -1464340057.8766763 +
                               Cn2 * 118227931355.9142 +
                               Cn3 * 4749880652985.519 +
                               Cn4 * -500134117486992.9) +
                      pbar4 * (      -62450343.06675269 +
                               Cn1 * 29451972719.20835 +
                               Cn2 * -1549857319079.3164 +
                               Cn3 * -102893082990634.16 +
                               Cn4 * 4767819152847117.0)) +
               Cm2 * (        (      11785.002918877664 +
                               Cn1 * -8038547.370769252 +
                               Cn2 * 516916001.2630029 +
                               Cn3 * 23644294564.871193 +
                               Cn4 * -1503616451360.0632) +
                      pbar1 * (      -689335.2048439214 +
                               Cn1 * -18719956.11997008 +
                               Cn2 * 15763579387.459055 +
                               Cn3 * -123705914222.03941 +
                               Cn4 * -35792238809959.37) +
                      pbar2 * (      5752071.3927846495 +
                               Cn1 * 3219243439.727787 +
                               Cn2 * -307093579853.41364 +
                               Cn3 * -9457334073276.697 +
                               Cn4 * 711120415491411.4) +
                      pbar3 * (      -5758649.725415029 +
                               Cn1 * -4921108332.342276 +
                               Cn2 * -832455950722.8717 +
                               Cn3 * -5053147272979.027 +
                               Cn4 * 3636870626953580.5) +
                      pbar4 * (      228323557.0309282 +
                               Cn1 * -270199535427.30966 +
                               Cn2 * 26749824537609.63 +
                               Cn3 * 1005883345193238.8 +
                               Cn4 * -7.597459818686704e+16)) +
               Cm3 * (        (      -1301676.2388656796 +
                               Cn1 * 186429672.17843947 +
                               Cn2 * 2935930478.2894964 +
                               Cn3 * -452235130237.3692 +
                               Cn4 * -5206792343184.337) +
                      pbar1 * (      17133332.629013404 +
                               Cn1 * -3400093877.4445825 +
                               Cn2 * 113959480980.67502 +
                               Cn3 * 7778390364179.625 +
                               Cn4 * -527893220305748.3) +
                      pbar2 * (      -284507456.2032094 +
                               Cn1 * 65991909711.22778 +
                               Cn2 * -4481586039446.462 +
                               Cn3 * -185825984730077.47 +
                               Cn4 * 1.2805223299277246e+16) +
                      pbar3 * (      12183906.225893425 +
                               Cn1 * 524123010381.03265 +
                               Cn2 * -38278117514152.69 +
                               Cn3 * -1301144106436602.0 +
                               Cn4 * 1.437807780820045e+17) +
                      pbar4 * (      30652768257.63511 +
                               Cn1 * -11847359792203.113 +
                               Cn2 * 581101616489099.5 +
                               Cn3 * 3.906452729628201e+16 +
                               Cn4 * -1.6912821931482056e+18)) +
               Cm4 * (        (      -1826923.2954044745 +
                               Cn1 * 919722380.9120979 +
                               Cn2 * -57393781471.7367 +
                               Cn3 * -2534083365605.6934 +
                               Cn4 * 163310270205576.12) +
                      pbar1 * (      47394247.17302025 +
                               Cn1 * 949314611.5203906 +
                               Cn2 * -1503975833710.7776 +
                               Cn3 * 22035744463982.66 +
                               Cn4 * 3100334467378848.5) +
                      pbar2 * (      -270773197.42817956 +
                               Cn1 * -236528813079.8105 +
                               Cn2 * 29077447630034.84 +
                               Cn3 * 587003755096233.6 +
                               Cn4 * -6.201077615129488e+16) +
                      pbar3 * (      251811201.15637037 +
                               Cn1 * 623142791842.5681 +
                               Cn2 * 83365311919019.06 +
                               Cn3 * -1602549498585366.0 +
                               Cn4 * -2.884926866842297e+17) +
                      pbar4 * (      -36170656677.54194 +
                               Cn1 * 17801992936182.68 +
                               Cn2 * -2377914949313378.5 +
                               Cn3 * -5.618551464124028e+16 +
                               Cn4 * 5.776577118611342e+18)) +
               Cm5 * (        (      78392072.6410813 +
                               Cn1 * -9817512079.467916 +
                               Cn2 * -372601174061.9458 +
                               Cn3 * 22186516384501.33 +
                               Cn4 * 899590394261581.5) +
                      pbar1 * (      -1119192305.2666655 +
                               Cn1 * 275577337562.492 +
                               Cn2 * -9791628610355.281 +
                               Cn3 * -486692977332225.2 +
                               Cn4 * 3.2437561788706884e+16) +
                      pbar2 * (      21771115268.974915 +
                               Cn1 * -5418570578322.236 +
                               Cn2 * 335767302209299.8 +
                               Cn3 * 1.3738794689432794e+16 +
                               Cn4 * -8.73057743761281e+17) +
                      pbar3 * (      -20607763835.25709 +
                               Cn1 * -39287125107941.89 +
                               Cn2 * 2694563869850619.0 +
                               Cn3 * 8.433721078921571e+16 +
                               Cn4 * -9.096243295919425e+18) +
                      pbar4 * (      -2299682881348.141 +
                               Cn1 * 911160971278688.2 +
                               Cn2 * -4.143964155507519e+16 +
                               Cn3 * -2.7601610409217423e+18 +
                               Cn4 * 1.1068100008694156e+20))));
    
    // set center control surface
    lr[5] = bounds(C);
    // loop thru inboard to outboard control surfaces
    for (i=0; i<5; i++) {
        // set left control surface
        lr[4-i] = bounds( L[i] );
        // set right control surface
        lr[6+i] = bounds( R[i] );
    }
}



#endif


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

#endif


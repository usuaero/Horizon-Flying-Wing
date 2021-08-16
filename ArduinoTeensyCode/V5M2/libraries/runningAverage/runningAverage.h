/*
 * runningAverage.h - Library for including Zachs yaw controller code
 * Created by Zach Montgomery, Sept 20 2019
*/

#ifndef runningAverage_h
    
    #define runningAverage_h
    
    class runAvg {
        
        public:
            runAvg(int length, double initialValue);
            double update(double val);
            double getAverage();
            
        private:
            
            int _z, _l;      // z= moving window, l=length of a
            double *_a;   // array of the running average
            
        
    };
    
    
#endif



#include "runningAverage.h"

runAvg::runAvg(int length, double initialValue) {
    int i;
    _l = length;
    _a = new double[_l];
    for (i=0; i<_l; i++) {
        _a[i] = initialValue;
    }
    _z = 0;
}

double runAvg::update(double value) {
    _a[_z] = value;
    _z += 1;
    if (_z >= _l) _z = 0;
    return getAverage();
}

double runAvg::getAverage() {
    double sum = 0;
    int i;
    for (i=0; i<_l; i++) {
        sum += _a[i];
    }
    return sum / (double) _l;
}


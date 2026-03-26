#ifndef LOWPASSFILTER
#define LOWPASSFILTER
#include <math.h>

class LPFilter{
public:
    LPFilter(double samplePeriod, double cutFrequency);
    ~LPFilter();
    void addValue(double newValue);
    double getValue();
    void clear();
private:
    double _weight;
    double _pastValue;
    bool _start;
};

LPFilter::LPFilter(double samplePeriod, double cutFrequency){
    _weight = 1.0 / ( 1.0 + 1.0/(2.0*M_PI * samplePeriod * cutFrequency) );
    _start  = false;
}

void LPFilter::addValue(double newValue){
    if(!_start){
        _start = true;
        _pastValue = newValue;
    }
    _pastValue = _weight*newValue + (1-_weight)*_pastValue;
}

double LPFilter::getValue(){
    return _pastValue;
}

void LPFilter::clear(){
    _start = false;
}
#endif  // LOWPASSFILTER
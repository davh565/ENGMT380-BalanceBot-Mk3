#ifndef BB_UTILITY_H
#define BB_UTILITY

double scale(double var,
            double oldMax, 
            double oldMin = 0, 
            double newMax=255, 
            double newMin=0);

double mix(double input1, double input2, double bias);
#endif

//Convert deg,rpm, etc. to PID/PWM range
double scale(double var,
            double oldMax, 
            double oldMin = 0, 
            double newMax=255, 
            double newMin=0) {
                double output = (var-oldMin)/(oldMax-oldMin)*(newMax-newMin)+newMin;
                if (output > newMax) output = newMax; 
                if (output < newMin) output = newMin; 
            return output;
}

double mix(double input1, double input2, double bias){
    return (1-bias)*input1+bias*input2;
}

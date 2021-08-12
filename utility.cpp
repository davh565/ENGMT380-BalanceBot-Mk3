//Convert deg,rpm, etc. to PID/PWM range
double scale(double var,
            double oldMax, 
            double oldMin = 0, 
            double newMax=255, 
            double newMin=0) {
            return (var-oldMin)/(oldMax-oldMin)*(newMax-newMin)+newMin;
}

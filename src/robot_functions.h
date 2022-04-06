double servoMap(double x, double in_min, double in_max, double out_min, double out_max) {
    double calc = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (calc < 15) {
        calc = 15;
    }
    return calc;
}

double stepperMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double deadzoneMap(double x, double in_min, double in_max, double out_min, double out_max) {
    if (x > 450 && x < 550) {
        return 0;
    }

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
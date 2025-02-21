float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Scale the value from input range to output range
    float scaledValue = (value - inputMin) / (inputMax - inputMin);
    return outputMin + scaledValue * (outputMax - outputMin);
}


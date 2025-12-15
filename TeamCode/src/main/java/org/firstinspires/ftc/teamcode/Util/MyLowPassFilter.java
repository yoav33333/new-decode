package org.firstinspires.ftc.teamcode.Util;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.Filter;

public class MyLowPassFilter implements Filter {

    public double gain;

    protected double previousEstimate = 0;

    /**
     * gain of the low pass filter.
     *
     * (0 < x < 1)
     *
     * High values of A are smoother but have more phase lag, low values of A allow more noise but
     * will respond faster to quick changes in the measured state.
     *
     *
     * @param gain Aforementioned Gain. (0 < x < 1)
     */
    public MyLowPassFilter(double gain) {
        this.gain = gain;
    }

    /**
     * Low Pass Filter estimate
     * @param measurement current measurement
     * @return filtered value
     */
    @Override
    public double estimate(double measurement) {
        double estimate = gain * previousEstimate + (1-gain) * measurement;
        previousEstimate = estimate;
        return estimate;
    }

}

package org.firstinspires.ftc.teamcode.Util;

/**
 * Simple 1D Kalman filter for estimating odometry drift.
 *
 * This filter estimates only a single variable: the drift (bias) affecting odometry.
 * Prediction increases uncertainty, and vision/absolute measurements correct the drift.
 */
public class DriftKalmanFilter {

    /**
     * Current drift estimate (the state).
     */
    private double driftEstimate;

    /**
     * Variance (uncertainty) of the drift estimate.
     */
    private double P;

    /**
     * Process noise (variance) for the drift. Typically small.
     */
    private final double processNoise;

    /**
     * Constructor.
     *
     * @param initialDriftEstimate Initial guess of the drift (often 0).
     * @param initialUncertainty   Initial variance for the drift estimate (large if unknown).
     * @param processNoise         Process noise variance for the drift model.
     */
    public DriftKalmanFilter(double initialDriftEstimate, double initialUncertainty, double processNoise) {
        this.driftEstimate = initialDriftEstimate;
        this.P = initialUncertainty;
        this.processNoise = processNoise;
    }

    /**
     * Predict step: propagate uncertainty forward.
     *
     * In this simplified model, the drift state itself does not change —
     * only its uncertainty increases.
     *
     * @param deltaTime Time since last call (seconds or milliseconds depending on unit).
     */
    public void predict(double deltaTime) {
        // State stays the same:
        // driftEstimate = driftEstimate

        // Uncertainty grows with process noise
        P = P + processNoise * deltaTime;
    }

    /**
     * Update step: correct the drift estimate using a measurement.
     *
     * @param measuredDrift     The observed drift from sensor fusion (e.g., odometry - vision).
     * @param measurementNoise  The variance (uncertainty) of that measurement.
     */
    public void update(double measuredDrift, double measurementNoise) {
        // Kalman gain
        double K = P / (P + measurementNoise);

        // Update estimate
        driftEstimate = driftEstimate + K * (measuredDrift - driftEstimate);

        // Update uncertainty
        P = (1.0 - K) * P;
    }

    /**
     * Get the current drift estimate.
     *
     * @return Estimated drift (bias term).
     */
    public double getDriftEstimate() {
        return driftEstimate;
    }

    /**
     * Get the current uncertainty (variance) of the drift estimate.
     *
     * @return Variance of the drift estimate.
     */
    public double getUncertainty() {
        return P;
    }

    /**
     * Reset filter to new initial values.
     *
     * @param initialDrift   New initial drift estimate.
     * @param initialVariance New initial uncertainty.
     */
    public void reset(double initialDrift, double initialVariance) {
        this.driftEstimate = initialDrift;
        this.P = initialVariance;
    }
}
//DriftKalmanFilter driftFilter = new DriftKalmanFilter(
//        0.0,       // initial drift
//        1000.0,    // large initial uncertainty
//        0.01       // small process noise
//);
//
//double lastTime = getRuntime();
//
//while (opModeIsActive()) {
//double now = getRuntime();
//double dt = now - lastTime;
//lastTime = now;
//
//// 1) Predict — advance uncertainty
//    driftFilter.predict(dt);
//
//// 2) Optional: compute "measured drift" when vision is present
//    if (visionHasMeasurement()) {
//double measuredDrift = computeOdometryDriftFromVision();
//double visionNoiseVariance = estimateVisionNoise();
//
//// 3) Correct drift
//        driftFilter.update(measuredDrift, visionNoiseVariance);
//    }
//
//double bias = driftFilter.getDriftEstimate();
//// Use bias to correct odometry, path following, etc.
//}

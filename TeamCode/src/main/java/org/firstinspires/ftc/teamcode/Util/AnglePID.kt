package org.firstinspires.ftc.teamcode.Util

import kotlin.math.abs
import kotlin.math.min

class AnglePID @JvmOverloads constructor(    var p: Double,
                                             var i: Double,
                                             var d: Double,
                                             var f: Double,
                                             private var setPoint: Double = 0.0,
                                             private var measuredValue: Double = 0.0
) {
    private var minIntegral: Double = -1.0
    private var maxIntegral = 1.0

    /**
     * @return the positional error e(t)
     */
    var positionError: Double = 0.0
        private set

    /**
     * @return the velocity error e'(t)
     */
    var velocityError: Double = 0.0
        private set

    private var totalError = 0.0
    private var prevErrorVal = 0.0

    private var errorTolerance_p = 0.05
    private var errorTolerance_v = Double.POSITIVE_INFINITY

    private var lastTimeStamp = 0.0
    var period: Double = 0.0
        private set

    init {
        // Initialize positionError correctly on creation
        positionError = getContinuousError(setPoint - measuredValue)
        reset()
    }

    fun reset() {
        totalError = 0.0
        prevErrorVal = positionError // Start with the current error
        lastTimeStamp = 0.0
        velocityError = 0.0
    }

    /**
     * Wraps an angle error to the range [-180, 180].
     *
     * @param error The angle error.
     * @return The normalized angle error.
     */
    private fun getContinuousError(error: Double): Double {
        var cappedError = error % 360.0
        if (cappedError > 180.0) {
            cappedError -= 360.0
        } else if (cappedError <= -180.0) {
            cappedError += 360.0
        }
        return cappedError
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with [.atSetPoint].
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        errorTolerance_p = positionTolerance
        errorTolerance_v = velocityTolerance
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    fun getSetPoint(): Double {
        return setPoint
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    fun setTargetPosition(sp: Double) {
        setPoint = sp
        positionError = getContinuousError(setPoint - measuredValue)
        velocityError = if (abs(period) > 1E-6) {
            (positionError - prevErrorVal) / period
        } else {
            0.0
        }
    }

    /**
     * Returns true if the error is within the acceptable bounds.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    fun atSetPoint(): Boolean {
        return abs(positionError) < errorTolerance_p && abs(velocityError) < errorTolerance_v
    }

    /**
     * @return the PIDF coefficients
     */
    val coefficients: DoubleArray
        get() = doubleArrayOf(p, i, d, f)

    /**
     * @return the tolerances of the controller
     */
    val tolerance: DoubleArray
        get() = doubleArrayOf(errorTolerance_p, errorTolerance_v)

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output.
     */
    fun calculate(pv: Double, sp: Double): Double {
        setTargetPosition(sp)
        return calculate(pv)
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    @JvmOverloads
    fun calculate(pv: Double = measuredValue): Double {
        prevErrorVal = positionError

        val currentTimeStamp = System.nanoTime().toDouble() / 1E9
        if (lastTimeStamp == 0.0) lastTimeStamp = currentTimeStamp
        period = currentTimeStamp - lastTimeStamp
        lastTimeStamp = currentTimeStamp

        measuredValue = pv
        positionError = getContinuousError(setPoint - measuredValue)

        velocityError = if (abs(period) > 1E-6) {
            (positionError - prevErrorVal) / period
        } else {
            0.0
        }

        totalError += period * positionError
        totalError = totalError.coerceIn(minIntegral, maxIntegral)

        // Feed-forward is typically applied to the setpoint, not just multiplied
        val feedForward = if (abs(setPoint) > 1E-6) f * setPoint else 0.0

        // returns u(t)
        return (p * positionError) + (i * totalError) + (d * velocityError) + feedForward
    }

    fun setPIDF(kp: Double, ki: Double, kd: Double, kf: Double) {
        p = kp
        i = ki
        d = kd
        f = kf
    }

    fun setIntegrationBounds(integralMin: Double, integralMax: Double) {
        minIntegral = integralMin
        maxIntegral = integralMax
    }

    fun clearTotalError() {
        totalError = 0.0
    }
}

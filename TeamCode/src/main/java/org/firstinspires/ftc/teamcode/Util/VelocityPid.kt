package org.firstinspires.ftc.teamcode.Util

import kotlin.math.abs

class VelocityPid(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
) {
    var targetVelocity: Double = 0.0
    private var measuredVelocity = 0.0

    var error: Double = 0.0
        private set
    private var prevError = 0.0
    var derivative: Double = 0.0
        private set
    var integral: Double = 0.0
        private set

    private var minIntegral = -1.0
    private var maxIntegral = 1.0

    private var velocityTolerance = 0.0

    private var lastTimestamp = 0.0
    var period: Double = 0.0
        private set

    fun reset() {
        error = 0.0
        prevError = 0.0
        integral = 0.0
        derivative = 0.0
        lastTimestamp = 0.0
    }

    fun setTolerance(velTol: Double) {
        velocityTolerance = velTol
    }

    fun atSetpoint(): Boolean {
        return abs(error) < velocityTolerance
    }

    fun setIntegrationBounds(min: Double, max: Double) {
        minIntegral = min
        maxIntegral = max
    }

    fun calculate(measuredV: Double): Double {
        prevError = error
        measuredVelocity = measuredV

        val timestamp = System.nanoTime() / 1e9
        if (lastTimestamp == 0.0) lastTimestamp = timestamp
        period = timestamp - lastTimestamp
        lastTimestamp = timestamp

        error = targetVelocity - measuredVelocity

        if (period > 1e-6) {
            derivative = (error - prevError) / period
        } else {
            derivative = 0.0
        }

        integral += error * period
        if (integral > maxIntegral) integral = maxIntegral
        if (integral < minIntegral) integral = minIntegral

        return kP * error + kI * integral + kD * derivative
    }
    fun setPID(kP: Double, kI: Double, kD: Double) {
        kP.let { this.kP = it }
        kI.let { this.kI = it }
        kD.let { this.kD = it }
    }
}

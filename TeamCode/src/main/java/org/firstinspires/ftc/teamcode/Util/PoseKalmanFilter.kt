package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import java.util.TreeMap
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Kalman Filter for fusing dead wheel odometry with AprilTag vision.
 * Features:
 * 1. Delta-based prediction (prevents fighting raw odometry)
 * 2. Latency compensation (TreeMap buffer for historical look-back)
 * 3. Dynamic noise updates
 */
class PoseKalmanFilter(
    initialPose: Pose,
    // Process noise - trust in dead wheels (inches and radians)
    private var processNoiseX: Double = 0.01,
    private var processNoiseY: Double = 0.01,
    private var processNoiseTheta: Double = 0.005,
    // Measurement noise - uncertainty in AprilTag (inches and radians)
    private var measurementNoiseX: Double = 0.5,
    private var measurementNoiseY: Double = 0.5,
    private var measurementNoiseTheta: Double = 0.02
) {
    // State estimate [x, y, theta]
    private var state = doubleArrayOf(initialPose.x, initialPose.y, initialPose.heading)

    // Track previous odometry to calculate deltas
    private var lastDeadWheelPose: Pose? = null

    // Error covariance matrix (3x3)
    private var P = Array(3) { i ->
        DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 }
    }

    // Buffer to store history: Key = Timestamp (ms), Value = Odometry Pose
    private val poseHistory = TreeMap<Long, Pose>()
    private val MAX_HISTORY_MS = 500 // Store 500ms of history

    // Process noise covariance (Q)
    private val Q = Array(3) { DoubleArray(3) }

    // Measurement noise covariance (R)
    private val R = Array(3) { DoubleArray(3) }

    init {
        updateProcessNoise(processNoiseX, processNoiseY, processNoiseTheta)
        updateMeasurementNoise(measurementNoiseX, measurementNoiseY, measurementNoiseTheta)
    }

    /**
     * Prediction step using dead wheel odometry.
     * @param currentOdoPose The current pose from the localizer
     * @param timestamp The current system time in milliseconds
     */
    fun predict(currentOdoPose: Pose, timestamp: Long) {
        if (lastDeadWheelPose == null) {
            lastDeadWheelPose = currentOdoPose
        }

        // 1. Calculate the Delta (Change in Odometry)
        val deltaX = currentOdoPose.x - lastDeadWheelPose!!.x
        val deltaY = currentOdoPose.y - lastDeadWheelPose!!.y
        val deltaTheta = normalizeAngle(currentOdoPose.heading - lastDeadWheelPose!!.heading)

        // 2. Apply Delta to our filtered state
        state[0] += deltaX
        state[1] += deltaY
        state[2] = normalizeAngle(state[2] + deltaTheta)

        // 3. Update error covariance: P = P + Q
        for (i in 0..2) {
            P[i][i] += Q[i][i]
        }

        lastDeadWheelPose = currentOdoPose

        // 4. Save this pose to history for future latency compensation
        poseHistory[timestamp] = currentOdoPose

        // Prune old entries
        while (poseHistory.size > 0 && timestamp - poseHistory.firstKey() > MAX_HISTORY_MS) {
            poseHistory.pollFirstEntry()
        }
    }

    /**
     * Update step with Latency Compensation.
     * @param visionPose The pose reported by Limelight/Vision
     * @param captureTimestamp The timestamp when the image was actually taken
     */
    fun updateWithLatency(visionPose: Pose, captureTimestamp: Long) {
        // Find the odometry pose closest to when the image was captured
        val historicalEntry = poseHistory.floorEntry(captureTimestamp) ?: return
        val historicalOdo = historicalEntry.value

        // Innovation (Residual): Difference between Vision and Odometry at capture time
        val z = doubleArrayOf(visionPose.x, visionPose.y, normalizeAngle(visionPose.heading))
        val h = doubleArrayOf(historicalOdo.x, historicalOdo.y, historicalOdo.heading)

        val y = DoubleArray(3) { i ->
            if (i == 2) normalizeAngle(z[i] - h[i]) else z[i] - h[i]
        }

        // Innovation covariance: S = P + R
        val S = Array(3) { i ->
            DoubleArray(3) { j -> P[i][j] + R[i][j] }
        }

        // Kalman gain: K = P * S^-1
        val K = matrixMultiply(P, matrixInverse3x3(S))

        // Update CURRENT state estimate using the historical error
        for (i in 0..2) {
            for (j in 0..2) {
                state[i] += K[i][j] * y[j]
            }
        }
        state[2] = normalizeAngle(state[2])

        // Update error covariance: P = (I - K) * P
        val I = Array(3) { i -> DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 } }
        val IminusK = Array(3) { i ->
            DoubleArray(3) { j -> I[i][j] - K[i][j] }
        }
        P = matrixMultiply(IminusK, P)
    }

    fun updateProcessNoise(x: Double, y: Double, theta: Double) {
        Q[0][0] = x; Q[1][1] = y; Q[2][2] = theta
    }

    fun updateMeasurementNoise(x: Double, y: Double, theta: Double) {
        R[0][0] = x; R[1][1] = y; R[2][2] = theta
    }

    fun getPose(): Pose = Pose(state[0], state[1], state[2])

    fun getUncertainty(): Pose = Pose(sqrt(P[0][0]), sqrt(P[1][1]), sqrt(P[2][2]))

    fun reset(newPose: Pose) {
        state[0] = newPose.x
        state[1] = newPose.y
        state[2] = normalizeAngle(newPose.heading)
        lastDeadWheelPose = null
        poseHistory.clear()
        P = Array(3) { i -> DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 } }
    }

    // ==================== Matrix Math Helpers ====================

    private fun normalizeAngle(angle: Double): Double {
        var a = angle
        while (a > PI) a -= 2 * PI
        while (a < -PI) a += 2 * PI
        return a
    }

    private fun matrixMultiply(A: Array<DoubleArray>, B: Array<DoubleArray>): Array<DoubleArray> {
        val result = Array(3) { DoubleArray(3) }
        for (i in 0..2) {
            for (j in 0..2) {
                for (k in 0..2) result[i][j] += A[i][k] * B[k][j]
            }
        }
        return result
    }

    private fun matrixInverse3x3(m: Array<DoubleArray>): Array<DoubleArray> {
        val det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

        if (abs(det) < 1e-10) return Array(3) { i -> DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 } }

        val inv = Array(3) { DoubleArray(3) }
        inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det
        inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det
        inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det
        inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det
        inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det
        inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det
        inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det
        inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det
        inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det
        return inv
    }
}
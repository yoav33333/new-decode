package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Kalman Filter for fusing dead wheel odometry with AprilTag vision
 *
 * Usage:
 * ```
 * val filter = KalmanFilter(
 *     initialPose = Pose(0.0, 0.0, 0.0),
 *     processNoiseX = 1.0,        // mm - tune based on dead wheel accuracy
 *     processNoiseY = 1.0,        // mm
 *     processNoiseTheta = 0.005,  // radians (~0.3 degrees)
 *     measurementNoiseX = 10.0,   // mm - tune based on AprilTag accuracy
 *     measurementNoiseY = 10.0,   // mm
 *     measurementNoiseTheta = 0.02 // radians (~1 degree)
 * )
 *
 * // In your control loop:
 * fun loop() {
 *     // Get current pose from dead wheels
 *     val deadWheelPose = getDeadWheelPose() // your odometry calculation
 *     filter.predict(deadWheelPose)
 *
 *     // Update with AprilTag when available
 *     val aprilTagPose = getAprilTagPose() // may be null
 *     if (aprilTagPose != null && !filter.isOutlier(aprilTagPose)) {
 *         filter.update(aprilTagPose)
 *     }
 *
 *     // Get fused pose - use this as your robot's position
 *     val robotPose = filter.getPose()
 * }
 * ```
 */
class PoseKalmanFilter(
    initialPose: Pose,
    // Process noise - how much we trust the dead wheels (in mm and radians)
    public var processNoiseX: Double = 1.0,
    public var processNoiseY: Double = 1.0,
    public var processNoiseTheta: Double = 0.005,
    // Measurement noise - uncertainty in AprilTag measurements (in mm and radians)
    public var measurementNoiseX: Double = 10.0,
    public var measurementNoiseY: Double = 10.0,
    public var measurementNoiseTheta: Double = 0.02
) {
    // State estimate [x, y, theta]
    private var state = doubleArrayOf(initialPose.x, initialPose.y, initialPose.heading)

    // Error covariance matrix (3x3) - starts with high uncertainty
    private var P = Array(3) { i ->
        DoubleArray(3) { j ->
            if (i == j) 1.0 else 0.0
        }
    }

    // Process noise covariance
    private val Q = Array(3) { i ->
        DoubleArray(3) { j ->
            when {
                i == j && i == 0 -> processNoiseX
                i == j && i == 1 -> processNoiseY
                i == j && i == 2 -> processNoiseTheta
                else -> 0.0
            }
        }
    }

    // Measurement noise covariance
    private val R = Array(3) { i ->
        DoubleArray(3) { j ->
            when {
                i == j && i == 0 -> measurementNoiseX
                i == j && i == 1 -> measurementNoiseY
                i == j && i == 2 -> measurementNoiseTheta
                else -> 0.0
            }
        }
    }

    /**
     * Prediction step using dead wheel odometry pose
     * Call this every loop iteration with the current pose from dead wheels
     *
     * @param deadWheelPose The current pose from your dead wheel odometry
     */
    fun predict(deadWheelPose: Pose) {
        // Update state to dead wheel pose
        state[0] = deadWheelPose.x
        state[1] = deadWheelPose.y
        state[2] = normalizeAngle(deadWheelPose.heading)

        // Update error covariance: P = P + Q
        // This increases uncertainty over time without vision corrections
        for (i in 0..2) {
            for (j in 0..2) {
                P[i][j] += Q[i][j]
            }
        }
    }

    /**
     * Update step using AprilTag vision measurement
     * Call this when you get a valid AprilTag pose measurement
     *
     * @param measurement The pose from AprilTag detection
     * @param measurementQuality Optional quality factor (0-1), lower = less trust
     */
    fun update(measurement: Pose, measurementQuality: DoubleArray =
        doubleArrayOf(measurementNoiseX, measurementNoiseY, measurementNoiseTheta)) {
        // Measurement vector
        val z = doubleArrayOf(measurement.x, measurement.y, normalizeAngle(measurement.heading))

        // Innovation (measurement residual)
        val y = DoubleArray(3) { i ->
            if (i == 2) {
                // Handle angle wraparound
                normalizeAngle(z[i] - state[i])
            } else {
                z[i] - state[i]
            }
        }
//        val scaledR = Array(3) { i ->
//            DoubleArray(3) { j ->
//                when {
//                    i == j && i == 0 -> measurementQuality[0]
//                    i == j && i == 1 -> measurementQuality[1]
//                    i == j && i == 2 -> measurementQuality[2]
//                    else -> 0.0
//                }
//            }
//        }
//
//        // Scale measurement noise by quality (lower quality = more noise)
//        val scaledR = Array(3) { i ->
//            DoubleArray(3) { j ->
//                R[i][j] / measurementQuality
//            }
//        }

        // Innovation covariance: S = P + R
        val S = Array(3) { i ->
            DoubleArray(3) { j ->
                P[i][j] + R[i][j]
            }
        }

        // Kalman gain: K = P * S^-1
        val K = matrixMultiply(P, matrixInverse3x3(S))

        // Update state estimate: state = state + K * y
        for (i in 0..2) {
            for (j in 0..2) {
                state[i] += K[i][j] * y[j]
            }
        }

        // Normalize angle
        state[2] = normalizeAngle(state[2])

        // Update error covariance: P = (I - K) * P
        val I = Array(3) { i -> DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 } }
        val IminusK = Array(3) { i ->
            DoubleArray(3) { j ->
                I[i][j] - K[i][j]
            }
        }
        P = matrixMultiply(IminusK, P)
    }

    /**
     * Get the current fused pose estimate
     * This is your robot's best estimate of its position
     */
    fun getPose(): Pose {
        return Pose(state[0], state[1], state[2])
    }

    /**
     * Get the current uncertainty (standard deviations in mm and radians)
     * Useful for debugging or adaptive behavior
     */
    fun getUncertainty(): Pose {
        return Pose(sqrt(P[0][0]), sqrt(P[1][1]), sqrt(P[2][2]))
    }

    fun isOutlier(base:Pose, measurement: Pose, threshold: Double = 15.0): Boolean {
//        val diff = Pose(
//            abs(measurement.x - state[0]),
//            abs(measurement.y - state[1]),
//            abs(normalizeAngle(measurement.heading - state[2]))
//        )
//
//        val uncertainty = getUncertainty()

        // Check if difference is more than threshold * standard deviation
        return Vector(measurement.x-base.x,
            measurement.y-base.y)
            .magnitude > threshold
    }

    /**
     * Reset the filter to a new pose
     * Useful when you have a known starting position
     */
    fun reset(newPose: Pose) {
        state[0] = newPose.x
        state[1] = newPose.y
        state[2] = normalizeAngle(newPose.heading)

        // Reset covariance to initial uncertainty
        P = Array(3) { i ->
            DoubleArray(3) { j ->
                if (i == j) 1.0 else 0.0
            }
        }
    }

    // ==================== Helper Functions ====================

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
                result[i][j] = 0.0
                for (k in 0..2) {
                    result[i][j] += A[i][k] * B[k][j]
                }
            }
        }
        return result
    }

    private fun matrixInverse3x3(m: Array<DoubleArray>): Array<DoubleArray> {
        val det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

        if (abs(det) < 1e-10) {
            // Matrix is singular, return identity
            return Array(3) { i -> DoubleArray(3) { j -> if (i == j) 1.0 else 0.0 } }
        }

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
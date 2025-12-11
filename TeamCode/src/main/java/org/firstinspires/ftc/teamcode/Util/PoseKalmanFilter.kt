package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import kotlin.math.abs

class PoseKalmanFilter(
    initialPose: Pose,
    private val trustLL: Double   // Single fusion trust factor 0.0–1.0
) {

    private var fusedPose = Pose(initialPose.x, initialPose.y, initialPose.heading)

    fun predict(odomPose: Pose) {
        // Prediction step: trust odometry fully
        fusedPose = Pose(
            odomPose.x,
            odomPose.y,
            odomPose.heading
        )
    }

    fun isOutlier(measured: Pose, maxError: Double): Boolean {
        val dx = abs(measured.x - fusedPose.x)
        val dy = abs(measured.y - fusedPose.y)
        val dh = abs(angleWrap(measured.heading - fusedPose.heading))

        return (dx > maxError || dy > maxError || dh > Math.toRadians(maxError))
    }

    fun update(camPose: Pose) {
        // Simple linear fusion between odometry (fusedPose) and vision (camPose)
        fusedPose = Pose(
            fusedPose.x * (1 - trustLL) + camPose.x * trustLL,
            fusedPose.y * (1 - trustLL) + camPose.y * trustLL,
            fusedPose.heading * (1 - trustLL) + camPose.heading * trustLL
        )
    }

    fun getPose(): Pose = fusedPose

    private fun angleWrap(angle: Double): Double {
        var a = angle
        while (a > Math.PI) a -= 2 * Math.PI
        while (a < -Math.PI) a += 2 * Math.PI
        return a
    }
}

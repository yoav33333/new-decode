package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

object Util {

    fun wrap360(angle: Double): Double {
        return ((angle % 360) + 360) % 360
    }
    fun wrap360(angle: Int): Int {
        return ((angle % 360) + 360) % 360
    }
    fun pose3dToPose(pose3d: Pose): Pose {
        return Pose(pose3d.y+72,144-(pose3d.x+72),pose3d.heading)
    }
    fun pose3DMetersToInches(pose3d: Pose3D): Pose {
        return Pose(pose3d.position.x * 39.3701,
            pose3d.position.y * 39.3701,
            pose3d.orientation.yaw,
            FTCCoordinates.INSTANCE)
    }

    @JvmStatic fun mmToInches(mm: Double): Double {
        return mm / 25.4
    }
    fun angleWrap(radians: Double): Double {
        var radians = radians
        while (radians > Math.PI) {
            radians -= 2 * Math.PI
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI
        }

        // keep in mind that the result is in radians
        return radians
    }
}
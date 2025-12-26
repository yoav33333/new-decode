package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

object Util {

    fun wrap360(angle: Double): Double {
        return ((angle % 360) + 360) % 360
    }
    fun wrap360(angle: Int): Int {
        return ((angle % 360) + 360) % 360
    }
    fun pose3dToPose(pose3d: Pose3D): Pose {
        return InvertedFTCCoordinates.INSTANCE.convertToPedro(Pose(pose3d.position.x, pose3d.position.y, pose3d.orientation.yaw))
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
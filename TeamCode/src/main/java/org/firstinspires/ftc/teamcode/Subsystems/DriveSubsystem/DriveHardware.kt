package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

//import PoseKalmanFilter
import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.Component
import org.firstinspires.ftc.teamcode.Pedro.Constants.localizer

import org.firstinspires.ftc.teamcode.Pedro.Tuning.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.trustLL
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Util.PoseKalmanFilter

object DriveHardware: Component {
    val filter = PoseKalmanFilter(Pose(0.0, 0.0, 0.0),
        trustLL)

    fun getPoseEstimate(): Pose {
        return follower.pose
    }
    fun setPoseEstimate(pose: Pose) {
        follower.pose = pose
    }
    fun updatePoseEstimate(aprilTagPose: Pose?, timeStamp: Double) {
//        // Always run prediction from dead wheels
//        filter.predict(getPoseEstimate())
//
//        // Vision update only when valid pose exists
//        if (aprilTagPose != null) {
//            filter.update(aprilTagPose)
//        }
//
//        // Push fused pose into the follower
//        setPoseEstimate(filter.getPose())
        if (aprilTagPose == null)return
        localizer.value.addMeasurement(aprilTagPose, (timeStamp*1_000_000).toLong())
    }



    override fun postInit() {

    }
    override fun postUpdate() {
        LimeLight.getRes().let { (pose, time) ->
            updatePoseEstimate(pose, time)
        }
//        updatePoseEstimate(LimeLight.getRes())
        vectorFromTarget = getPoseEstimate().asVector.minus(RobotVars.goalPos)

    }
}
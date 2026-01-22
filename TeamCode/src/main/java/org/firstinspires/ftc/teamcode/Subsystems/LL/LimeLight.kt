package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.components.Component
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.bluePipeline
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
//import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.filter
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.dist
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.distFilter
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.goalHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightLensHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightMountAngleDegrees
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.offsetFromAxis
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.redPipeline
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose
import kotlin.math.tan


object LimeLight: Component {
    val ll = lazy { hardwareMap.get(Limelight3A::class.java, "ll")  }

    fun setPipeline(pipeline: Int) {
        ll.value.pipelineSwitch(pipeline)
    }


    fun updateDistFormTag(): Double {
        val result: LLResult? = LimeLightVars.result
        if (result != null && result.isValid()) {
            val targetOffsetAngle_Vertical = result.ty
            val angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical
            val angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)
            dist =
                (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)
        }
        return dist
    }

    fun getRes(): Triple<Pose?, Double, DoubleArray> {
//        ll.value.updateRobotOrientation(TurretHardware.getGlobalHeading());
        val result: LLResult? = LimeLightVars.result
        if (result != null && result.isValid()) {
            if (result.isValid()) {
                var pose = pose3DMetersToInches(result.botpose)
                MyTelemetry.addData("MT1 pose: ", pose)
                val llPose = pose3dToPose(pose)
//                val llPose = pose3dToPose(result.botpose)
                MyTelemetry.addData("LL Pose TRUE", result.botpose)
//                MyTelemetry.addData("LL Pose", llPose.toString())
                val poseWithOffsets = addOffsets(llPose)
                LimeLightVars.llPose = poseWithOffsets
                MyTelemetry.addData("LL Pose with Offsets", poseWithOffsets.toString())
                return Triple(poseWithOffsets, result.timestamp, result.stddevMt1)
            }
        }
        return Triple(null,0.0, doubleArrayOf())
    }
    fun addOffsets(pose: Pose): Pose {
//        return pose
        LimeLightVars.centerOfRotationOffset.rotateVector(pose.heading)
        return rotationOffset(pose, 0.0+pose.heading)
            .minus(Pose(centerOfRotationOffset.xComponent, centerOfRotationOffset.yComponent))
    }
    fun rotationOffset(pose:Pose, theta: Double): Pose {
//        val offsetX = offsetFromAxis * cos(theta)
//        val offsetY = offsetFromAxis * sin(theta)
        val idoVector = Vector(offsetFromAxis, theta)
        return pose.minus(
        Pose(idoVector.xComponent, idoVector.yComponent, theta)
        )
    }
    fun updateLL(){
        ll.value.updateRobotOrientation(360-(180-Math.toDegrees(follower.pose.heading)))
        result = ll.value.getLatestResult()
    }
    override fun postInit() {
        setPipeline(if (RobotVars.allianceColor == AllianceColor.RED) redPipeline else bluePipeline)
        ll.value.setPollRateHz(50)
//        ll.value.latestResult.barcodeResults.size.
        ll.value.start()
//        Drawing.init()
    }

    fun updateSmartDist(){
        val result: LLResult? = LimeLightVars.result
//        MyTelemetry.addData("dist from tag", updateDistFormTag())
        if (result != null && result.isValid()) {
            var pose = pose3DMetersToInches(result.botpose)
            smartDist = distFilter.estimate(pose3dToPose(pose)
                .asVector.minus(RobotVars.goalPos).magnitude)
        }
    }

    override fun postUpdate() {
        updateLL()
        updateSmartDist()
        val result: LLResult? = LimeLightVars.result
        MyTelemetry.addData("smartDist", smartDist)
        MyTelemetry.addData("dist from tag", updateDistFormTag())
        if (result != null && result.isValid()) {
            var pose = pose3DMetersToInches(result.botpose)
            MyTelemetry.addData("stdDiv x: ", result.stddevMt1[0]* 39.3701)
            MyTelemetry.addData("stdDiv y: ", result.stddevMt1[1]* 39.3701)
            MyTelemetry.addData("stdDiv yaw: ", result.stddevMt1[5])
            MyTelemetry.addData("MT1 pose: ", pose)
            MyTelemetry.addData("MT1 field pose: ", pose3dToPose(pose))
//            Drawing.drawDebug(pose3dToPose(pose))
        }
    }


}

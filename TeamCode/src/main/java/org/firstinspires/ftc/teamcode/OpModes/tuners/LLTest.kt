package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Pedro.Constants.createFollower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.Drawing
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.updatePoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose

@TeleOp(group = "tuning")
class LLTest: TunerOpMode(LimeLight,
    DriveHardware, PedroComponent(Constants::createFollower)
) {
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
    }

    override fun onUpdate() {
        // 1. Fetch Limelight data
        val result: LLResult? = LimeLightVars.result
        MyTelemetry.addData("pre Robot Pose", getPoseEstimate())

        if (result != null && result.isValid()) {
            var pose = pose3DMetersToInches(result.botpose)
//            MyTelemetry.addData("stdDiv x: ", result.stddevMt1[0]* 39.3701)
//            MyTelemetry.addData("stdDiv y: ", result.stddevMt1[1]* 39.3701)
//            MyTelemetry.addData("stdDiv yaw: ", result.stddevMt1[5])
//            MyTelemetry.addData("MT1 pose: ", pose)
            MyTelemetry.addData("MT1 field pose: ", pose3dToPose(pose))
            updatePoseEstimate(pose3dToPose(pose), result.timestamp, DoubleArray(9))
//            Drawing.drawDebug(pose3dToPose(pose))
        }

        // 2. Only process if we have a valid result
//        if (res != null && res.first != null) {
//            val (pose, timestamp, dev) = res
//            updatePoseEstimate(pose, timestamp, dev)
//        }

        // 3. Update Visuals & Telemetry
        MyTelemetry.addData("post Robot Pose", getPoseEstimate())
        Drawing.drawDebug(follower)

        // 4. Update scoring vectors
        vectorFromTarget = getPoseEstimate().asVector.minus(RobotVars.goalPos)
    }
}
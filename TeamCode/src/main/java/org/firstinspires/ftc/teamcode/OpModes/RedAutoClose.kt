package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.p
import org.firstinspires.ftc.teamcode.Util.MySeqCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoClose: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(116.704, 131.789,Math.toRadians(216.0))
    @JvmField var shootingPose = Pose(93.014, 92.282)
    @JvmField var preIntakePose = Pose(97.239, 83.887)
    @JvmField var endIntakePose = Pose(125.634, 82.944)
    @JvmField var finishPose = Pose(119.775, 91.634)
    var moveToShooting1 = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
    var moveToShooting2 = PathChain()
    var moveToFinish = PathChain()
    init {
        DriveVars.startingPose = startingPose
    }
    override fun onInit(){
        moveToShooting1 = follower.pathBuilder()
            .addPath(BezierLine(startingPose, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(216.0))
            .build()
        moveToPreIntake = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(216.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose, endIntakePose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        moveToShooting2 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(216.0))
            .build()
        moveToFinish = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, finishPose))
            .setLinearHeadingInterpolation(Math.toRadians(216.0), Math.toRadians(0.0))
            .build()
//        intakeCommand.schedule()
    }

    override fun onStartButtonPressed() {
        setTargetPosition(0.0)
        var auto= SequentialGroup(
            FollowPath(moveToShooting1),
            Delay(1.0.seconds),
            FollowPath(moveToPreIntake),
            FollowPath(
                moveToEndIntake, holdEnd=false, maxPower = 0.3,
            ),
            FollowPath(moveToShooting2),
            ParallelGroup(
            FollowPath(moveToFinish),
                RepeatCommand(InstantCommand{ MyTelemetry.addData("ewtdfghtyu87", "cd") })
            )
        )
        auto.schedule()
    }



}
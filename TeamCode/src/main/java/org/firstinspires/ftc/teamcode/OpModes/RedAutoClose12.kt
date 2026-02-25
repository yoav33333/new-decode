package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.findPattern
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
import org.firstinspires.ftc.teamcode.Util.FollowPath
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPosition
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoClose12: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(116.704, 131.789,Math.toRadians(216.0))
    @JvmField var shootingPose = Pose(93.014, 92.282)
    @JvmField var preIntakePose = Pose(94.239, 88.887)
    @JvmField var preIntakePose2 = Pose(94.239, 88.887-22)
    @JvmField var gatePose = Pose(115.239, 88.887-10)
    @JvmField var preIntakePose3 = Pose(94.239, 88.887-24-22)
//    @JvmField var preIntakePose4 = Pose(127.239, 88.887-24)
    @JvmField var endIntakePose = Pose(125.634, 88.944-10)
    @JvmField var endIntakePose2 = Pose(113.634, 88.944-22)
    @JvmField var endIntakePose3 = Pose(116.634, 88.944-24-22)
//    @JvmField var endIntakePose4 = Pose(123.634, 88.944-20)
    @JvmField var finishPose = Pose(119.775, 91.634)
    var moveToShooting1 = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
    var openGate = PathChain()
    var moveToShooting2 = PathChain()
    var moveToPreIntake2 = PathChain()
    var moveToPreIntake3 = PathChain()
    var moveToPreIntake4 = PathChain()
    var moveToEndIntake2 = PathChain()
    var moveToEndIntake3 = PathChain()
    var moveToEndIntake4 = PathChain()
    var moveToShooting3 = PathChain()
    var moveToShooting4 = PathChain()
    var moveToShooting5 = PathChain()
    var moveToFinish = PathChain()
    init {
        DriveVars.startingPose = startingPose
    }
    val scan = SequentialGroup(
        resetingSeq,
        scanCommand.raceWith(WaitUntil{ isFull() })
    )
    override fun onInit(){
        moveToShooting1 = follower.pathBuilder()
            .addPath(BezierLine(startingPose, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(216.0), Math.toRadians(180.0))
            .build()
        moveToPreIntake = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose, endIntakePose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        openGate = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, gatePose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        moveToShooting2 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToPreIntake2 = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose2))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake2 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose2, endIntakePose2))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        moveToShooting3 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose2, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToPreIntake3 = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose3))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake3 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose3, endIntakePose3))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        moveToShooting4 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose3, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
//        moveToPreIntake4 = follower.pathBuilder()
//            .addPath(BezierLine(shootingPose, preIntakePose4))
//            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(215.0))
//            .build()
////        moveToEndIntake4 = follower.pathBuilder()
////            .addPath(BezierLine(preIntakePose4, endIntakePose4))
////            .setConstantHeadingInterpolation(Math.toRadians(180.0))
////            .build()
//        moveToShooting5 = follower.pathBuilder()
//            .addPath(BezierLine(endIntakePose4, shootingPose))
//            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
//            .build()
        moveToFinish = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, finishPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
//        scanCommand.schedule()
        scan.setRequirements(emptySet())
//        = mutableSetOf()
        scan.schedule()
    }

    fun auto(): SequentialGroup = SequentialGroup(

        FollowPath(moveToShooting1).and(findPattern(
            Delay(0.5).then(WaitUntil{ cachedVelocity < 15.0 })
        )
        ),
        shootingCommand,
        UninteraptingCommand(intakeCommandAuto),
        FollowPath(moveToPreIntake),
        Delay(0.2.seconds),
        (FollowPath(
             moveToEndIntake , holdEnd=false, maxPower = 0.55,
        )),
//        FollowPath( openGate),
        FollowPath( moveToShooting2 ),
        InstantCommand{intakeCommandAuto.cancel()},
        outtake,
        shootingCommand,
        FollowPath(moveToPreIntake2),
        IntakeCommands.stopIntake,
        ParallelGroup(
            UninteraptingCommand(intakeCommandAuto),
            Delay(0.01.seconds).then(FollowPath(
                moveToEndIntake2, holdEnd=false, maxPower = 0.4
            )),
        ),
        FollowPath(moveToShooting3),
        InstantCommand{intakeCommandAuto.cancel()},
        outtake,
        shootingCommand,
        FollowPath(moveToPreIntake3),
        IntakeCommands.stopIntake,
        ParallelGroup(
            UninteraptingCommand(intakeCommandAuto),
            Delay(0.01.seconds).then(FollowPath(
                moveToEndIntake3, holdEnd=false, maxPower = 0.4,
            )),
        ),
        FollowPath(moveToShooting4),
        InstantCommand{intakeCommandAuto.cancel()},
        outtake,
        shootingCommand,
        ParallelGroup(
            FollowPath(moveToFinish),
            RepeatCommand(InstantCommand{ MyTelemetry.addData("ewtdfghtyu87", "cd") })
        )
    )
    override fun onStartButtonPressed() {

        auto().schedule()
        val auto = auto()
        auto.setRequirements(this)
        auto.schedule()
        SequentialGroup(
            Delay(28.seconds),
            InstantCommand{
                LambdaCommand().setRequirements(this).schedule()
                shootingCommand.cancel()
                follower.holdPoint(finishPose)
            }
        ).schedule()
    }



}
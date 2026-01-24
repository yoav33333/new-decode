package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.outtakeThreshold
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeqAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.p
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.MySeqCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.CommandSeqNoReq
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelDeadlineGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.math.abs
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoFar: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(89.211, 8.254,Math.toRadians(270.0))
//    @JvmField var shootingPose = Pose(93.014, 92.282)
    @JvmField var preIntakePose = Pose(98.169, 34.479)
    @JvmField var preIntakePose2 = Pose(115.986, 9.000)
    @JvmField var endIntakePose = Pose(125.944, 34.789)
    @JvmField var endIntakePose2 = Pose(130.986, 9.000)
    @JvmField var finishPose = Pose(90.958, 38.408)
//    var moveToShooting1 = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
    var moveToShooting1 = PathChain()
    var moveToPreIntake2 = PathChain()
    var moveToEndIntake2 = PathChain()
    var moveToShooting2 = PathChain()
    var moveToFinish = PathChain()
    init {
        DriveVars.startingPose = startingPose
    }
    override fun onInit(){
        moveToPreIntake = follower.pathBuilder()
            .addPath(BezierLine(startingPose, preIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose, endIntakePose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        moveToShooting1 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, startingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(270.0))
            .build()
        moveToPreIntake2 = follower.pathBuilder()
            .addPath(BezierLine(startingPose, preIntakePose2))
            .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(180.0))
            .build()
        moveToEndIntake2 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose2, endIntakePose2))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        moveToFinish = follower.pathBuilder()
            .addPath(BezierLine(startingPose, finishPose))
            .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(0.0))
            .build()
//        scanCommand.schedule()
    }

    fun auto(): CommandSeqNoReq = CommandSeqNoReq(
        InstantCommand{turretSeq().schedule()
            ParallelGroup(
                InstantCommand { resetSpindexer() },
                RepeatCommand(runIntakeSeqAuto){isFull()},
            ).schedule()
        },
        Delay(0.4),
        Delay(0.9.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        reverseTransfer,
        Delay(0.2),
        stopTransfer,
        FollowPath(moveToPreIntake),
        IntakeCommands.stopIntake,
        ParallelDeadlineGroupKill(
            Delay(0.6.seconds).then(FollowPath(
                moveToEndIntake, holdEnd=false, maxPower = 0.35,
            )),
            InstantCommand { resetSpindexer() },
            RepeatCommand(runIntakeSeqAuto){isFull()},
            RepeatCommand(InstantCommand{
                MyTelemetry.addData("running","")
                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
                else setPower(intakePower)}
            ){isFull()},

        ),
        FollowPath(moveToShooting1),
        InstantCommand{ turretSeq().schedule() },
        Delay(1.5.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        reverseTransfer,
        Delay(0.2),
        stopTransfer,
        FollowPath(moveToPreIntake2),
        IntakeCommands.stopIntake,
        ParallelDeadlineGroupKill(
            Delay(0.6.seconds).then(FollowPath(
                moveToEndIntake2, holdEnd=false, maxPower = 0.35,
            )),
            InstantCommand { resetSpindexer() },
            RepeatCommand(runIntakeSeqAuto){isFull()},
            RepeatCommand(InstantCommand{
                MyTelemetry.addData("running","")
                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
                else setPower(intakePower)}
            ){isFull()},

            ),
        FollowPath(moveToShooting2),
        InstantCommand{ turretSeq().schedule() },
        Delay(1.5.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        reverseTransfer,
        Delay(0.2),
        stopTransfer,
        ParallelGroup(
            FollowPath(moveToFinish),
            RepeatCommand(InstantCommand{ MyTelemetry.addData("ewtdfghtyu87", "cd") })
        )
    )
    override fun onStartButtonPressed() {

//        turretSeq().schedule()
        auto().schedule()
    }



}
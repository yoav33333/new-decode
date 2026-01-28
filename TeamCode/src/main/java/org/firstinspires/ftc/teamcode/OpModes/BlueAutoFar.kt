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
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.TurnBy
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.outtakeThreshold
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeqAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.UtilCommands.CommandSeqNoReq
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.math.abs
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class BlueAutoFar: MegiddoOpMode(AllianceColor.BLUE) {

    @JvmField var startingPose = Pose(89.211, 8.254,Math.toRadians(270.0)).mirror()
    @JvmField var shootingPose = Pose(89.211, 15.254).mirror()
    @JvmField var preIntakePose = Pose(98.169, 34.479).mirror()
    @JvmField var preIntakePose2 = Pose(115.986, 9.000).mirror()
    @JvmField var endIntakePose = Pose(125.944, 34.789).mirror()
    @JvmField var endIntakePose2 = Pose(130.986, 12.000).mirror()
    @JvmField var finishPose = Pose(90.958, 38.408).mirror()
    var moveToShooting1 = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
    var moveToShooting2 = PathChain()
    var moveToPreIntake2 = PathChain()
    var moveToEndIntake2 = PathChain()
    var moveToShooting3 = PathChain()
    var moveToFinish = PathChain()
    init {
        DriveVars.startingPose = startingPose
    }
    override fun onInit(){
        moveToShooting1 = follower.pathBuilder()
            .addPath(BezierLine(startingPose, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180-270.0), Math.toRadians(180-240.0))
            .build()
        moveToPreIntake = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(180-270.0), Math.toRadians(180-180.0))
            .build()
        moveToEndIntake = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose, endIntakePose))
            .setConstantHeadingInterpolation(Math.toRadians(180-180.0))
            .build()
        moveToShooting2 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180-180.0), Math.toRadians(180-240.0))
            .build()
        moveToPreIntake2 = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose2))
            .setLinearHeadingInterpolation(Math.toRadians(180-240.0), Math.toRadians(180-180.0))
            .build()
        moveToEndIntake2 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose2, endIntakePose2))
            .setConstantHeadingInterpolation(Math.toRadians(180-180.0))
            .build()
        moveToEndIntake2 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose2, endIntakePose2))
            .setConstantHeadingInterpolation(Math.toRadians(180-180.0))
            .build()

        moveToShooting3 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose2, shootingPose))
            .setLinearHeadingInterpolation(Math.toRadians(180-180.0), Math.toRadians(180-240.0))
            .build()
        moveToFinish = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, finishPose))
            .setLinearHeadingInterpolation(Math.toRadians(180-240.0), Math.toRadians(180-180.0))
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
        Delay(0.2),
        FollowPath(moveToShooting1),
        Delay(0.7.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        ParallelGroup(
            SequentialGroup(reverseTransfer,
                Delay(0.1),
                stopTransfer),
            FollowPath(moveToPreIntake),),
        IntakeCommands.stopIntake,
        ParallelGroup(
            InstantCommand { resetSpindexer() },
            RepeatCommand(runIntakeSeqAuto){isFull()},
            RepeatCommand(InstantCommand{
                MyTelemetry.addData("running","")
                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
                else setPower(intakePower)}
            ){isFull()},
            Delay(0.3.seconds).then(FollowPath(
                moveToEndIntake, holdEnd=false, maxPower = 0.32,
            )),
        ),
        Delay(0.9.seconds).then(outtake),
        InstantCommand{ turretSeq().schedule() },
        FollowPath(moveToShooting2),
        Delay(0.4.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        ParallelGroup(
            SequentialGroup(reverseTransfer,
                Delay(0.1),
                stopTransfer),
            FollowPath(moveToPreIntake2),),
        IntakeCommands.stopIntake,
        ParallelGroup(
            InstantCommand { resetSpindexer() },
            RepeatCommand(runIntakeSeqAuto){isFull()},
            RepeatCommand(InstantCommand{
                MyTelemetry.addData("running","")
                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
                else setPower(intakePower)}
            ){isFull()},
            Delay(0.3.seconds).then(FollowPath(
                moveToEndIntake2, holdEnd=false, maxPower = 0.32,
            )),
        ),
        Delay(0.9.seconds).then(outtake),
        InstantCommand{ turretSeq().schedule() },
        FollowPath(moveToShooting3),
        Delay(0.3.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        reverseTransfer,
        Delay(0.1),
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
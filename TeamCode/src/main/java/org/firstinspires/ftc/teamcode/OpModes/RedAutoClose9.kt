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
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.findPattern
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
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
class RedAutoClose9: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(116.704, 131.789,Math.toRadians(216.0))
    @JvmField var shootingPose = Pose(93.014, 92.282)
    @JvmField var preIntakePose = Pose(94.239, 88.887)
    @JvmField var preIntakePose2 = Pose(94.239, 88.887-22)
    @JvmField var endIntakePose = Pose(110.634, 88.944)
    @JvmField var endIntakePose2 = Pose(110.634, 88.944-22)
    @JvmField var finishPose = Pose(119.775, 91.634)
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

//        Delay(0.2),
        FollowPath(moveToShooting1).and(findPattern(
            Delay(0.5).then(WaitUntil{ cachedVelocity < 15.0 })
        )
        ),
        //        Delay(0.2.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        ParallelGroup(
            SequentialGroup(
                reverseTransfer,
                Delay(0.1),
                stopTransfer),
        FollowPath( moveToPreIntake )),
//        IntakeCommands.stopIntake,
        UninteraptingCommand(intakeCommandAuto),

//        ParallelGroup(
//            InstantCommand { resetSpindexer() },
//            RepeatCommand(runIntakeSeqAuto){isFull()},
//            RepeatCommand(InstantCommand{
//                MyTelemetry.addData("running","")
//                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
//                else setPower(intakePower)}
//            ){isFull()},
        Delay(0.1.seconds),
        (FollowPath(
             moveToEndIntake , holdEnd=false, maxPower = 0.5,
        )),
//        ),
//        UninteraptingCommand(Delay(0.1.seconds).then(outtake)),
        FollowPath( moveToShooting2 ),
//        Delay(0.1.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),

        UninteraptingCommand(
            SequentialGroup(
                reverseTransfer,
                Delay(0.1),
                stopTransfer
            ),),
        FollowPath(moveToPreIntake2),
        IntakeCommands.stopIntake,
        ParallelGroup(
            UninteraptingCommand(intakeCommandAuto),
//            InstantCommand { resetSpindexer() },
//            RepeatCommand(runIntakeSeqAuto){isFull()},
//            RepeatCommand(InstantCommand{
//                MyTelemetry.addData("running","")
//                if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
//                else setPower(intakePower)}
//            ){isFull()},
            Delay(0.3.seconds).then(FollowPath(
                moveToEndIntake2, holdEnd=false, maxPower = 0.5,
            )),
        ),
//        UninteraptingCommand(Delay(0.9.seconds).then(outtake)),
//        InstantCommand{ turretSeq().schedule() },
        FollowPath(moveToShooting3),
//        Delay(0.1.seconds),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        ParallelGroup(
            SequentialGroup(
                reverseTransfer,
                Delay(0.1),
                stopTransfer
            ),
        ParallelGroup(
            FollowPath(moveToFinish),
            RepeatCommand(InstantCommand{ MyTelemetry.addData("ewtdfghtyu87", "cd") })
        ))
    )
    override fun onStartButtonPressed() {

//        turretSeq().schedule()
        InstantCommand{
            scan.cancel()
        }
        auto().schedule()
    }



}
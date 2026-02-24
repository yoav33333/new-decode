package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
import java.lang.Math.toRadians
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoFar9: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(89.211, 8.254, toRadians(180.0))
    @JvmField var shootingPose = Pose(89.211, 15.254)
    @JvmField var preIntakePose = Pose(98.169, 34.479)
    @JvmField var endIntakePose = Pose(115.169, 34.479)
    //    @JvmField var preIntakePose = Pose(98.169, 34.479).mirror()

    @JvmField var intakePoseHP = Pose(130.944, 5.789)
//    @JvmField var endIntakePose = Pose(125.944, 34.789)
    @JvmField var finishPose = Pose(90.958, 38.408)
    var moveToShooting1 = PathChain()
    var moveToIntakeHP = PathChain()
    var moveToShootingCycle = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
//    var moveToShooting3 = PathChain()
//    var moveToFinish = PathChain()
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
            .setLinearHeadingInterpolation(toRadians(180.0), toRadians(180.0))
            .build()
        moveToIntakeHP = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, intakePoseHP))
            .setLinearHeadingInterpolation(toRadians(180.0), toRadians(180.0))
            .build()
        moveToShootingCycle = follower.pathBuilder()
            .addPath(BezierLine(intakePoseHP, shootingPose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToPreIntake = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToEndIntake = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose, endIntakePose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToShooting1 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, startingPose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        scan.setRequirements(emptySet())
//        = mutableSetOf()
        scan.schedule()
    }

    fun auto(): SequentialGroup = SequentialGroup(
        FollowPath( moveToShooting1 ),

        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        UninteraptingCommand(intakeCommandAuto),
        Delay(0.2.seconds),
        (FollowPath(
            moveToEndIntake , holdEnd=false, maxPower = 0.55,
        )),
//        FollowPath( openGate),
//        FollowPath( moveToShooting1 ),
        FollowPath( moveToPreIntake ),
        UninteraptingCommand( intakeCommandAuto ),
//        Delay(0.3),
        FollowPath( moveToEndIntake ),
        Delay(0.3),
        outtake,
        InstantCommand{intakeCommandAuto.cancel()},
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),
        UninteraptingCommand( intakeCommandAuto ),
        FollowPath( moveToIntakeHP ),
        Delay(0.3),
        FollowPath( moveToShootingCycle ),
        Delay(0.3),
        outtake,
        InstantCommand{intakeCommandAuto.cancel()},
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),

            InstantCommand{follower.holdPoint(finishPose)}
    )
    override fun onStartButtonPressed() {

        val auto = auto()
        auto.setRequirements(this)
        auto.schedule()
        SequentialGroup(
            Delay(27.seconds),
            InstantCommand{
                LambdaCommand().setRequirements(this).schedule()
                follower.holdPoint(finishPose)
            }
        ).schedule()
    }




}
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
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.findPattern
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
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
    @JvmField var shootingPose = Pose(90.211, 15.254)
    @JvmField var preIntakePose = Pose(101.169, 34.479)
    @JvmField var endIntakePose = Pose(119.169, 34.479)
    @JvmField var preIntakePose2 = Pose(101.169, 34.479+24)
    @JvmField var endIntakePose2 = Pose(124.169, 34.479+29)
    //    @JvmField var preIntakePose = Pose(98.169, 34.479).mirror()

    @JvmField var intakePoseHP = Pose(130.944, 5.789)
//    @JvmField var endIntakePose = Pose(125.944, 34.789)
    @JvmField var finishPose = Pose(90.958, 38.408)
    var moveToShooting1 = PathChain()
    var moveToShooting2 = PathChain()
    var moveToIntakeHP = PathChain()
    var moveToShootingCycle = PathChain()
    var moveToPreIntake = PathChain()
    var moveToEndIntake = PathChain()
    var moveToPreIntake2 = PathChain()
    var moveToEndIntake2 = PathChain()
    var moveToShooting3 = PathChain()
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
        moveToShooting2 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose, startingPose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToPreIntake2 = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, preIntakePose2))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToEndIntake2 = follower.pathBuilder()
            .addPath(BezierLine(preIntakePose2, endIntakePose2))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        moveToShooting3 = follower.pathBuilder()
            .addPath(BezierLine(endIntakePose2, startingPose))
            .setConstantHeadingInterpolation(toRadians(180.0))
            .build()
        scan.setRequirements(emptySet())
//        = mutableSetOf()
        scan.schedule()
//        findPattern(
//            Delay(0.5)
//                .then(WaitUntil{ cachedVelocity < 15.0 }
//                    .then(Delay(0.5))))
//            .schedule()
    }

    fun auto(): SequentialGroup = SequentialGroup(
        FollowPath( moveToShooting1 ).and(findPattern(
            Delay(0.5).then(WaitUntil{ cachedVelocity < 15.0 })
        )),
        shootingCommand,
        FollowPath( moveToPreIntake ),
        UninteraptingCommand(intakeCommandAuto),
        Delay(0.2.seconds),
        (FollowPath(
            moveToEndIntake , holdEnd=false, maxPower = 0.5,
        )),
        FollowPath( moveToShooting2 ),
        Delay(0.1),
        outtake,
        InstantCommand{intakeCommandAuto.cancel()},
        shootingCommand,
        FollowPath( moveToPreIntake2 ),
        UninteraptingCommand(intakeCommandAuto),
        Delay(0.2.seconds),
        (FollowPath(
            moveToEndIntake2 , holdEnd=false, maxPower = 0.5,
        )),
        FollowPath( moveToShooting3 ),
        Delay(0.1),
        outtake,
        InstantCommand{intakeCommandAuto.cancel()},
        shootingCommand,
        UninteraptingCommand( intakeCommandAuto ),
        FollowPath( moveToIntakeHP ),
        Delay(0.1),
        FollowPath( moveToShootingCycle ),
        Delay(0.9),
        outtake,
        InstantCommand{intakeCommandAuto.cancel()},
        shootingCommand,

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
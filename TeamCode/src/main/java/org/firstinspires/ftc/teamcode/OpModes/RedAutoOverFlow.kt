package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
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
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.findPattern
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeqAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.Util.createPath
import org.firstinspires.ftc.teamcode.Util.UtilCommands.CommandSeqNoReq
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelDeadlineGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
import java.lang.Math.toRadians
import kotlin.math.abs
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoOverFlow: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(89.211, 8.254,Math.toRadians(180.0))
    @JvmField var shootingPose = Pose(89.211, 15.254)
    @JvmField var intakePose = Pose(130.944, 5.789)
//    @JvmField var endIntakePose = Pose(125.944, 34.789)
    @JvmField var finishPose = Pose(90.958, 38.408)
    var moveToShooting1 = PathChain()
    var moveToIntake = PathChain()
    var moveToShootingCycle = PathChain()
//    var moveToPreIntake2 = PathChain()
//    var moveToEndIntake2 = PathChain()
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
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToIntake = follower.pathBuilder()
            .addPath(BezierLine(shootingPose, intakePose))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        moveToShootingCycle = follower.pathBuilder()
            .addPath(BezierLine(intakePose, shootingPose))
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
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
//        InstantCommand{intakeCommand.schedule()},
//        Delay(0.1),
        transferAll(
            SequentialGroup(
                WaitUntil { atTargetVelocity() },
                runTransfer
            )
        ),

        RepeatCommand(
            SequentialGroup(
                UninteraptingCommand( intakeCommandAuto ),
                FollowPath( moveToIntake ),
                Delay(0.5),
                FollowPath( moveToShootingCycle ),
                Delay(0.2),
                outtake,
                InstantCommand{intakeCommandAuto.cancel()},
                transferAll(
                    SequentialGroup(
                        WaitUntil { atTargetVelocity() },
                        runTransfer
                    )
                ),

            ),
//            {time>25},
            ),

//    ),
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
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
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.Util.Cycle
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
class BlueAutoOverFlow: AutoBase(AllianceColor.BLUE) {

    override var startingPose = Pose(93.211, 8.254, toRadians(180.0)).mirror()
    @JvmField var shootingPose = Pose(93.211, 15.254,toRadians(180.0)).mirror()
    @JvmField var intakePose = Pose(135.944, 5.789,toRadians(180.0)).mirror()
    override var finishPose = Pose(90.958, 38.408,toRadians(180.0)).mirror()
    var moveToShooting1 = PathChain()
    var moveToIntake = PathChain()
    var moveToShootingCycle = PathChain()
    override fun initializePaths(){
        moveToShooting1 = createPath(startingPose, shootingPose)
        moveToIntake = createPath(shootingPose, intakePose)
        moveToShootingCycle = createPath(intakePose, shootingPose)
    }

    override fun auto(): SequentialGroup = SequentialGroup(
        FollowPath( moveToShooting1 ).and(findPattern(
            Delay(0.5).then(WaitUntil{ cachedVelocity < 15.0 })
        )),
        shootingCommand,
        RepeatCommand(
            Cycle(moveToIntake, moveToShootingCycle, 0.8)
        ),
    )
}
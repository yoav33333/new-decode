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
import org.firstinspires.ftc.teamcode.Util.Util.Cycle
import org.firstinspires.ftc.teamcode.Util.Util.createPath
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPosition
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
//import org.firstinspires.ftc.teamcode.Util.FollowPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
@Configurable
class RedAutoCloseCycle: AutoBase(AllianceColor.RED) {
    override var startingPose = Pose(116.704, 127.789,Math.toRadians(216.0))
    @JvmField var shootingPose = Pose(56.0, 83.408, Math.toRadians(180+180.0)).mirror()
    @JvmField var intakeMidCon1 = Pose(41.0, 78.5).mirror()
    @JvmField var intakeMidCon2 = Pose(43.814, 57.2).mirror()
    @JvmField var intakeMid = Pose(15.699, 56.865).mirror()
    @JvmField var backMidCon1 = Pose(35.181, 58.297).mirror()
    @JvmField var backMidCon2 = Pose(35.428, 74.068).mirror()
    @JvmField var intakeGateCon1 = Pose(38.942, 65.652).mirror()
    @JvmField var intakeGate = Pose(14.527, 58.344,Math.toRadians(335.0)).mirror()
    @JvmField var backGateCon1 = Pose(30.406, 59.837).mirror()
    @JvmField var backGateCon2 = Pose(42.038, 68.590).mirror()
    @JvmField var backGateCon3 = Pose(39.692, 72.233).mirror()
    @JvmField var intakeClose = Pose(22.028, 83.620, 0.0).mirror()

    override var finishPose = Pose(119.775, 91.634)
    var moveToShooting1 = PathChain()
    var moveToIntakeMid = PathChain()
    var moveToShooting2 = PathChain()
    var moveToIntakeGate = PathChain()
    var moveToIntakeGate2 = PathChain()
    var moveToShooting3 = PathChain()
    var moveToIntakeClose = PathChain()
    var moveToShooting4 = PathChain()
    var moveToShooting5 = PathChain()

    override fun initializePaths(){
        moveToShooting1 = createPath(startingPose, shootingPose)
        moveToIntakeMid = createPath(shootingPose, intakeMid,tangent = true, reversed = true, intakeMidCon1, intakeMidCon2)
        moveToShooting2 = createPath(intakeMid, shootingPose, tangent = true, reversed = false, backMidCon1, backMidCon2)
        moveToIntakeGate = createPath(moveToShooting2.endPose(), intakeGate, tangent = false, reversed = false, intakeGateCon1)
        moveToShooting3 = createPath(intakeGate, shootingPose, tangent = true, reversed = false, backGateCon1, backGateCon2, backGateCon3)
        moveToIntakeClose = createPath(shootingPose, intakeClose, tangent = false)
        moveToShooting4 = createPath(intakeClose, shootingPose, tangent = false)
        moveToIntakeGate2 = createPath(moveToShooting4.endPose(), intakeGate, tangent = false, reversed = false, intakeGateCon1)
        moveToShooting5 = createPath(intakeGate, shootingPose, tangent = true, reversed = false, backGateCon1, backGateCon2, backGateCon3)
    }

    override fun auto(): SequentialGroup = SequentialGroup(

        FollowPath(moveToShooting1).and(findPattern(
            Delay(0.3).then(WaitUntil{ cachedVelocity < 10.0 }))
        ),
        shootingCommand,
        Cycle(moveToIntakeMid, moveToShooting2),
        Cycle(moveToIntakeGate, moveToShooting3, 0.9),
        Cycle(moveToIntakeGate2, moveToShooting5, 1.0),
        Cycle(moveToIntakeClose, moveToShooting4),
    )
}
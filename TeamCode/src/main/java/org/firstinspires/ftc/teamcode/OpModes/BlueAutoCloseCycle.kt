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
class BlueAutoCloseCycle: AutoBase(AllianceColor.BLUE) {

    override var startingPose = Pose(116.704, 127.789,Math.toRadians(216.0)).mirror()
    @JvmField var shootingPose = Pose(56.0, 83.408, Math.toRadians(180+180.0))
    @JvmField var intakeMidCon1 = Pose(41.0, 78.5)
    @JvmField var intakeMidCon2 = Pose(43.814, 57.2)
    @JvmField var intakeMid = Pose(11.699, 56.865)
    @JvmField var backMidCon1 = Pose(35.181, 58.297)
    @JvmField var backMidCon2 = Pose(35.428, 74.068)
    @JvmField var intakeGateCon1 = Pose(49.942, 55.652)
    @JvmField var intakeGate = Pose(14.527, 61.544,Math.toRadians(335.0))
    @JvmField var backGateCon1 = Pose(30.406, 59.837)
    @JvmField var backGateCon2 = Pose(42.038, 68.590)
    @JvmField var backGateCon3 = Pose(39.692, 72.233)
    @JvmField var intakeCloseCon1 = Pose(41.521, 82.352)
    @JvmField var intakeCloseCon2 = Pose(37.359, 85.758)
    @JvmField var intakeClose = Pose(21.028, 83.620, 0.0)

    override var finishPose = Pose(60.0, 110.408, Math.toRadians(180+180.0))
    var moveToShooting1 = PathChain()
    var moveToIntakeMid = PathChain()
    var moveToShooting2 = PathChain()
    var moveToIntakeGate = PathChain()
    var moveToIntakeGate2 = PathChain()
    var moveToShooting3 = PathChain()
    var moveToIntakeClose = PathChain()
    var moveToShooting4 = PathChain()
    var moveToShooting5 = PathChain()
    var moveToFinish = PathChain()

    override fun initializePaths(){
        moveToShooting1 = createPath(startingPose, shootingPose)
        moveToIntakeMid = createPath(shootingPose, intakeMid,tangent = true, reversed = true, intakeMidCon1, intakeMidCon2)
        moveToShooting2 = createPath(intakeMid, shootingPose, tangent = true, reversed = false, backMidCon1, backMidCon2)
        moveToIntakeGate = createPath(moveToShooting2.endPose(), intakeGate, tangent = false, reversed = false, intakeGateCon1)
        moveToShooting3 = createPath(intakeGate, shootingPose, tangent = true, reversed = false, backGateCon1, backGateCon2, backGateCon3)
        moveToIntakeClose = createPath(shootingPose, intakeClose, tangent = false)
        moveToShooting4 = createPath(intakeClose, finishPose, tangent = false)
        moveToIntakeGate2 = createPath(moveToShooting4.endPose(), intakeGate.plus(Pose(1.0,-0.5)), tangent = false, reversed = false, intakeGateCon1)
        moveToShooting5 = createPath(intakeGate, shootingPose, tangent = true, reversed = false, backGateCon1, backGateCon2, backGateCon3)
    }

    override fun auto(): SequentialGroup = SequentialGroup(
        FollowPath(moveToShooting1).and(findPattern(
            Delay(0.3).then(WaitUntil{ cachedVelocity < 10.0 }))
        ),
        shootingCommand,
        Cycle(moveToIntakeMid, moveToShooting2),
        Cycle(moveToIntakeGate, moveToShooting3, 1.0),
        Cycle(moveToIntakeGate2, moveToShooting5, 1.0),
        Cycle(moveToIntakeGate2, moveToShooting5, 1.0),
        Cycle(moveToIntakeClose, moveToShooting4),
    )
}
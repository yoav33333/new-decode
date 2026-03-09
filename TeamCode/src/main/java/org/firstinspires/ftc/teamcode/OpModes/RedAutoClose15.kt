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
class RedAutoClose15: MegiddoOpMode(AllianceColor.RED) {

    @JvmField var startingPose = Pose(116.704, 127.789,Math.toRadians(216.0))
    @JvmField var shootingPose = Pose(60.085, 83.408, Math.toRadians(180+180.0)).mirror()
    @JvmField var intakeMidCon1 = Pose(53.239, 83.915).mirror()
    @JvmField var intakeMidCon2 = Pose(42.114, 58.481).mirror()
    @JvmField var intakeMid = Pose(13.699, 56.865).mirror()
    @JvmField var backMidCon1 = Pose(35.181, 58.297).mirror()
    @JvmField var backMidCon2 = Pose(55.428, 77.468).mirror()
    @JvmField var intakeGateCon1 = Pose(52.242, 75.852).mirror()
    @JvmField var intakeGateCon2 = Pose(58.164, 55.541).mirror()
    @JvmField var intakeGateCon3 = Pose(30.767, 46.573).mirror()
    @JvmField var intakeGateCon4 = Pose(23.324, 55.268).mirror()
    @JvmField var intakeGate = Pose(18.027, 59.144).mirror()
    @JvmField var backGateCon1 = Pose(30.406, 51.837).mirror()
    @JvmField var backGateCon2 = Pose(42.038, 68.590).mirror()
    @JvmField var backGateCon3 = Pose(52.692, 78.233).mirror()
    @JvmField var intakeCloseCon1 = Pose(55.521, 79.352).mirror()
    @JvmField var intakeCloseCon2 = Pose(37.359, 84.758).mirror()
    @JvmField var intakeClose = Pose(25.028, 83.620).mirror()
    @JvmField var backCloseCon1 = Pose(26.120, 83.655).mirror()
    @JvmField var backCloseCon2 = Pose(49.944, 73.521).mirror()
    @JvmField var intakeFarCon1 = Pose(51.324, 71.739).mirror()
    @JvmField var intakeFarCon2 = Pose(46.901, 51.211).mirror()
    @JvmField var intakeFarCon3 = Pose(52.958, 37.704).mirror()
    @JvmField var intakeFar = Pose(15.254, 39.338).mirror()
    @JvmField var backFarCon1 = Pose(23.676, 33.824).mirror()
    @JvmField var backFarCon2 = Pose(60.592, 82.901).mirror()



    @JvmField var finishPose = Pose(119.775, 91.634)
    var moveToShooting1 = PathChain()
    var moveToIntakeMid = PathChain()
    var moveToShooting2 = PathChain()
    var moveToIntakeGate = PathChain()
    var moveToShooting3 = PathChain()
    var moveToIntakeClose = PathChain()
    var moveToShooting4 = PathChain()
    var moveToIntakeFar = PathChain()
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
        moveToShooting1 = createPath(startingPose, shootingPose)
        moveToIntakeMid = createPath(shootingPose, intakeMid,tangent = true, reversed = true, intakeMidCon1, intakeMidCon2)
        moveToShooting2 = createPath(intakeMid, shootingPose, tangent = true, reversed = false, backMidCon1, backMidCon2)
        moveToIntakeGate = createPath(shootingPose, intakeGate, tangent = true, reversed = true, intakeGateCon1, intakeGateCon2, intakeGateCon3, intakeGateCon4)
        moveToShooting3 = createPath(intakeGate, shootingPose, tangent = true, reversed = false, backGateCon1, backGateCon2, backGateCon3)
        moveToIntakeClose = createPath(shootingPose, intakeClose, tangent = true, reversed = true, intakeCloseCon1, intakeCloseCon2)
        moveToShooting4 = createPath(intakeClose, shootingPose, tangent = true, reversed = false, backCloseCon1, backCloseCon2)
        moveToIntakeFar = createPath(shootingPose, intakeFar, tangent = true, reversed = true, intakeFarCon1, intakeFarCon2, intakeFarCon3)
        moveToShooting5 = createPath(intakeFar, shootingPose, tangent = true, reversed = false, backFarCon1, backFarCon2)
//        moveToFinish = createPath(shootingPose, finishPose)
        scan.setRequirements(emptySet())
        scan.schedule()
    }

    fun auto(): SequentialGroup = SequentialGroup(

        FollowPath(moveToShooting1).and(findPattern(
            Delay(0.3).then(WaitUntil{ cachedVelocity < 15.0 }))
        ),
        shootingCommand,
        Cycle(moveToIntakeMid, moveToShooting2),
        Cycle(moveToIntakeGate, moveToShooting3, 2.0),
        Cycle(moveToIntakeClose, moveToShooting4),
        Cycle(moveToIntakeFar, moveToShooting5),
//        ParallelGroup(
//            FollowPath(moveToFinish),
//            RepeatCommand(InstantCommand{ MyTelemetry.addData("ewtdfghtyu87", "cd") })
//        )
    )
    override fun onStartButtonPressed() {

        val auto = auto()
        auto.setRequirements(this)
        auto.schedule()
        SequentialGroup(
            Delay(28.5.seconds),
            InstantCommand{
                LambdaCommand().setRequirements(this).schedule()
                shootingCommand.cancel()
                follower.holdPoint(finishPose)
            }
        ).schedule()
    }



}
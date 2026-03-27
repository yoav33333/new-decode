package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetIMU
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetPos
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.OpModeType
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.cancelShooting
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.auto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.randomizationOffset
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands.shoot
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.fixSpindexSeq
//import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.fixSpindex
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.moveToTransferPositionLocking
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeCycle
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isAtTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.makeCurrentPosCorrect
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexerEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.state
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.State
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.lockOnGoal
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.toggleLock
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.runTurret
import org.firstinspires.ftc.teamcode.Util.ActiveDelay
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPositionFromDegrees
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds


open class ShittyTeleop(color: AllianceColor): MegiddoOpMode(color, OpModeType.TELE) {
    init {
        auto = false
        //Operator controls(Fail safes)
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(rotate(1))
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(rotate(-1))
        Gamepads.gamepad2.dpadUp.whenBecomesTrue (runTransfer )
            .whenBecomesFalse(stopTransfer )
        Gamepads.gamepad2.leftStickButton.whenBecomesTrue{tracker = SpindexerTracker()}
        Gamepads.gamepad2.dpadDown.whenBecomesTrue((IntakeCommands.intake))
            .whenBecomesFalse(IntakeCommands.stopIntake)
        Gamepads.gamepad2.x.whenBecomesTrue { makeCurrentPosCorrect() }
        Gamepads.gamepad2.rightTrigger.atLeast(0.3)
            .whenBecomesTrue{randomizationOffset-=1}
        Gamepads.gamepad2.leftTrigger.atLeast(0.3)
            .whenBecomesTrue{randomizationOffset+=1}

        //Driver controls
        Gamepads.gamepad1.rightBumper.whenBecomesTrue (intakeCommand )
        Gamepads.gamepad1.leftBumper.whenBecomesTrue (shootingCommand )
        //pre speed up
        Gamepads.gamepad1.dpadUp.whenBecomesTrue (resetIMU )
        Gamepads.gamepad1.dpadDown.whenBecomesTrue (resetPos )
        Gamepads.gamepad1.y.whenBecomesTrue {runTurret = !runTurret}
        Gamepads.gamepad1.a.whenBecomesTrue(toggleLock)
        Gamepads.gamepad1.x.whenBecomesTrue(cancelShooting)
        Gamepads.gamepad1.options.whenBecomesTrue(fixSpindexSeq)



    }
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
    }

    override fun onInit() {
//        resetingSeq.schedule()
//        turretSeq().schedule()

//        moveToAngle(270.0).schedule()
    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}
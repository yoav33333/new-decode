package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetIMU
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands.shoot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeCycle
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.lockOnGoal
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.turretSeq
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.runTurret
//import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPositionFromDegrees
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand


open class ShittyTeleop(color: AllianceColor): MegiddoOpMode(color) {
    init {
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


        //Driver controls
        Gamepads.gamepad1.rightBumper.whenBecomesTrue (intakeCommand )
        Gamepads.gamepad1.leftBumper.whenBecomesTrue (shootingCommand )
        //pre speed up
        Gamepads.gamepad1.a.whenBecomesTrue (shoot )
        Gamepads.gamepad1.dpadDown.whenBecomesTrue (resetIMU )
        Gamepads.gamepad1.y.whenBecomesTrue (turretSeq())
//            .whenBecomesFalse {  InstantCommand{globalAngle = true}}
        Gamepads.gamepad1.b.whenBecomesTrue(InstantCommand {runTurret = false})
            .whenBecomesFalse (InstantCommand{runTurret = true})




    }
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
    }

    override fun onInit() {
        turretSeq().schedule()

//        moveToAngle(270.0).schedule()
    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}
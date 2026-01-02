package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetIMU
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeCycle
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.lockOnGoal
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPositionFromDegrees
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand

@TeleOp
class ShittyTeleopRed: MegiddoOpMode(AllianceColor.RED) {
    init {
//        Gamepads.gamepad1.leftBumper
////            .whenBecomesTrue(intakeCommand)
//            .whenBecomesFalse(IntakeCommands.stopIntake)
//            .whenBecomesTrue(RepeatCommand(IntakeCommands.intake))
//        Gamepads.gamepad2.rightBumper
//            .whenBecomesTrue(shootingCommand)
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(rotate(1))
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(rotate(-1))
        Gamepads.gamepad1.a.whenBecomesTrue (intakeCommand )
        Gamepads.gamepad1.b.whenBecomesTrue (shootingCommand )
        Gamepads.gamepad1.dpadDown.whenBecomesTrue (resetIMU )
//        Gamepads.gamepad2.b.whenBecomesTrue (runIntakeCycle )
//        Gamepads.gamepad2.y.whenBecomesTrue ()
//        moveToAngle(270.0).schedule()
        Gamepads.gamepad2.leftStickButton.whenBecomesTrue{tracker = SpindexerTracker()}



    }
    override fun onStartButtonPressed() {
//        setTargetPositionFromDegrees(0.0)

        DriveCommands.driverControlled.schedule()
//        lockOnGoal.schedule()
    }

    override fun onInit() {
        moveToAngle(270.0).schedule()
    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}
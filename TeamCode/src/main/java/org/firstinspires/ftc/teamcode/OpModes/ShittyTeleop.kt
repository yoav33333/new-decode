package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.lockOnGoal

@TeleOp
class ShittyTeleopRed: MegiddoOpMode(AllianceColor.RED) {
    init {
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(intakeCommand)
            .whenBecomesFalse(IntakeCommands.stopIntake)
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(shootingCommand)


    }
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
        lockOnGoal.schedule()
    }
}
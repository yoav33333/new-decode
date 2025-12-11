package org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower

object IntakeCommands {
    val intake = InstantCommand { setPower(intakePower) }.setRequirements(IntakeHardware)
    val stopIntake = InstantCommand { setPower(0.0) }.setRequirements(IntakeHardware)
    val outtake = InstantCommand { setPower(-intakePower) }.setRequirements(IntakeHardware)
}
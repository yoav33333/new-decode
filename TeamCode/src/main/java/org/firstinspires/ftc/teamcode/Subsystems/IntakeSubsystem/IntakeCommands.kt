package org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import kotlin.math.abs

object IntakeCommands {
    val intake = InstantCommand { setPower(intakePower) }
    val stopIntake = InstantCommand { setPower(0.0) }
    val outtake = InstantCommand { setPower(-intakePower) }
    val smartIntake = LambdaCommand().setUpdate {
        MyTelemetry.addData("running","")
        if (abs(getVel()) <2000) setPower(-intakePower)
        else setPower(intakePower)}
}
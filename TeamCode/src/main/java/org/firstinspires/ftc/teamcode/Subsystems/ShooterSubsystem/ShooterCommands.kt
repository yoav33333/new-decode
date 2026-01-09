package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import java.util.function.DoubleSupplier

object ShooterCommands {
    fun setVelocity(velocity: Double) =
        InstantCommand{ ShooterHardware.setVelocity(velocity)}.setRequirements(ShooterHardware)
    fun setHoodPosition(position: Double) =
        InstantCommand{ ShooterHardware.setHoodPosition(position)}.setRequirements(ShooterHardware)
    fun shoot(distance: DoubleSupplier) =
        InstantCommand{ ShooterHardware.shoot(distance.asDouble)}
}
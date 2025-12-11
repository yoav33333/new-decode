package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads

object DriveCommands {
    val driverControlled = PedroDriverControlled(
        Gamepads.gamepad1.leftStickY,
        Gamepads.gamepad1.leftStickX,
        Gamepads.gamepad1.rightStickX
    ).setRequirements(DriveHardware)
}
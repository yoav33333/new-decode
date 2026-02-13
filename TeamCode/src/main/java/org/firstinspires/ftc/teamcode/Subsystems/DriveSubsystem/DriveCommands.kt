package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.isHolding

object DriveCommands {
    val driverControlled = PedroDriverControlled(
        Gamepads.gamepad1.leftStickY,
        Gamepads.gamepad1.leftStickX,
        -Gamepads.gamepad1.rightStickX,
        false
    ).setRequirements(DriveHardware)
    val resetIMU = InstantCommand{
        follower.heading = 0.0
        if (isHolding) follower.holdPoint(follower.pose)
    }
}
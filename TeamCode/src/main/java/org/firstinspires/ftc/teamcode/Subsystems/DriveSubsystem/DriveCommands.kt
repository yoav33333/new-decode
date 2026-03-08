package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import androidx.core.util.plus
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.isHolding
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.resetPosBlue
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.resetPosRed
import org.firstinspires.ftc.teamcode.Util.MyPedroDrive

object DriveCommands {
    val driverControlled = MyPedroDrive(
        Gamepads.gamepad1.leftStickY,
        Gamepads.gamepad1.leftStickX,
        -Gamepads.gamepad1.rightStickX,
        false
    )
    val resetIMU = InstantCommand{
        follower.heading = 0.0
        if (isHolding) follower.holdPoint(follower.pose)

    }
    val resetPos = InstantCommand{
        if (RobotVars.allianceColor == AllianceColor.RED) {
            follower.pose = resetPosRed
        } else {
            follower.pose = resetPosBlue
        }
        if (isHolding) follower.holdPoint(follower.pose)

    }
}
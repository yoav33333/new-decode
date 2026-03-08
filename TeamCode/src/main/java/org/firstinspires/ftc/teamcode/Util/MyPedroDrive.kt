package org.firstinspires.ftc.teamcode.Util

import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.allianceColor
import java.util.function.Supplier

class MyPedroDrive @JvmOverloads constructor(
    drivePower: Supplier<Double>,
    strafePower: Supplier<Double>,
    turnPower: Supplier<Double>,
    private val robotCentric: Boolean = true
) : DriverControlledCommand(drivePower, strafePower, turnPower) {

    override fun start() {
        follower.startTeleopDrive()
    }

    override fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers
        follower.setTeleOpDrive(drive* if (allianceColor == AllianceColor.RED) -1 else 1, strafe* if (allianceColor == AllianceColor.RED) -1 else 1, turn, robotCentric)
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) follower.breakFollowing()
    }
}
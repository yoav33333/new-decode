package org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem

import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.MotorEx

object TransferHardware: Component {
    val transferMotor = lazy { MotorEx("Transfer").brakeMode() }

    fun setPower(power: Double) {
        transferMotor.value.power = power
    }
}
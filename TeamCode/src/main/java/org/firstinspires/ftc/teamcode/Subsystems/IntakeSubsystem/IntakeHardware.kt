package org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem

import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.MotorEx

object IntakeHardware: Component {
    val intakeMotor = lazy { MotorEx("Intake").reversed() }
    fun setPower(power: Double) {
        intakeMotor.value.power = power
    }

}
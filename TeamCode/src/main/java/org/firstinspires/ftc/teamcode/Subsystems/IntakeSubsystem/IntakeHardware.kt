package org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem

import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry

object IntakeHardware: Component {
    val intakeMotor = lazy { MotorEx("Intake").reversed() }
    fun setPower(power: Double) {
        intakeMotor.value.power = power
    }

    override fun postUpdate() {
        MyTelemetry.addData("Intake power", intakeMotor.value.power)
    }

}
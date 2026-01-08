package org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem

import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import kotlin.math.abs

object IntakeHardware: Component {
    val intakeMotor = lazy { MotorEx("Intake").reversed() }
    fun setPower(power: Double) {
        intakeMotor.value.power = power
    }
    fun getVel(): Double{
        return intakeMotor.value.velocity
    }

    override fun postUpdate() {
        MyTelemetry.addData("Intake power", intakeMotor.value.power)
        MyTelemetry.addData("Intake speed", getVel())
        MyTelemetry.addData("pass threshold", abs(getVel()) <2000)
    }

}
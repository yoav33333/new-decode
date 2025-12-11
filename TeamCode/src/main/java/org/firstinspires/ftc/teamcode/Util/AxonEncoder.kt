package org.firstinspires.ftc.teamcode.Util

import com.qualcomm.robotcore.hardware.AnalogInput
import dev.nextftc.ftc.ActiveOpMode.hardwareMap

class AxonEncoder(name: String) {
//    init {
    val encoder = hardwareMap.get(AnalogInput::class.java, name)
//    }
    fun getVoltage(): Double {
//        val encoder = hardwareMap.get(AnalogInput::class.java, "axonEncoder")
        return encoder.voltage
    }
    fun getPosition(): Double {
        val voltage = getVoltage()
        val position = (voltage / encoder.maxVoltage) * 360.0
        return position
    }

}
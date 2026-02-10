package org.firstinspires.ftc.teamcode.Util

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

class InputChannel(val hmap: HardwareMap, val name: String) {
    fun get(): DigitalChannel{
        val dev =  hmap.get(DigitalChannel::class.java, name)
        dev.mode = DigitalChannel.Mode.INPUT
        return dev

    }
}
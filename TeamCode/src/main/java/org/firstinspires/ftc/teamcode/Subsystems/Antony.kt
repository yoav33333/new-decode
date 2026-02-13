package org.firstinspires.ftc.teamcode.Subsystems

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareDevice
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker

object Antony : Component {
    val antony = lazy{ hardwareMap.get(RevBlinkinLedDriver::class.java, "LED") }
    fun setPattern(pattern: RevBlinkinLedDriver.BlinkinPattern) {
        antony.value.setPattern(pattern)
    }
    val zero = RevBlinkinLedDriver.BlinkinPattern.RED
    val one = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE
    val two = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN
    val three = RevBlinkinLedDriver.BlinkinPattern.GREEN
    override fun postUpdate() {
        var pattern = when(tracker.getAmount()){
            0->zero
            1-> one
            2->two
            3->three
            else -> {zero}
        }
        setPattern(pattern)
    }
}
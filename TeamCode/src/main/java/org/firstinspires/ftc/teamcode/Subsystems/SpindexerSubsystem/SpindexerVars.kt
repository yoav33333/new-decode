package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.bylazar.configurables.annotations.Configurable
//import org.firstinspires.ftc.teamcode.Util.ColorRange
import org.firstinspires.ftc.teamcode.Util.HSVRange
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState


@Configurable
object SpindexerVars {
//    @JvmField var p = 0.1
//    @JvmField var i = 0.1
//    @JvmField var d = 0.0
//    @JvmField var kv = 0.0
//    @JvmField var gain = 0.2
//    @JvmField var f = 0.0
    @JvmField var targetPosition = 0.0
    @JvmField var steps = 0
    @JvmField var maxRotation = 355.0
    @JvmField var degreesPerSlot = 66
    @JvmField var intakeSlot = 2
    @JvmField var transferSlot = 0
    @JvmField var spinDelayShoot = 0.25
    @JvmField var spinDelayIntake = 0.00
    @JvmField var delayMul = 1.0
    @JvmField var offset = 4.5
    @JvmField var offsetEnc = 16
    @JvmField var MulEnc = 1.1
    @JvmField var purpleRange = HSVRange(190.0,1000.0,0.0,1000.0,0.0,1000.0)
    @JvmField var greenRange = HSVRange(80.0,190.0,0.0,1000.0,0.0,1000.0)
//    @JvmField var distThreshold = 30
    @JvmField var defaultColor = SpindexerSlotState.PURPLE

//    @JvmField var coefficients= PIDCoefficients(0.5, 0.0, 0.0)
//    @JvmField var pid = BasicPID(coefficients)
//    @JvmField var controller = AngleController(pid)


}
package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
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
    @JvmField var maxRotation = 255.0
    @JvmField var degreesPerSlot = 30
    @JvmField var intakeSlot = 0
    @JvmField var transferSlot = 1
    @JvmField var spinDelay = 0.5
    @JvmField var offset = 0.0
    @JvmField var purpleRange = HSVRange(160.0,1000.0,0.0,1000.0,0.0,1000.0)
    @JvmField var greenRange = HSVRange(30.0,160.0,0.0,1000.0,0.0,1000.0)
//    @JvmField var distThreshold = 30
    @JvmField var defaultColor = SpindexerSlotState.PURPLE

//    @JvmField var coefficients= PIDCoefficients(0.5, 0.0, 0.0)
//    @JvmField var pid = BasicPID(coefficients)
//    @JvmField var controller = AngleController(pid)


}
package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.bylazar.configurables.annotations.Configurable

@Configurable
object TurretVars {
    @JvmField var servoRange = 310.0
    @JvmField var p = 1.7
    @JvmField var distP = 0.05
    @JvmField var encoderMul = 8192*360
    @JvmField var Kv = 1.0
    @JvmField var offset = .0
    @JvmField var targetPosition = 0.65
    @JvmField var posCoefficients: PIDCoefficients = PIDCoefficients(0.0072, 0.0, 0.08)
    @JvmField var angleControl: BasicPID = BasicPID(posCoefficients)
//310/360*255=219.583
}
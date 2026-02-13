package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.bylazar.configurables.annotations.Configurable

@Configurable
object TurretVars {
    @JvmField var servoRange = 351.69
    @JvmField var p = -0.9
    @JvmField var runTurret = true
    @JvmField var distP = 0.05
    @JvmField var encoderMul = 8192*360
    @JvmField var Kv = 0.5
    @JvmField var offset = .0
    @JvmField var servoOffset = 1.0

    @JvmField var state = TurretState.Disabled
    @JvmField var targetPosition = 0.0
    @JvmField var posCoefficients: PIDCoefficients = PIDCoefficients(0.0135, 0.0, 0.0009)
    @JvmField var angleControl: BasicPID = BasicPID(posCoefficients)
//310/360*255=219.583
}
enum class TurretState{
    TrackingAprilTags,
    ResetEncoder,
    Disabled
}
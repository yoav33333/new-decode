package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.bylazar.configurables.annotations.Configurable

@Configurable
object TurretVars {
    @JvmField var servoRange = 346.81
    @JvmField var p = -.3
    @JvmField var rotationPred = 0.07
    @JvmField var runTurret = true
    @JvmField var offset = .0
    @JvmField var offsetLL = .0

    @JvmField var reducer = 0.93
    @JvmField var servoOffset = -4.0

    @JvmField var state = TurretState.Disabled
    @JvmField var targetPosition = 0.0
//310/360*255=219.583
}
enum class TurretState{
    TrackingAprilTags,
    ResetEncoder,
    Disabled
}
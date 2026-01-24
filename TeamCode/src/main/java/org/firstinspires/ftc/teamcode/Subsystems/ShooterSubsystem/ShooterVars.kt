package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.Util.InterpLUT


@Configurable
object ShooterVars {
    @JvmField var shootPowLUT = createShooterLut()
    @JvmField var hoodLUT = createHoodLUT()
    @JvmField var runShooter = false
    @JvmField var disableAutoShooter = false
    @JvmField var f = 0.65
    @JvmField var hoodCorrectionMul = 0.000
    @JvmField var controlledSpeed = 0.0
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 10.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.013, 0.0, 0.01)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(61.3 to 950.0, 71.0 to 1100.0, 94.0 to 1250.0, 117.2 to 1300.0, 135.0 to 1400.0, 162.8 to 1500.0))
    }
    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(61.3 to 0.00, 71.0 to 0.0, 94.0 to 0.2, 117.2 to 0.3, 135.0 to 0.35, 162.8 to 0.45))
    }
    fun createLUT(map: Map<Double, Double>): InterpLUT{
        var lut = InterpLUT()
        map.forEach{
            lut.add(it.key, it.value)
        }
        lut.createLUT()
        return lut
    }
}
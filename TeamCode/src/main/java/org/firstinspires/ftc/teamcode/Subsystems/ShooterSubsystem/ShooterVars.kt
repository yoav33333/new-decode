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
    @JvmField var f = 0.67
    @JvmField var hoodCorrectionMul = 0.000
    @JvmField var controlledSpeed = 0.0
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 10.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.012, 0.0, 0.001)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(61.3 to 950.0, 71.0 to 1150.0, 94.0 to 1270.0, 117.2 to 1410.0, 135.0 to 1480.0, 162.8 to 1665.0))
    }
    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(61.3 to 0.00, 71.0 to 0.15, 94.0 to 0.29, 117.2 to 0.36, 135.0 to 0.39, 162.8 to 0.4))
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
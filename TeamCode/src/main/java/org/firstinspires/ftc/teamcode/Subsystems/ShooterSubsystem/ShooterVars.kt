package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.Util.InterpLUT


@Configurable
object ShooterVars {
    @JvmField var shootPowLUT = createShooterLut()
    @JvmField var hoodLUT = createHoodLUT()
    @JvmField var kv = 0.00039
    @JvmField var angleCorrectMul = -0.000
    @JvmField var ks = 0.045
    @JvmField var offset = 261.77
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 90.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.009, 0.0, 0.00)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(61.3 to 960.0, 71.0 to 1015.0, 94.0 to 1120.0, 117.2 to 1240.0, 135.0 to 1380.0, 162.8 to 1470.0))
    }

    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(61.3 to 0.0, 71.0 to 0.1, 94.0 to 0.25, 117.2 to 0.25, 135.0 to 0.35, 162.8 to 0.35))
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
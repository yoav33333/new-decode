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
    @JvmField var f = 0.62
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 100.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.03 , 0.0, 0.001)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(66.7 to 1100.0,77.1 to 1200.0,89.3 to 1250.0,104.0 to 1300.0, 112.8 to 1350.0, 156.5 to 1500.0))
    }
    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(66.7 to 0.00,77.1 to 0.3,89.3 to 0.3,104.0 to 0.2, 112.8 to 0.35, 156.5 to 0.35))
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
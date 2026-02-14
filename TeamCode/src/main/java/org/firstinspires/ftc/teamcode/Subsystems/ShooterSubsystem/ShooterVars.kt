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
    @JvmField var kv = 0.00051
    @JvmField var ks = 0.1
    @JvmField var minPos = 0.13
    @JvmField var maxPos = 0.51
    @JvmField var minAngle = 90-48.54
    @JvmField var maxAngle = 90-35.38
    @JvmField var hoodTargetAngle = 40.0
    @JvmField var hoodCorrectionMul = 0.000
    @JvmField var entryAng = 20.0
    @JvmField var dy = 25.98425197
    @JvmField var controlledSpeed = 0.0
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 20.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.007, 0.0, 0.001)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(61.3 to 800.0, 71.0 to 950.0, 94.0 to 1070.0, 117.2 to 1110.0, 135.0 to 1290.0, 162.8 to 1380.0))
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
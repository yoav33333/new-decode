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
    @JvmField var kv = 0.00054
    @JvmField var ks = 0.12
    @JvmField var minPos = 0.13
    @JvmField var maxPos = 0.51
    @JvmField var minAngle = 90-48.54-0.5
    @JvmField var maxAngle = 90-35.38-0.5
    @JvmField var mul = 6.0722
    @JvmField var offset = 261.77
    @JvmField var hoodTargetAngle = 40.0
    @JvmField var hoodCorrectionMul = 0.000
    @JvmField var entryAng = 20.0
    @JvmField var dy = 25.98425197
    @JvmField var controlledSpeed = 0.0
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 80.0
    @JvmField var veloCoefficients: PIDCoefficients = PIDCoefficients(0.009, 0.0, 0.00)
    @JvmField var veloControl: BasicPID = BasicPID(veloCoefficients)

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(61.3 to 780.0, 71.0 to 820.0, 94.0 to 970.0, 117.2 to 1000.0, 135.0 to 1260.0, 162.8 to 1330.0))
    }
    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(61.3 to 0.10, 71.0 to 0.25, 94.0 to 0.35, 117.2 to 0.36, 135.0 to 0.38, 162.8 to 0.42))
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
package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.Util.InterpLUT

@Configurable
object ShooterVars {
    @JvmField var shootPowLUT = createShooterLut()
    @JvmField var hoodLUT = createHoodLUT()
    @JvmField var p = 0.0
    @JvmField var i = 0.0
    @JvmField var d = 0.0
    @JvmField var hoodTarget = 0.0
    @JvmField var targetVelocity = 0.0
    @JvmField var deltaThreshold = 200.0

    fun createShooterLut(): InterpLUT{
        return createLUT(mapOf(0.0 to 0.0, 100.0 to 1.0, 200.0 to 2.0, 300.0 to 3.0))
    }
    fun createHoodLUT(): InterpLUT{
        return createLUT(mapOf(0.0 to 0.0, 100.0 to 1.0, 200.0 to 2.0, 300.0 to 3.0))
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
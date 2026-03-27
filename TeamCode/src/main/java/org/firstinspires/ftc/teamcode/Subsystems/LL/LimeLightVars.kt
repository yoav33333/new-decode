package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.Util.Util.mmToInches

@Configurable
object LimeLightVars {
    @JvmField var result: LLResult? = null
    @JvmField var redPipeline = 0
    @JvmField var bluePipeline = 1
    @JvmField var patternPipeline = 7
    @JvmField var smartDist = 0.0
    @JvmField var distFilter = LowPassFilter(0.2)
    @JvmField var centerOfRotationOffset = Pose( -mmToInches(28.06),mmToInches(68.0)).asVector

    val PPG = 23
    val PGP = 22
    val GPP  = 21
    //h - 434.7
}
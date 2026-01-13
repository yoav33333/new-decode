package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.Util.Util.mmToInches

@Configurable
object LimeLightVars {
    @JvmField var result: LLResult? = null
    @JvmField var localizationPipeline = 0
    @JvmField var regularAprilTagPipeline = 1
    @JvmField var llPose = Pose()
    @JvmField var dist = 0.0
    @JvmField var smartDist = 0.0
    @JvmField var distFilter = LowPassFilter(0.8)
    @JvmField var limelightMountAngleDegrees = 0.0
    @JvmField var limelightLensHeightInches = 17.1141732
    @JvmField var goalHeightInches = mmToInches(753.0)
    @JvmField var centerOfRotationOffset = Pose(mmToInches(68.0), mmToInches(28.06))
    @JvmField var offsetFromAxis = mmToInches(54.5)
    val redId = 24
    val blueId = 20
    val PPG = 21
    val PGP = 22
    val GPP  = 23
    //h - 434.7
}
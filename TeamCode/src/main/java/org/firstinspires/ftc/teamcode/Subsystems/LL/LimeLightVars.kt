package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.Util.Util.mmToInches

@Configurable
object LimeLightVars {
    @JvmField var result: LLResult? = null
    @JvmField var localizationPipeline = 0
    @JvmField var regularAprilTagPipeline = 1
    @JvmField var dist = 0.0
    @JvmField var limelightMountAngleDegrees = 0.0
    @JvmField var limelightLensHeightInches = 20.0
    @JvmField var goalHeightInches = 60.0
    @JvmField var centerOfRotationOffset = Pose(mmToInches(68.0), mmToInches(28.06))
    @JvmField var offsetFromAxis = mmToInches(54.5)
    val redId = 24
    val blueId = 20
    val PPG = 21
    val PGP = 22
    val GPP  = 23
    //h - 434.7
}
package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.Pedro.FusionLocalizer

@Configurable
object DriveVars {
    var trustLL = 0.25

    // Inside your Follower/Localizer initialization:
    @JvmField var P: DoubleArray = doubleArrayOf(0.1, 0.1, 0.1) // Initial uncertainty

    // Q: How much the wheels drift (Increase these to make the robot "listen" to the camera more)
    @JvmField var processVariance: DoubleArray = doubleArrayOf(0.2, 0.2, 0.2)

    // R: How much you trust the camera (Decrease these to make the camera "stronger")
    @JvmField var measurementVariance: DoubleArray = doubleArrayOf(0.01, 0.01, 0.01)

    @JvmField var startingPose = Pose(72.0,72.0,0.0)

}
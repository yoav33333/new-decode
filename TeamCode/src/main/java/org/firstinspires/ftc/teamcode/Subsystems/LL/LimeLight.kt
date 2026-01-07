package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.offsetFromAxis
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose
import kotlin.math.cos
import kotlin.math.sin


object LimeLight: Component {
    val ll = lazy { hardwareMap.get(Limelight3A::class.java, "ll")  }

    fun setPipeline(pipeline: Int) {
        ll.value.pipelineSwitch(pipeline)
    }

    override fun postInit() {
        setPipeline(LimeLightVars.localizationPipeline)
        ll.value.setPollRateHz(50)
        ll.value.start()
    }

    fun getRes(): Triple<Pose?, Double, DoubleArray> {
//        ll.value.updateRobotOrientation(TurretHardware.getGlobalHeading());
        val result: LLResult? = ll.value.getLatestResult()
        if (result != null && result.isValid()) {
            if (result.isValid()) {
                val llPose = pose3dToPose(result.botpose)
                MyTelemetry.addData("LL Pose TRUE", result.botpose)
                MyTelemetry.addData("LL Pose", llPose.toString())
                val poseWithOffsets = addOffsets(llPose)
                MyTelemetry.addData("LL Pose with Offsets", poseWithOffsets.toString())
                return Triple(poseWithOffsets, result.timestamp, result.stddevMt1)
            }
        }
        return Triple(null,0.0, doubleArrayOf())
    }
    fun addOffsets(pose: Pose): Pose {
        return pose
//        return rotationOffset(pose, 0.0)
//            .plus(LimeLightVars.centerOfRotationOffset).plus(Pose())
    }
    fun rotationOffset(pose:Pose, theta: Double): Pose {
        val offsetX = offsetFromAxis * cos(theta)
        val offsetY = offsetFromAxis * sin(theta)

        return pose.plus(Pose(offsetX, offsetY, 0.0) )
    }



}
package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.components.Component
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.LLMul
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.bluePipeline
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
//import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.filter
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.dist
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.distFilter
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.goalHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightLensHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightMountAngleDegrees
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.offsetFromAxis
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.patternPipeline
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.redPipeline
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Randomization
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.allianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.deltaVec
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPos
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosBlue
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosRed
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.obelisk
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.randomization
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.offset
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose
import kotlin.math.tan


object LimeLight: Component {
    val ll = lazy { hardwareMap.get(Limelight3A::class.java, "ll")  }

    fun setPipeline(pipeline: Int) {
        ll.value.pipelineSwitch(pipeline)
    }
    fun setScan(){
        setPipeline(patternPipeline)
        goalPos = obelisk
    }
    fun setTarget(){
        setPipeline(if (RobotVars.allianceColor == AllianceColor.RED) redPipeline else bluePipeline)
        RobotVars.goalPos =( if (allianceColor == AllianceColor.RED) goalPosRed else goalPosBlue)
    }

    fun updateLL(){
        ll.value.updateRobotOrientation(180+Math.toDegrees(deltaVec.theta-offset))
        result = ll.value.getLatestResult()
    }
    override fun postInit() {
        setPipeline(if (RobotVars.allianceColor == AllianceColor.RED) redPipeline else bluePipeline)
        ll.value.setPollRateHz(50)
        ll.value.start()
    }
    fun getId():Int?{
        val result: LLResult? = LimeLightVars.result
        return if (result != null && result.isValid())
            result.fiducialResults[0].fiducialId
        else
            null
    }

    fun getPattern(): Randomization{
        return when(getId()){
            21 -> Randomization.GPP
            22 -> Randomization.PGP
            23 -> Randomization.PPG
            else -> randomization
        }
    }

    fun updateSmartDist(){
        val result: LLResult? = LimeLightVars.result
//        MyTelemetry.addData("dist from tag", updateDistFormTag())
        if (result != null && result.isValid()) {
            MyTelemetry.addData("id", result.fiducialResults[0].fiducialId)
            var pose = pose3DMetersToInches(result.botpose)
            smartDist = distFilter.estimate(pose3dToPose(pose)
                .asVector.minus(RobotVars.goalPos).magnitude)
            smartDist = distFilter.estimate(pose3dToPose(pose)
                .plus(Pose(follower.velocity.xComponent*smartDist*LLMul, follower.velocity.yComponent*smartDist*LLMul))
                .asVector.minus(RobotVars.goalPos).magnitude)
        }
    }

    override fun postUpdate() {
        updateLL()
        updateSmartDist()
        randomization = getPattern()
    }

    override fun postWaitForStart() {
        updateLL()
        updateSmartDist()

    }


}

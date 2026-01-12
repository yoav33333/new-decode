package org.firstinspires.ftc.teamcode.Subsystems.LL

import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.components.Component
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
//import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.filter
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.dist
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.distFilter
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.goalHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightLensHeightInches
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.limelightMountAngleDegrees
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.offsetFromAxis
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan


object LimeLight: Component {
    val ll = lazy { hardwareMap.get(Limelight3A::class.java, "ll")  }

    fun setPipeline(pipeline: Int) {
        ll.value.pipelineSwitch(pipeline)
    }


    fun updateDistFormTag(): Double {
        val result: LLResult? = LimeLightVars.result
        if (result != null && result.isValid()) {
            val targetOffsetAngle_Vertical = result.ty
            val angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical
            val angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)
            dist =
                (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)
        }
        return dist
    }

    fun getRes(): Triple<Pose?, Double, DoubleArray> {
//        ll.value.updateRobotOrientation(TurretHardware.getGlobalHeading());
        val result: LLResult? = LimeLightVars.result
        if (result != null && result.isValid()) {
            if (result.isValid()) {
                var pose = pose3DMetersToInches(result.botpose)
                MyTelemetry.addData("MT1 pose: ", pose)
                val llPose = pose3dToPose(pose)
//                val llPose = pose3dToPose(result.botpose)
                MyTelemetry.addData("LL Pose TRUE", result.botpose)
//                MyTelemetry.addData("LL Pose", llPose.toString())
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
    fun updateLL(){
        result = ll.value.getLatestResult()
    }
    override fun postInit() {
        setPipeline(LimeLightVars.localizationPipeline)
        ll.value.setPollRateHz(50)
        ll.value.start()
        Drawing.init()
    }

    fun updateSmartDist(){
        val result: LLResult? = LimeLightVars.result
//        MyTelemetry.addData("dist from tag", updateDistFormTag())
        if (result != null && result.isValid()) {
            var pose = pose3DMetersToInches(result.botpose)
            smartDist = distFilter.estimate(pose3dToPose(pose).asVector.minus(RobotVars.goalPos).magnitude)
        }
    }

    override fun postUpdate() {
        updateLL()
        updateSmartDist()
        val result: LLResult? = LimeLightVars.result
        MyTelemetry.addData("smartDist", smartDist)
        MyTelemetry.addData("dist from tag", updateDistFormTag())
        if (result != null && result.isValid()) {
            var pose = pose3DMetersToInches(result.botpose)
            MyTelemetry.addData("stdDiv x: ", result.stddevMt1[0]* 39.3701)
            MyTelemetry.addData("stdDiv y: ", result.stddevMt1[1]* 39.3701)
            MyTelemetry.addData("stdDiv yaw: ", result.stddevMt1[5])
            MyTelemetry.addData("MT1 pose: ", pose)
            MyTelemetry.addData("MT1 field pose: ", pose3dToPose(pose))
//            Drawing.drawDebug(pose3dToPose(pose))
        }
    }


}
internal object Drawing {
    const val ROBOT_RADIUS: Double = 9.0 // woah
    private val panelsField = field

    private val robotLook = Style(
        "", "#3F51B5", 0.75
    )
    private val historyLook = Style(
        "", "#4CAF50", 0.75
    )

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    fun init() {
        panelsField.setOffsets(presets.PEDRO_PATHING)
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    fun drawDebug(pose: Pose) {
//        drawPath(follower.getCurrentPath(), robotLook)
//        val closestPoint =
//            follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue())
        drawRobot(
            Pose(
                pose.x,
                pose.y,
                pose.heading
//                    .getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())
            ), robotLook
        )

//        drawPoseHistory(follower.getPoseHistory(), historyLook)
//        drawRobot(follower.getPose(), historyLook)

        sendPacket()
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    @JvmOverloads
    fun drawRobot(pose: Pose?, style: Style = robotLook) {
        if (pose == null || java.lang.Double.isNaN(pose.getX()) || java.lang.Double.isNaN(pose.getY()) || java.lang.Double.isNaN(
                pose.getHeading()
            )
        ) {
            return
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.getX(), pose.getY())
        panelsField.circle(ROBOT_RADIUS)

        val v = pose.getHeadingAsUnitVector()
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS)
        val x1 = pose.getX() + v.getXComponent() / 2
        val y1 = pose.getY() + v.getYComponent() / 2
        val x2 = pose.getX() + v.getXComponent()
        val y2 = pose.getY() + v.getYComponent()

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    fun drawPath(path: Path, style: Style) {
        val points = path.getPanelsDrawingPoints()

        for (i in points[0]!!.indices) {
            for (j in points.indices) {
                if (java.lang.Double.isNaN(points[j]!![i])) {
                    points[j]!![i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(points[0]!![0], points[0]!![1])
        panelsField.line(points[1]!![0], points[1]!![1])
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    fun drawPath(pathChain: PathChain, style: Style) {
        for (i in 0..<pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    @JvmOverloads
    fun drawPoseHistory(poseTracker: PoseHistory, style: Style = historyLook) {
        panelsField.setStyle(style)

        val size = poseTracker.getXPositionsArray().size
        for (i in 0..<size - 1) {
            panelsField.moveCursor(
                poseTracker.getXPositionsArray()[i],
                poseTracker.getYPositionsArray()[i]
            )
            panelsField.line(
                poseTracker.getXPositionsArray()[i + 1],
                poseTracker.getYPositionsArray()[i + 1]
            )
        }
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    fun sendPacket() {
        panelsField.update()
    }
}
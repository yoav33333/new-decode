package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import dev.nextftc.core.components.Component
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Util.PoseKalmanFilter
import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.pedropathing.math.Vector
import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight.addOffsets
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.llPose
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose

@Configurable
object DriveHardware : Component {
    @JvmField var filter = PoseKalmanFilter(
        initialPose = Pose(72.0, 72.0, 0.0),
        processNoiseX = 0.01,
        processNoiseY = 0.01,
        processNoiseTheta = 0.005,
        measurementNoiseX = 8.5,
        measurementNoiseY = 8.5,
        measurementNoiseTheta = 0.2 // Trusting vision heading more than your previous 2.0
    )

    fun getPoseEstimate(): Pose = follower.pose

    fun setPoseEstimate(pose: Pose) {
        follower.pose = pose
    }
    // In DriveHardware.kt
    @JvmStatic
    fun updatePoseEstimate(aprilTagPose: Pose?, totalLatency: Double, dev: DoubleArray) {
        val deadWheelPose = getPoseEstimate()

        // Pass the delta or just predict
        filter.predict(deadWheelPose)
        MyTelemetry.addData("vel", follower.velocity.magnitude)
        if (aprilTagPose!=null) {
            MyTelemetry.addData("outlier", filter.isOutlier(follower.pose, aprilTagPose))
            MyTelemetry.addData("delta",
                Vector(aprilTagPose.x-follower.pose.x,
                    aprilTagPose.y-follower.pose.y)
                    .magnitude
            )
        }
        if (aprilTagPose != null && !filter.isOutlier(follower.pose, aprilTagPose)
            && !(follower.velocity.magnitude>10)&&!(follower.angularVelocity>1)) {
            // Use the actual standard deviation from Limelight if available
            // result.stddevMt1[0] is X dev, [1] is Y dev

//            val quality = if (dev[0] > 0) 1.0 / (dev[0] + 1.0) else 1.0
            filter.update(aprilTagPose)
        }
        else{
            filter.update(getPoseEstimate())
        }
        val fusedPose = filter.getPose().setHeading(getPoseEstimate().heading)
        setPoseEstimate(fusedPose)


        // IMPORTANT: Get the full fused pose including heading


        // Only update the follower if the correction is significant
        // to avoid "fighting" the internal integration of Pedro Pathing
//        if (aprilTagPose != null&& !filter.isOutlier(aprilTagPose)) {
//        }
    }

    override fun postInit() {
        setPoseEstimate(Pose(72.0,72.0,0.0))
        Drawing.init()
        filter.reset(getPoseEstimate())
    }

    override fun postUpdate() {
//        // Fetch Limelight data and pass to our update method
//        val result: LLResult? = LimeLightVars.result
//        MyTelemetry.addData("pre Robot Pose", getPoseEstimate())
////        var pose = pose3DMetersToInches(result.botpose)
//
////        MyTelemetry.addData("LL Pose with Offsets", poseWithOffsets.toString())
//        if (result != null && result.isValid()) {
//            var pose = pose3DMetersToInches(result.botpose)
////            MyTelemetry.addData("stdDiv x: ", result.stddevMt1[0]* 39.3701)
////            MyTelemetry.addData("stdDiv y: ", result.stddevMt1[1]* 39.3701)
////            MyTelemetry.addData("stdDiv yaw: ", result.stddevMt1[5])
////            MyTelemetry.addData("MT1 pose: ", pose)
////            MyTelemetry.addData("MT1 field pose: ", pose3dToPose(pose))
//            MyTelemetry.addData("MT1 pose: ", pose)
//            val llPose = pose3dToPose(pose)
////                val llPose = pose3dToPose(result.botpose)
//            MyTelemetry.addData("LL Pose TRU E", result.botpose)
////                MyTelemetry.addData("LL Pose", llPose.toString())
//            val poseWithOffsets = addOffsets(llPose)
//            LimeLightVars.llPose = poseWithOffsets
//            MyTelemetry.addData("LL Pose with Offsets", poseWithOffsets.toString())
//            updatePoseEstimate(poseWithOffsets, result.timestamp, result.stddevMt1)
////            Drawing.drawDebug(pose3dToPose(pose))
//        }
//        else{
//            updatePoseEstimate(null, 0.0, DoubleArray(6))
//
//        }
        Drawing.drawDebug(follower)
//
        // Update target vector for automated scoring
        vectorFromTarget = getPoseEstimate().asVector.minus(RobotVars.goalPos)
    }
}

internal object Drawing {
    const val ROBOT_RADIUS: Double = 9.0 // woah
    private val panelsField = field

    private val robotLook = Style(
        "", "#3F51B5", 0.75
    )
    private val llLook = Style(
        "", "#FFBF00", 0.75
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
    fun drawDebug(follower: Follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook)
            val closestPoint =
                follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue())
            drawRobot(
                Pose(
                    closestPoint.getX(),
                    closestPoint.getY(),
                    follower.getCurrentPath()
                        .getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())
                ), robotLook
            )
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook)
        drawRobot(follower.getPose(), historyLook)

        sendPacket()
    }
    fun drawDebug(follower: Follower, llPose: Pose) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook)
            val closestPoint =
                follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue())
            drawRobot(
                Pose(
                    closestPoint.getX(),
                    closestPoint.getY(),
                    follower.getCurrentPath()
                        .getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())
                ), robotLook
            )
        }
        drawRobot(llPose, llLook)
        drawPoseHistory(follower.getPoseHistory(), historyLook)
        drawRobot(follower.getPose(), historyLook)

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
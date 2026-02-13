package org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.bylazar.gamepad.Gamepad
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import com.qualcomm.hardware.limelightvision.LLResult
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode.runtime
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.Drawing.drawLine
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.startingPose
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight.rotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight.updateLL
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getTargetAngle
import org.firstinspires.ftc.teamcode.Util.DriftKalmanFilter
import org.firstinspires.ftc.teamcode.Util.Util.pose3DMetersToInches
import org.firstinspires.ftc.teamcode.Util.Util.pose3dToPose
import kotlin.math.abs

@Configurable
object DriveHardware : Component {
    @JvmField var driftFilterX = DriftKalmanFilter(
        0.0,
        10.0,
        0.1
    )
    @JvmField var driftFilterY = DriftKalmanFilter(
        0.0,
        10.0,
        0.1
    )
    var lastTime = 0.0


    fun getPoseEstimate(): Pose = follower.pose

    fun setPoseEstimate(pose: Pose) {
        follower.pose = pose
    }

    /**
     * Computes measured drift (bias) between odometry position and camera position.
     *
     * @param odoPose double[3] = {odoX, odoY, odoHeading}
     * @param camPose double[3] = {camX, camY, camHeading}
     * @return double[3] = {driftX, driftY, driftHeading}
     */
    fun computeMeasuredDrift(odoPose: Pose, camPose: Pose): DoubleArray {

        val driftX = odoPose.x - camPose.x
        val driftY = odoPose.y - camPose.y
        val driftHeading = normalizeAngle(odoPose.heading - camPose.heading)

        return doubleArrayOf(driftX, driftY, driftHeading)
    }

    /**
     * Normalize an angle to the range [-pi, pi].
     *
     * @param angle Angle in radians.
     * @return Equivalent angle normalized to [-pi, pi].
     */
    private fun normalizeAngle(angle: Double): Double {
        var angle = angle
        while (angle > Math.PI) angle -= 2.0 * Math.PI
        while (angle < -Math.PI) angle += 2.0 * Math.PI
        return angle
    }
    // In DriveHardware.kt
    @JvmStatic
    fun updatePoseEstimate(result: LLResult?) {
        var deadWheelPose = getPoseEstimate()
        if (lastTime==0.0) lastTime = runtime
        var now = runtime
        var dt = now - lastTime
        lastTime = now

        driftFilterX.predict(dt);
        driftFilterY.predict(dt);

        if (result != null && result.isValid) {
            MyTelemetry.addData("MT2  X",result.botpose_MT2.position.x)
            MyTelemetry.addData("MT2  Y",result.botpose_MT2.position.y)
        val (measuredDriftX, measuredDriftY, _) = computeMeasuredDrift(
            deadWheelPose,
            pose3dToPose(pose3DMetersToInches(result.botpose_MT2)))
        var (visionNoiseVarianceX,visionNoiseVarianceY) = result.stddevMt2.take(2)

        // 3) Correct drift
        driftFilterX.update(measuredDriftX, visionNoiseVarianceX)
        driftFilterY.update(measuredDriftY, visionNoiseVarianceY)
        }

        follower.pose.plus(Pose(driftFilterX.driftEstimate,
            driftFilterY.driftEstimate, follower.heading))
    }

    override fun postInit() {
        setPoseEstimate(startingPose)
        Drawing.init()
        driftFilterX.reset(0.0,3.0)
        driftFilterY.reset(0.0,3.0)
    }

    // Add this variable at the top of your DriveHardware object
    var isHolding = false

    override fun postUpdate() {
        val stickThreshold = 0.05
        val velocityThreshold = 0.5
        val angularVelocityThreshold = 0.05

        val sticksAtRest = abs(Gamepads.gamepad1.leftStickY.get()) < stickThreshold &&
                abs(Gamepads.gamepad1.leftStickX.get()) < stickThreshold &&
                abs(Gamepads.gamepad1.rightStickX.get()) < stickThreshold

        val robotStopped = follower.velocity.magnitude < velocityThreshold &&
                follower.angularVelocity < angularVelocityThreshold

        if (sticksAtRest) {
            if (robotStopped && !isHolding) {
                follower.holdPoint(follower.pose)
                isHolding = true
            }

            MyTelemetry.addData("Drive State", if (isHolding) "Holding Pose" else "Decelerating")
        } else {
            if (isHolding || follower.isBusy) {
                follower.breakFollowing()
                follower.startTeleopDrive()
                isHolding = false
            }
            MyTelemetry.addData("Drive State", "Teleop Driving")
        }

//        vectorFromTarget = Vector(0.0,0.0)
        vectorFromTarget.setOrthogonalComponents(getPoseEstimate().asVector.minus(RobotVars.goalPos).xComponent, getPoseEstimate().asVector.minus(RobotVars.goalPos).yComponent)
        Drawing.drawVector(vectorFromTarget, follower.pose)
        val robotPose = getPoseEstimate()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)
        val turretPose = Pose(robotPose.asVector.plus(rotatedOffset).xComponent, robotPose.asVector.plus(rotatedOffset).xComponent, Math.toRadians(getTargetAngle()))

        Drawing.drawDebug(follower, turretPose)

    }
    fun addOffsets(pose: Pose): Pose {
//        return pose
        LimeLightVars.centerOfRotationOffset.rotateVector(pose.heading)
        return pose
            .minus(Pose(centerOfRotationOffset.xComponent, centerOfRotationOffset.yComponent))
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
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    fun drawLine(pose1: Pose, pose2:Pose, style: Style) {
//        val points = path.getPanelsDrawingPoints()

//        for (i in points[0]!!.indices) {
//            for (j in points.indices) {
//                if (java.lang.Double.isNaN(points[j]!![i])) {
//                    points[j]!![i] = 0.0
//                }
//            }
//        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose1.x, pose1.y)
        panelsField.line(pose2.x, pose2.y)
    }

fun drawVector(vec: Vector, basePose: Pose) {
    val endX = basePose.x + vec.xComponent
    val endY = basePose.y + vec.yComponent
    panelsField.setStyle(llLook)
    panelsField.moveCursor(basePose.x, basePose.y)
    panelsField.line(endX, endY)
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
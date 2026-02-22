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
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.startingPose
//import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight.rotationOffset
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
    var isHolding = false
        private set

    fun getPoseEstimate(): Pose = follower.pose
    fun setPoseEstimate(pose: Pose) { follower.pose = pose }

    override fun postInit() {
        setPoseEstimate(startingPose)
        Drawing.init()
    }

    override fun postUpdate() {
        val stickThreshold = 0.05
        val velocityThreshold = 0.5
        val angularThreshold = 0.05

        // Cache stick values to avoid repeated calls
        val lx = Gamepads.gamepad1.leftStickX.get()
        val ly = Gamepads.gamepad1.leftStickY.get()
        val rx = Gamepads.gamepad1.rightStickX.get()

        val sticksAtRest = abs(lx) < stickThreshold &&
                abs(ly) < stickThreshold &&
                abs(rx) < stickThreshold

        if (sticksAtRest) {
            val vel = follower.velocity
            val robotStopped = vel.magnitude < velocityThreshold &&
                    abs(follower.angularVelocity) < angularThreshold

            if (robotStopped && !isHolding) {
                follower.holdPoint(follower.pose)
                isHolding = true
            }
            MyTelemetry.addData("Drive State", if (isHolding) "Holding" else "Decelerating")
        } else {
            // Only reset if we were previously holding or Pedro is running a path
            if (isHolding || follower.isBusy) {
                follower.breakFollowing()
                follower.startTeleopDrive()
                isHolding = false
            }
            MyTelemetry.addData("Drive State", "Teleop")
        }

        Drawing.drawDebug(follower)
    }
}
internal object Drawing {
    const val ROBOT_RADIUS: Double = 9.0
    private val panelsField = field

    private val robotLook = Style("", "#3F51B5", 0.75)
    private val llLook = Style("", "#FFBF00", 0.75)
    private val historyLook = Style("", "#4CAF50", 0.75)

    fun init() {
        panelsField.setOffsets(presets.PEDRO_PATHING)
    }

    fun drawDebug(follower: Follower, llPose: Pose? = null) {
        val currentPath = follower.currentPath

        if (currentPath != null) {
            drawPath(currentPath, robotLook)
            val t = currentPath.closestPointTValue
            val closestPoint = follower.getPointFromPath(t)

            // Draw Ghost Robot at Target
            drawRobot(Pose(closestPoint.x, closestPoint.y, currentPath.getHeadingGoal(t)), robotLook)
        }

        if (llPose != null) drawRobot(llPose, llLook)

        drawPoseHistory(follower.poseHistory, historyLook)
        drawRobot(follower.pose, historyLook)
        panelsField.update()
    }

    fun drawRobot(pose: Pose?, style: Style = robotLook) {
        if (pose == null || pose.x.isNaN() || pose.y.isNaN()) return

        with(panelsField) {
            setStyle(style)
            moveCursor(pose.x, pose.y)
            circle(ROBOT_RADIUS)

            // Calculate heading line without creating a new Vector object if possible
            val heading = pose.heading
            val cos = Math.cos(heading)
            val sin = Math.sin(heading)

            // Draw heading line from center to edge
            moveCursor(pose.x + (cos * ROBOT_RADIUS * 0.5), pose.y + (sin * ROBOT_RADIUS * 0.5))
            line(pose.x + (cos * ROBOT_RADIUS), pose.y + (sin * ROBOT_RADIUS))
        }
    }

    fun drawPath(path: Path, style: Style) {
        val points = path.getPanelsDrawingPoints() ?: return
        panelsField.setStyle(style)

        // direct access to array indices is faster than nested iterators
        val xPoints = points[0]!!
        val yPoints = points[1]!!

        for (i in 0 until xPoints.size - 1) {
            panelsField.moveCursor(xPoints[i], yPoints[i])
            panelsField.line(xPoints[i+1], yPoints[i+1])
        }
    }

    fun drawPoseHistory(poseTracker: PoseHistory, style: Style = historyLook) {
        panelsField.setStyle(style)
        val xPos = poseTracker.xPositionsArray
        val yPos = poseTracker.yPositionsArray

        for (i in 0 until xPos.size - 1) {
            panelsField.moveCursor(xPos[i], yPos[i])
            panelsField.line(xPos[i + 1], yPos[i + 1])
        }
    }
}
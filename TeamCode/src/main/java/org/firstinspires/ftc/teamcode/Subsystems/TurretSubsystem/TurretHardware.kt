package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.offsetLL
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.p
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.servoOffset
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.servoRange
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.state
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.targetPosition
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware.transferMotor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.deltaVec
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.reducer
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.sin
import kotlin.math.cos

object TurretHardware : Component {
    // Lazy initialize to avoid null refs during object creation
    private val s1 by lazy { ServoEx("turretServo1", 0.0) }
    private val s2 by lazy { ServoEx("turretServo2", 0.0) }
    private val motor by lazy { transferMotor.value }

    // --- CACHED VALUES (The "Snapshot") ---
    private var cachedEncoderPos = 0.0
    private var cachedServoPos = 0.0
    private var cachedVelocity = 0.0
    private var lastSetPos = -1.0 // Track what we LAST sent to the hub

    /**
     * Read all hardware ONCE at the start of the loop.
     * This eliminates redundant I2C/USB overhead.
     */
    override fun preUpdate() {
        cachedEncoderPos = motor.currentPosition.toDouble()
        cachedVelocity = motor.velocity
        cachedServoPos = s1.position
    }

    fun setPosition(pos: Double) {
        // Optimization: Only write to hardware if the change is significant (> 0.1%)
        // This keeps the Control Hub's command buffer clear.
        if (Math.abs(lastSetPos - pos) > 0.0001) {
            s1.position = pos
            s2.position = pos
            lastSetPos = pos
        }
    }

    fun setAngle(angle: Double) {
        // Multiplication is slightly faster than division; pre-calc 1/servoRange if possible
        val pos = (angle + servoOffset + (servoRange * 0.5)) / servoRange
        setPosition(pos)
    }

    fun centerApriltag(): Double {
        val res = result ?: return 0.0
        if (!res.isValid) return 0.0

        MyTelemetry.addData("tx", res.tx)
        // Convert tx to radians and apply gain p
        return Math.toRadians(res.tx * p)
    }

    // --- GETTERS (Using Cached Data) ---
    fun getPosition(): Double = -(cachedEncoderPos - offset)
    fun getPositionServo(): Double = cachedServoPos
    fun getVel(): Double = cachedVelocity
    fun getAngle(): Double = ((getPosition() / 8192.0) * 360.0) / 5.0
    fun getTargetAngle(): Double = cachedServoPos * 360.0

    fun getGlobalHeading(): Double {
        val baseHeading = -Math.toDegrees(DriveHardware.getPoseEstimate().heading)
        return wrap360(baseHeading + getAngle())
    }

    /**
     * Normalizes an angle to [-PI, PI] using atan2 for O(1) performance.
     * Much faster than while loops for large angles.
     */
    private fun fastNormalizeRad(radians: Double): Double {
        return atan2(sin(radians), cos(radians))
    }

    fun update(targetOffset: Double) {
        // 1. Snapshot robot state to local variables
        val robotHeadingRad = follower.heading.rad.value + (follower.angularVelocity * loopTime / 1000.0)
        val globalTargetAngleRad = deltaVec.theta - targetOffset

        // 2. Compute local target with Limelight adjustment
        // Only adjust Limelight offset if the robot is relatively stable
        if (cachedVelocity < 1.0 && follower.velocity.magnitude < 1.0) {
            offsetLL -= centerApriltag()
        }
        else{
            offsetLL *= reducer
        }

        var localAngleRad = globalTargetAngleRad - robotHeadingRad + offsetLL

        // 3. Normalize to shortest path
        localAngleRad = fastNormalizeRad(localAngleRad)

        // 4. Convert and Clamp
        val localAngleDeg = Math.toDegrees(localAngleRad)
        val finalAngle = clamp(localAngleDeg, -servoRange * 0.5, servoRange * 0.5)

        setAngle(finalAngle)

        // 5. Minimal Telemetry (Avoid heavy string concat in the main update)
        MyTelemetry.addData("Turret Local Target", localAngleDeg)
    }

    override fun preInit() {
        lastSetPos = -1.0
        state = TurretState.TrackingAprilTags
        targetPosition = 180.0
    }

    fun zeroEnc() {
        offset = motor.currentPosition.toDouble()
    }

    override fun postUpdate() {
        // Telemetry uses cached values to avoid extra IO hits
        MyTelemetry.addData("Turret Deg", getAngle())
        MyTelemetry.addData("Turret Servo Pos", cachedServoPos)
        MyTelemetry.addData("Turret State", state)
        MyTelemetry.addData("Turret Global", getGlobalHeading())
    }
}
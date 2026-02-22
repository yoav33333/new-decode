package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot.normalizePower
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.controlledSpeed
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.deltaThreshold
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.dy
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.kv
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.ks
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxPos
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minPos
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.mul
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.runShooter
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.shootPowLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.targetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.veloControl
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.math.*

object ShooterHardware : Component {
    val shooterMotor1 = lazy { MotorEx("High shooter") }

    val shooterMotor2 = lazy { MotorEx("Low shooter").reversed() }

    val hoodServo1 = lazy { ServoEx("Hood servo") }
    // Accessing .value on a lazy property repeatedly is a function call.
    // We'll cache these in postInit for faster access.
    private lateinit var motor1: MotorEx
    private lateinit var motor2: MotorEx
    private lateinit var hood: ServoEx

    private var lastPower = 0.0
    private var lastHoodPos = -1.0
    private val IO_THRESHOLD = 0.001 // Smallest change worth sending to hardware

    private var currentVelocity = 0.0
    private var hoodTarget = 0.0

    override fun postInit() {
        lastHoodPos = -1.0
        lastPower = -2.0
        motor1 = shooterMotor1.value
        motor2 = shooterMotor2.value
        hood = hoodServo1.value
        setPower(0.0, force = true)
    }

    // --- Optimized Getters (Read once per loop) ---

    fun getVelocity(): Double = currentVelocity

    // --- Optimized Setters (Write-Skipping) ---

    fun setPower(power: Double, force: Boolean = false) {
        val clampedPower = normalizePower(power)
        if (force || abs(clampedPower - lastPower) > IO_THRESHOLD) {
            motor1.power = clampedPower
            motor2.power = clampedPower
            lastPower = clampedPower
        }
    }

    fun setHoodPosition(position: Double) {
        val clampedPos = clamp(position, 0.0, 0.52)
        if (abs(clampedPos - lastHoodPos) > IO_THRESHOLD) {
            hood.position = clampedPos
            lastHoodPos = clampedPos
        }
    }

    fun atTargetVelocity(): Boolean = abs(currentVelocity - targetVelocity) < deltaThreshold

    /**
     * Logic calculation - No Hardware IO here
     */
    fun update() {
        // Cache smartDist locally to ensure consistency across calculations
        val distance = clamp(smartDist, 62.1, 162.0)

        targetVelocity = shootPowLUT.get(distance)
        hoodTarget = hoodLUT.get(minPos + distance)

        // Motor Control Logic
        if (targetVelocity < 1.0) {
            setPower(0.0)
        } else {
            val feedForward = (kv * targetVelocity) + ks
            val feedback = veloControl.calculate(targetVelocity, currentVelocity)
            setPower(feedback + feedForward)
        }

        setHoodPosition(hoodTarget)
        TurretHardware.update(0.0)
    }

    override fun preUpdate() {
        // Perform all hardware READS at the start of the loop
        currentVelocity = -motor1.velocity
    }

    override fun postUpdate() {
        update()

        // Telemetry is also IO, but usually handled by a separate thread or bulk-buffered
        MyTelemetry.addData("Shooter velocity", currentVelocity)
        MyTelemetry.addData("Shooter target", targetVelocity)
        MyTelemetry.addData("Hood Servo Pos", lastHoodPos)
    }
}
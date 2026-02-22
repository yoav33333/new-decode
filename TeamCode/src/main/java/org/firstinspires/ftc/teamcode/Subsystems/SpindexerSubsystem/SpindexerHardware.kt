package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.ThermalEquilibrium.homeostasis.Utils.Timer
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.button
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.shooterMotor2
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.fixSpindex
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.MulEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.degreesPerSlot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.delayMul
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.greenRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.intakeSlot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.offsetEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.purpleRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.state
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.stuckTimeStart
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.wasStuck
import org.firstinspires.ftc.teamcode.Util.AxonEncoder
import org.firstinspires.ftc.teamcode.Util.FilteredColorSensor
import org.firstinspires.ftc.teamcode.Util.InputChannel
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.Util
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import kotlin.math.abs
import kotlin.math.max

@Configurable
object SpindexerHardware : Component {
    // --- Hardware References ---
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { ServoEx("spindex", cacheTolerance = 0.0) }
//    @JvmField var transferSensor = lazy { InputChannel(hardwareMap, "tm").get() }
    @JvmField var colorSensor1 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Lcolor")) }
    @JvmField var colorSensor2 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Rcolor")) }

    val timer = Timer()
    var tracker = SpindexerTracker()

    // --- I/O Caching Variables ---
    private var cachedEncoderPos = 0.0
    private var cachedVelocity = 0.0
    private var cachedTransferState = false
    private var lastCommandedServoPos = -1.0 // Initialize with impossible value
    private val SERVO_EPSILON = 0.001 // Minimum change required to trigger a write

    @JvmField var currentSteps = 2

    // --- Optimized Getters (Using Caches) ---
    fun getSpindexerPos(): Double = -(offsetEnc + cachedEncoderPos) / 8192 * 360
    fun getSpindexerVel(): Double = (cachedVelocity) / 8192 * 360
    fun getVel(): Double = cachedVelocity
    fun hasBallInTransfer(): Boolean = !cachedTransferState

    // --- Logic Methods ---
    fun resetSpindexerEnc() {
        offsetEnc = -shooterMotor2.value.currentPosition
    }

    fun isFull(): Boolean = tracker.isFull()
    fun isEmpty(): Boolean = tracker.isEmpty()
    fun isAtTarget(): Boolean = targetPosition == getPosition()

    fun getColorInIntake(): SpindexerSlotState {
        // Localizing sensor reads
        val s1 = colorSensor1.value
        val s2 = colorSensor2.value

        var hsv = s1.getHSV()
        if (purpleRange.inRange(hsv)) return SpindexerSlotState.PURPLE
        if (greenRange.inRange(hsv)) return SpindexerSlotState.GREEN

        hsv = s2.getHSV()
        if (purpleRange.inRange(hsv)) return SpindexerSlotState.PURPLE
        if (greenRange.inRange(hsv)) return SpindexerSlotState.GREEN

        return SpindexerSlotState.EMPTY
    }

    fun checkIntakeColorAndUpdate(): Boolean {
        val color = getColorInIntake()
        return if (color != SpindexerSlotState.EMPTY) {
            tracker.set(intakeSlot, color)
            true
        } else false
    }

    fun checkIntakeColorAndUpdateAuto(): Boolean {
        val color = getColorInIntake()
        tracker[intakeSlot] = if (color != SpindexerSlotState.EMPTY) color else SpindexerSlotState.PURPLE
        return color != SpindexerSlotState.EMPTY
    }

    /**
     * Optimized Write: Only communicates with the servo if the position
     * has changed significantly.
     */
    fun setPosition(position: Double) {
        if (abs(position - lastCommandedServoPos) > SERVO_EPSILON) {
            spindexerServo.value.position = position
            lastCommandedServoPos = position
        }
    }

    fun getTargetPosition(): Double = lastCommandedServoPos

    fun getPosition(): Double {
        return wrap360(-spindexerEncoder.value.getPosition() * MulEnc + SpindexerVars.offsetEnc)
    }

    fun moveStateToPosition(color: SpindexerSlotState, pos: Int): Boolean {
        val steps = tracker.stepsToState(color, pos) ?: return false
        delayMul = max(1.0, steps / 1.5)
        rotate(steps)
        colorSensor1.value.resetFilter()
        colorSensor2.value.resetFilter()
        return true
    }

    fun moveEmptyToIntakePosition(): Boolean = moveStateToPosition(SpindexerSlotState.EMPTY, SpindexerVars.intakeSlot)

    fun moveColorToTransferPosition(color: SpindexerSlotState): Boolean {
        return if (moveStateToPosition(color, SpindexerVars.transferSlot)) {
            tracker[SpindexerVars.transferSlot] = SpindexerSlotState.EMPTY
            true
        } else false
    }

    fun rotate(steps: Int) {
        val newStepPosition = currentSteps + steps
        if (newStepPosition > 4 || newStepPosition < 2) return

        currentSteps = newStepPosition
        tracker.move(steps)
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
    }

    fun resetSpindexer() {
        currentSteps = 2
        tracker.setPose(currentSteps)
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
        setPosition(angleToServoPos(targetPosition))
    }

    fun isAtTargetPosition(): Boolean {
        return abs(getSpindexerPos() / 2 + 2 * SpindexerVars.degreesPerSlot - targetPosition) < 15
    }

    fun isStuck(): Boolean = abs(cachedVelocity) < 2
    fun timeStuck(): Double = timer.currentTime() - stuckTimeStart
    fun angleToServoPos(angle: Double): Double = angle / SpindexerVars.maxRotation

    override fun postInit() {lastCommandedServoPos = -1.0}
    override fun postStartButtonPressed() {}

    /**
     * Main Loop execution.
     * Reads all sensors once, processes logic, then writes once.
     */
    private fun updateSystem() {
        // 1. READ (Slow I/O)
        cachedEncoderPos = shooterMotor2.value.currentPosition.toDouble()
        cachedVelocity = shooterMotor2.value.velocity
        cachedTransferState = true

        // 2. LOGIC
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset

        if (state == State.RUN) {
            setPosition(angleToServoPos(targetPosition))
        }

        // Stuck detection logic
        if (isStuck()) {
            if (!wasStuck) {
                stuckTimeStart = timer.currentTime()
                wasStuck = true
            }
        } else {
            wasStuck = false
        }

        // 3. TELEMETRY (Combined for speed)
        MyTelemetry.addData("Spin Pos/Vel", "%.1f / %.1f".format(getSpindexerPos(), getSpindexerVel()))
        MyTelemetry.addData("Target/Step", "%.1f / %d".format(targetPosition, currentSteps))
        MyTelemetry.addData("Stats", "Full: ${tracker.isFull()} | Ball: ${hasBallInTransfer()}")
    }

    override fun postUpdate() = updateSystem()
    override fun postWaitForStart() = updateSystem()
}
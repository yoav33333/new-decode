package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.ThermalEquilibrium.homeostasis.Utils.Timer
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.shooterMotor2
//import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.fixSpindex
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.mulEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.degreesPerSlot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.greenRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.startIntakingStep
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.offsetEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.purpleRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.state
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.stuckTimeStart
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.wasStuck
import org.firstinspires.ftc.teamcode.Util.AxonEncoder
import org.firstinspires.ftc.teamcode.Util.FilteredColorSensor
import org.firstinspires.ftc.teamcode.Util.NewSpindexerTracker
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import kotlin.math.abs

@Configurable
object SpindexerHardware : Component {
    // --- Hardware References ---
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { ServoEx("spindex", cacheTolerance = 0.0) }
//    @JvmField var transferSensor = lazy { InputChannel(hardwareMap, "tm").get() }
    @JvmField var colorSensor1 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Lcolor")) }
    @JvmField var colorSensor2 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Rcolor")) }

    val timer = Timer()
    var tracker = NewSpindexerTracker()

    // --- I/O Caching Variables ---
    private var cachedEncoderPos = 0.0
    private var cachedVelocity = 0.0
    private var cachedTransferState = false
    private var lastCommandedServoPos = -1.0 // Initialize with impossible value
    private val SERVO_EPSILON = 0.001 // Minimum change required to trigger a write

    @JvmField var currentSteps = 0

    // --- Optimized Getters (Using Caches) ---
    fun getSpindexerPos(): Double = -(offsetEnc + cachedEncoderPos) / 8192 * 360
    fun getSpindexerVel(): Double = (cachedVelocity) / 8192 * 360
    fun getVel(): Double = cachedVelocity
    fun hasBallInTransfer(): Boolean = !cachedTransferState

    // --- Logic Methods ---
    fun resetSpindexerEnc() {
        offsetEnc = -shooterMotor2.value.currentPosition
    }
    fun makeCurrentPosCorrect(){
        offsetEnc = -cachedEncoderPos - ((currentSteps-5)*degreesPerSlot*2*8192/360/mulEnc)
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
            tracker.insert(color)
            update()
//            if (isFull()){
//                preShoot()
//            }
            true
        } else false
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
        return wrap360(-spindexerEncoder.value.getPosition() * mulEnc + SpindexerVars.offsetEnc)
    }
    fun preShoot(){
        tracker.preShoot(RobotVars.randomization)
        update()
    }
    fun shoot(){
        tracker.shoot()
        update()
    }
    fun update(): Boolean {
        val steps = tracker.getCurrentPos()
        setStep(steps)
        colorSensor1.value.resetFilter()
        colorSensor2.value.resetFilter()
        return true
    }


    fun rotate(steps: Int) {
        val newStepPosition = currentSteps + steps
        if (newStepPosition > 5 || newStepPosition < 0) return

        currentSteps = newStepPosition
        tracker.rotate(steps)
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
    }
    fun setStep(steps: Int) {
        val newStepPosition = steps
        if (newStepPosition > 5 || newStepPosition < 0) return

        currentSteps = newStepPosition
//        tracker.rotate(steps)
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
    }

    fun resetSpindexer() {
        currentSteps = startIntakingStep
        tracker.init()
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
        setPosition(angleToServoPos(targetPosition))
    }

    fun isAtTargetPosition(): Boolean {
        return abs(getSpindexerPos() / 2*mulEnc + startIntakingStep * degreesPerSlot +offset - targetPosition) < 15
    }

    fun isStuck(): Boolean = abs(cachedVelocity) < 2
    fun timeStuck(): Double = timer.currentTime() - stuckTimeStart
    fun angleToServoPos(angle: Double): Double = angle / SpindexerVars.maxRotation

    override fun postInit() {lastCommandedServoPos = -1.0}
    override fun postStartButtonPressed() {
//        button { isStuck() && timeStuck()>0.3 && !isAtTargetPosition() && state != State.FIX}.whenBecomesTrue(fixSpindexSeq)
    }

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
        MyTelemetry.addData("at target", isAtTargetPosition())
        MyTelemetry.addData("offset ", getSpindexerPos()*mulEnc / 2 + startIntakingStep * SpindexerVars.degreesPerSlot +offset - targetPosition)
        MyTelemetry.addData("move Spin ", getSpindexerPos()*mulEnc / 2 + startIntakingStep * SpindexerVars.degreesPerSlot+offset)
        MyTelemetry.addData("spin state ", tracker.toString())
    }

    override fun postUpdate() = updateSystem()
    override fun postWaitForStart() = updateSystem()
}
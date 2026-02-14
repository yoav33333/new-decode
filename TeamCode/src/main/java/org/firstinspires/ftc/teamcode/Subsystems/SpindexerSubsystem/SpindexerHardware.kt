package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

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
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
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
object SpindexerHardware: Component {
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { ServoEx("spindex", cacheTolerance = 0.0) }
    @JvmField var transferSensor = lazy { InputChannel(hardwareMap, "tm").get() }
    @JvmField var colorSensor1 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Lcolor")) }
    @JvmField var colorSensor2 = lazy { FilteredColorSensor(hardwareMap.get(RevColorSensorV3::class.java, "Rcolor")) }

    var tracker = SpindexerTracker()

    // We need to track the absolute step position here to calculate angle correctly.
    // Range is expected to be -2 to 2 based on your description.
    @JvmField var currentSteps = 2
    fun resetSpindexerEnc(){
        offsetEnc = -shooterMotor2.value.currentPosition
    }
    fun getSpindexerPos(): Double{
        return -(offsetEnc+shooterMotor2.value.currentPosition)/8192*360
    }
    fun getSpindexerVel(): Double{
        return (shooterMotor2.value.velocity)/8192*360
    }
    fun isFull(): Boolean {
        return tracker.isFull()
    }
    fun isEmpty(): Boolean{
        return tracker.isEmpty()
    }
    fun isAtTarget(): Boolean = targetPosition == getPosition()

    fun getColorInIntake(): SpindexerSlotState {
        var hsv = colorSensor1.value.getHSV()

        MyTelemetry.addData("color sensor 1", hsv.toString())
        if (purpleRange.inRange(hsv)) {
            MyTelemetry.addData("Intake Color Sensor 1", "Purple")
            return SpindexerSlotState.PURPLE
        } else if (greenRange.inRange(hsv)) {
            MyTelemetry.addData("Intake Color Sensor 1", "Green")
            return SpindexerSlotState.GREEN
        } else {
            var hsv = colorSensor2.value.getHSV()
            MyTelemetry.addData("color sensor 2", hsv.toString())
            if (purpleRange.inRange(hsv)) {
                MyTelemetry.addData("Intake Color Sensor 2", "Purple")
                return SpindexerSlotState.PURPLE
            } else if (greenRange.inRange(hsv)) {
                MyTelemetry.addData("Intake Color Sensor 2", "Green")
                return SpindexerSlotState.GREEN
            } else {
                MyTelemetry.addData("Intake Color Sensor 2", "Empty")
                return SpindexerSlotState.EMPTY
            }
        }
    }

    fun checkIntakeColorAndUpdate(): Boolean{
        val color = getColorInIntake()
        if (color != SpindexerSlotState.EMPTY){
            tracker.set(intakeSlot, color)
            return true
        }
        return false
    }
    fun checkIntakeColorAndUpdateAuto(): Boolean{
        val color = getColorInIntake()
        if (color != SpindexerSlotState.EMPTY){
            tracker[intakeSlot] = color
            return true
        }
        tracker[intakeSlot] = SpindexerSlotState.PURPLE
        return false
    }

    fun setPosition(position: Double) {
        spindexerServo.value.position = position
    }
    fun getTargetPosition(): Double{
        return spindexerServo.value.position
    }

    fun getPosition(): Double {
        return wrap360(-spindexerEncoder.value.getPosition()*MulEnc +SpindexerVars.offsetEnc)
    }

    fun moveStateToPosition(color: SpindexerSlotState, pos: Int) : Boolean{
        // tracker.stepsToState now returns the shortest LEGAL move within servo limits
        val steps = tracker.stepsToState(color, pos)

        if (steps == null) {
            // If we can't reach the state (e.g., servo limit reached), we might need to reset
            // or we simply can't perform the action.
            return false
        }
        delayMul = max(1.0,steps/1.5)
        rotate(steps)
        colorSensor1.value.resetFilter()
        colorSensor2.value.resetFilter()
        return true
    }

    fun moveEmptyToIntakePosition(): Boolean {
        return moveStateToPosition(SpindexerSlotState.EMPTY, SpindexerVars.intakeSlot)
    }
    fun moveColorToTransferPosition(color: SpindexerSlotState): Boolean {
        if(moveStateToPosition(color, SpindexerVars.transferSlot)){
            tracker[SpindexerVars.transferSlot] = SpindexerSlotState.EMPTY
            return true
        }
        return false
    }
    fun moveColorToTransferPositionOff(color: SpindexerSlotState): Boolean {
        if(moveStateToPosition(color, SpindexerVars.transferSlot+1)){
//            tracker[SpindexerVars.transferSlot] = SpindexerSlotState.EMPTY
            return true
        }
        return false
    }

    fun rotate(steps: Int) {
        // Calculate the new potential position
        val newStepPosition = currentSteps + steps

        // Hard limits: Ensure we don't go beyond -2 or 2 (or whatever your SpindexerTracker limit is)
        if (newStepPosition > 4 || newStepPosition < 2) {
            MyTelemetry.addData("Spindexer Warning", "Rotation limit hit! Requested: $newStepPosition")
            return
        }

        // Update local step counter
        currentSteps = newStepPosition

        // Update the logical tracker so it knows where the "head" is
        tracker.move(steps)

        // Calculate target angle based on absolute steps
        // NO MODULO HERE: We want the servo to go to the actual physical angle
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
    }

    fun resetSpindexer(){
        currentSteps = 2
        tracker.setPose(currentSteps)
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
        setPosition(angleToServoPos(targetPosition))
    }

    fun isAtTargetPosition(): Boolean {
        return abs(getSpindexerPos()/2+2 * SpindexerVars.degreesPerSlot - targetPosition) < 15
//        return true
    }
    fun isStuck(): Boolean {
        return abs(getVel()) < 5
//        return true
    }

    fun getVel(): Double {
        return shooterMotor2.value.velocity
//        return true
    }
    fun hasBallInTransfer(): Boolean {
        return !transferSensor.value.state
    }

    fun angleToServoPos(angle: Double): Double {
        return angle/ SpindexerVars.maxRotation
    }

    override fun postInit() {
        resetingSeq.schedule()

//        spindexerServo.value.position = 0.5
    }

    override fun postStartButtonPressed() {
//        button { !isAtTargetPosition()&& isStuck() && state == State.RUN}.whenBecomesTrue (fixSpindex.value)
    }
    override fun postUpdate() {
        targetPosition = currentSteps * SpindexerVars.degreesPerSlot + SpindexerVars.offset
        if(state == State.RUN)setPosition(angleToServoPos(targetPosition))
        MyTelemetry.addData("Spindexer Position", getSpindexerPos())
        MyTelemetry.addData("Spindexer Vel", getSpindexerVel())
//        MyTelemetry.addData("Spindexer Vol", spindexerEncoder.value.getVoltage())
        MyTelemetry.addData("Spindexer Target angle", targetPosition)
        MyTelemetry.addData("Spindexer at target", isAtTargetPosition())
        MyTelemetry.addData("transfer sensor", hasBallInTransfer())
        MyTelemetry.addData("Spindexer target pos", getTargetPosition())
        MyTelemetry.addData("Spindexer Steps", currentSteps) // Debug current steps
        MyTelemetry.addData("Spindexer state", tracker.toString())
        MyTelemetry.addData("is full", tracker.isFull())
        MyTelemetry.addData("delta", abs(getSpindexerPos()/2+2 * SpindexerVars.degreesPerSlot - targetPosition))
        MyTelemetry.addData("state", state)
        MyTelemetry.addData("balls In spindexer", tracker.getAmount())
        MyTelemetry.addData("Spindexer vel", getVel())
    }
}

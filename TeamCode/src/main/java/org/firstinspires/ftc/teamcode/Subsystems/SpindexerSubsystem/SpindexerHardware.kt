package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.greenRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.intakeSlot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.purpleRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Util.AxonEncoder
import org.firstinspires.ftc.teamcode.Util.FilteredColorSensor
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.Util
import kotlin.math.abs

@Configurable
object SpindexerHardware: Component {
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { ServoEx("spindex") }
    @JvmField var colorSensor1 = lazy { FilteredColorSensor(hardwareMap.get(ColorRangeSensor::class.java, "Lcolor")) }
    @JvmField var colorSensor2 = lazy { FilteredColorSensor(hardwareMap.get(ColorRangeSensor::class.java, "Rcolor")) }

    var tracker = SpindexerTracker()

    // We need to track the absolute step position here to calculate angle correctly.
    // Range is expected to be -2 to 2 based on your description.
    private var currentSteps = 0

    fun isFull(): Boolean {
        return tracker.isFull()
    }
    fun isEmpty(): Boolean{
        return tracker.isEmpty()
    }

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

    fun setPosition(position: Double) {
        spindexerServo.value.position = position
    }
    fun getTargetPosition(): Double{
        return spindexerServo.value.position
    }

    fun getPosition(): Double {
        return spindexerEncoder.value.getPosition()+SpindexerVars.offset
    }

    fun moveStateToPosition(color: SpindexerSlotState, pos: Int) : Boolean{
        // tracker.stepsToState now returns the shortest LEGAL move within servo limits
        val steps = tracker.stepsToState(color, pos)

        if (steps == null) {
            // If we can't reach the state (e.g., servo limit reached), we might need to reset
            // or we simply can't perform the action.
            return false
        }

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

    fun rotate(steps: Int) {
        // Calculate the new potential position
        val newStepPosition = currentSteps + steps

        // Hard limits: Ensure we don't go beyond -2 or 2 (or whatever your SpindexerTracker limit is)
        if (newStepPosition > 5 || newStepPosition < 0) {
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
        currentSteps = 0
        tracker.setPose(currentSteps)
        targetPosition = SpindexerVars.offset
    }

    fun isAtTargetPosition(): Boolean {
        return abs(getPosition() - targetPosition) < 5
    }

    fun angleToServoPos(angle: Double): Double {
        return angle/ SpindexerVars.maxRotation
    }

    override fun postUpdate() {
        setPosition(angleToServoPos(targetPosition))
        MyTelemetry.addData("Spindexer Position", getPosition())
        MyTelemetry.addData("Spindexer Vol", spindexerEncoder.value.getVoltage())
        MyTelemetry.addData("Spindexer Target angle", targetPosition)
        MyTelemetry.addData("Spindexer target pos", getTargetPosition())
        MyTelemetry.addData("Spindexer Steps", currentSteps) // Debug current steps
        MyTelemetry.addData("Spindexer state", tracker.toString())
        MyTelemetry.addData("is full", tracker.isFull())
    }
}

package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.CRServoEx
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

//TODO: Tune PIDF values
//TODO: try hsv
//TODO: implement low pass filter
@Configurable
object SpindexerHardware: Component {
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { ServoEx("spindex") }
    @JvmField var colorSensor1 = lazy { FilteredColorSensor(hardwareMap.get(ColorRangeSensor::class.java, "Lcolor")) }
    @JvmField var colorSensor2 = lazy { FilteredColorSensor(hardwareMap.get(ColorRangeSensor::class.java, "Rcolor")) }
    val tracker = SpindexerTracker()
//    val anglePID = AnglePID(SpindexerVars.p, SpindexerVars.i, SpindexerVars.d, SpindexerVars.f)
//    override fun preInit() {
//        anglePID.reset()
//        anglePID.setTolerance(5.0)
//    }
//    override fun postStartButtonPressed() {
//        colorSensor1.value.gain = 0.2.toFloat()
//        colorSensor2.value.gain = 0.2.toFloat()
////    }
//    fun setGain(gain: Double){
//        colorSensor1.value.gain = gain.toFloat()
//        colorSensor2.value.gain = gain.toFloat()
//    }
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
        val steps = tracker.stepsToState(color, pos)
        if (steps == null) return false
        rotate(steps)
        return true
    }

    fun moveEmptyToIntakePosition(): Boolean {
        return moveStateToPosition(SpindexerSlotState.EMPTY, SpindexerVars.intakeSlot)
    }
    fun moveColorToTransferPosition(color: SpindexerSlotState): Boolean {
        return moveStateToPosition(color, SpindexerVars.transferSlot)
    }
    fun rotate(steps: Int) {
        targetPosition = Util.wrap360(steps * SpindexerVars.degreesPerSlot + targetPosition)
        tracker.rotate(steps)
    }
    fun isAtTargetPosition(): Boolean {
        return abs(getPosition() - targetPosition) < 5
    }
    fun angleToServoPos(angle: Double): Double {
        return angle/2 / SpindexerVars.maxRotation
    }

//    fun updatePid() {
//
//        setPower(
//            controller.calculate(targetPosition, getPosition())
//        )
//    }
    override fun postUpdate() {
//        updatePid()
//        setGain(gain)
        setPosition(angleToServoPos(targetPosition))
        MyTelemetry.addData("Spindexer Position", getPosition())
        MyTelemetry.addData("Spindexer Vol", spindexerEncoder.value.getVoltage())
        MyTelemetry.addData("Spindexer Target angle", targetPosition)
        MyTelemetry.addData("Spindexer target pos", getTargetPosition())
        MyTelemetry.addData("Spindexer state", tracker.toString())
//        MyTelemetry.addData("Spindexer pid",
//            controller.calculate(targetPosition, getPosition())
//        )
    }

}
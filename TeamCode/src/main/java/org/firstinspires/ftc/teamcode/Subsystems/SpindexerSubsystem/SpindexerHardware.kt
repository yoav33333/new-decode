package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.CRServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.controller
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.gain
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.greenRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.intakeSlot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Util.AnglePID
import org.firstinspires.ftc.teamcode.Util.AxonEncoder
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.Util
import kotlin.math.abs

//TODO: Tune PIDF values
//TODO: add color sensor integration
object SpindexerHardware: Component {
    val spindexerEncoder = lazy { AxonEncoder("Abs spin") }
    val spindexerServo = lazy { CRServoEx("spindex") }
    val colorSensor1 = lazy { hardwareMap.get(NormalizedColorSensor::class.java, "Lcolor") }
    val colorSensor2 = lazy { hardwareMap.get(NormalizedColorSensor::class.java, "Rcolor") }
    val tracker = SpindexerTracker()
//    val anglePID = AnglePID(SpindexerVars.p, SpindexerVars.i, SpindexerVars.d, SpindexerVars.f)
//    override fun preInit() {
//        anglePID.reset()
//        anglePID.setTolerance(5.0)
//    }
//    override fun postStartButtonPressed() {
//        colorSensor1.value.gain = 0.2.toFloat()
//        colorSensor2.value.gain = 0.2.toFloat()
//    }
    fun setGain(gain: Double){
        colorSensor1.value.gain = gain.toFloat()
        colorSensor2.value.gain = gain.toFloat()
    }
    fun isFull(): Boolean {
        return tracker.isFull()
    }
    fun isEmpty(): Boolean{
        return tracker.isEmpty()
    }

    fun getColorInIntake(): SpindexerSlotState {
        var colors = colorSensor1.value.normalizedColors
        var red = colors.red
        var blue = colors.blue
        var green = colors.green
        MyTelemetry.addData("Intake Color Sensor 1 Red", red)
        MyTelemetry.addData("Intake Color Sensor 1 Blue", blue)
        MyTelemetry.addData("Intake Color Sensor 1 Green", green)
        if (greenRange.inRange(red, green, blue)) {
            MyTelemetry.addData("Intake Color Sensor 1", "Purple")
            return SpindexerSlotState.PURPLE
        } else if (greenRange.inRange(red, green, blue)) {
            MyTelemetry.addData("Intake Color Sensor 1", "Green")
            return SpindexerSlotState.GREEN
        } else {
            var colors = colorSensor2.value.normalizedColors
            var red = colors.red
            var blue = colors.blue
            var green = colors.green
            MyTelemetry.addData("Intake Color Sensor 2 Red", red)
            MyTelemetry.addData("Intake Color Sensor 2 Blue", blue)
            MyTelemetry.addData("Intake Color Sensor 2 Green", green)
            if (greenRange.inRange(red, green, blue)) {
                MyTelemetry.addData("Intake Color Sensor 2", "Purple")
                return SpindexerSlotState.PURPLE
            } else if (greenRange.inRange(red, green, blue)) {
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
    fun setPower(power: Double) {
        spindexerServo.value.power = power
    }
    fun getPower(): Double{
        return spindexerServo.value.power
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

    fun updatePid() {

        setPower(
            controller.calculate(targetPosition, getPosition())
        )
    }
    override fun postUpdate() {
        updatePid()
        setGain(gain)
        MyTelemetry.addData("Spindexer Position", getPosition())
        MyTelemetry.addData("Spindexer Vol", spindexerEncoder.value.getVoltage())
        MyTelemetry.addData("Spindexer Target", targetPosition)
        MyTelemetry.addData("Spindexer power", getPower())
        MyTelemetry.addData("Spindexer state", tracker.toString())
        MyTelemetry.addData("Spindexer pid",
            controller.calculate(targetPosition, getPosition())
        )
    }

}
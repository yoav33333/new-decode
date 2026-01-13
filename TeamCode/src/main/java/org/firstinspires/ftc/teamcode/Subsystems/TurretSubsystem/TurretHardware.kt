package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.lockOnGoal
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.servoRange
import org.firstinspires.ftc.teamcode.Util.AxonEncoder
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import java.lang.Math.toRadians
import kotlin.math.atan

object TurretHardware: Component {
    val servo1 = lazy { ServoEx("turretServo1") }
    val servo2 = lazy { ServoEx("turretServo2") }
    val turretEncoder = lazy { AxonEncoder("Abs turret") }
    fun setTargetPosition(position: Double) {
        servo1.value.position = position
        servo2.value.position = position
    }
    fun getPosition(): Double {
        return servo1.value.position
    }
    fun getEncoderPosition(): Double {
        return wrap360(turretEncoder.value.getPosition() + offset)
    }
    fun setTargetPositionFromDegrees(degrees: Double) {
        var degrees=degrees
        if(degrees<0||degrees>servoRange){
            //closest to any end
            if((degrees-servoRange)<(360-servoRange)/2){
                degrees=servoRange
            }
            else{
                degrees=0.0
            }
        }
//        var degrees=clamp(degrees, 0.0, servoRange)
        val position = degrees / servoRange
        setTargetPosition(position)
    }
    fun setTargetPositionFromGlobalDegrees(globalDegrees: Double) {
        val baseHeading = toRadians(DriveHardware.getPoseEstimate().heading)
        val turretDegrees = wrap360(globalDegrees - baseHeading)
        setTargetPositionFromDegrees(turretDegrees)
    }
    fun getGlobalHeading(): Double {
        val baseHeading = toRadians(DriveHardware.getPoseEstimate().heading)
        val turretHeading = getEncoderPosition()
        return wrap360(baseHeading + turretHeading)
    }
    fun calcGlobalHeadingToTarget(target: Vector): Double{
        return target.theta
    }

    override fun postUpdate() {
        MyTelemetry.addData("Turret Servo Position", getPosition())
        MyTelemetry.addData("Turret Vol", turretEncoder.value.getVoltage())
        MyTelemetry.addData("Turret Encoder Position", getEncoderPosition())
        MyTelemetry.addData("Turret Global Heading", getGlobalHeading())
    }
}
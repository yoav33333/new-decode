package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.CRServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware.transferMotor
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.Kv
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.angleControl
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.distP
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.encoderMul
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.p
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.servoRange
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.targetPosition
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import java.lang.Math.toRadians

object TurretHardware: Component {
    val servo1 = lazy { CRServoEx("turretServo1") }
    val servo2 = lazy { CRServoEx("turretServo2") }

//    val turretEncoder = lazy { AxonEncoder("Abs turret") }
    fun setPower(power: Double) {
        servo1.value.power = power
        servo2.value.power = power
    }
    fun centerApriltag(){
        val result = result
        if (result== null || !result.isValid)return
        MyTelemetry.addData("new Target",getAngle()-(result.tx*p))
        MyTelemetry.addData("tx",result.tx)
        setTargetPosition(getAngle()-(result.tx*p))
//            (result.botposeAvgDist*distP))-follower.angularVelocity/loopTime
//        /(smartDist*distP)
    }
    fun setTargetPosition(position: Double) {
        targetPosition = position
    }
    fun getPosition(): Double {
        return -transferMotor.value.currentPosition
    }
    fun getAngle():Double{
        return ((getPosition()/8192)*360)/5+Math.toDegrees(follower.heading)
    }
//    fun getEncoderPosition(): Double {
//        return wrap360(getPosition() + offset)
//    }
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
        val turretHeading = getPosition()
        return wrap360(baseHeading + turretHeading)
    }
    fun calcGlobalHeadingToTarget(target: Vector): Double{
        return target.theta
    }

    fun update(){
        setPower(angleControl.calculate(targetPosition, getAngle())-follower.angularVelocity/loopTime*Kv)
    }

    override fun preInit() {
        transferMotor.value.zero()
    }
    override fun postUpdate() {
        centerApriltag()
//        se(targetPosition)
        update()
        MyTelemetry.addData("Turret Servo Position", getPosition())
        MyTelemetry.addData("Turret Servo angle", getAngle())
        MyTelemetry.addData("Turret Servo angle2", ((getPosition()/8192)*360)/5)
//        MyTelemetry.addData("Turret Vol", turretEncoder.value.getVoltage())
        MyTelemetry.addData("Turret Target Position", targetPosition)
//        MyTelemetry.addData("Turret vol", getEncoderPosition())
        MyTelemetry.addData("Turret Global Heading", getGlobalHeading())
    }
}
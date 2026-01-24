package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.ServoEx
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
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.state
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.targetPosition
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import org.firstinspires.ftc.teamcode.Util.Util.wrap360
import java.lang.Math.toRadians

object TurretHardware: Component {
    val servo1 = lazy { ServoEx("turretServo1", 0.0) }
    val servo2 = lazy { ServoEx("turretServo2",0.0) }

//    val turretEncoder = lazy { AxonEncoder("Abs turret") }
    fun setPosition(pos: Double) {
        servo1.value.position = pos
        servo2.value.position = pos
    }
    fun setAngle(angle: Double){
        setPosition((angle+servoRange/2)/servoRange)
    }
    fun centerApriltag(){
        val result = result
        if (result== null || !result.isValid)return
        MyTelemetry.addData("new Target",getAngle()-(result.tx*p)-follower.angularVelocity/loopTime)
        MyTelemetry.addData("tx",result.tx)
        setTargetPosition(getAngle()-(result.tx*p)-follower.angularVelocity.deg.value/loopTime)
//            (result.botposeAvgDist*distP))-follower.angularVelocity/loopTime
//        /(smartDist*distP)
    }
    fun setTargetPosition(position: Double) {
//        var degrees=position
//        if(degrees<-servoRange/2||degrees>servoRange/2){
//            //closest to any end
//            degrees = if (degrees>180) -servoRange/2 else servoRange/2
//        }
        targetPosition = clamp(position, -(servoRange/2-10), servoRange/2-10)
    }
    fun getPosition(): Double {
        return -(transferMotor.value.currentPosition-offset)
    }
    fun getPositionServo(): Double {
        return servo1.value.position
    }
    fun getAngle():Double{
        return ((getPosition()/8192)*360)/5
    }
//    fun getEncoderPosition(): Double {
//        return wrap360(getPosition() + offset)
//    }
//    fun setTargetPositionFromDegrees(degrees: Double) {
//        var degrees=degrees
//        if(degrees<0||degrees>servoRange){
//            //closest to any end
//            if((degrees-servoRange)<(360-servoRange)/2){
//                degrees=servoRange
//            }
//            else{
//                degrees=0.0
//            }
//        }
////        var degrees=clamp(degrees, 0.0, servoRange)
//        val position = degrees / servoRange
//        setTargetPosition(position)
//    }
    fun setTargetPositionFromGlobalDegrees(globalDegrees: Double) {
        val baseHeading = toRadians(DriveHardware.getPoseEstimate().heading)
        val turretDegrees = wrap360(globalDegrees - baseHeading)
        setTargetPosition(turretDegrees)
    }
    fun zeroEnc(){
        offset =  transferMotor.value.currentPosition
    }
    fun getVel(): Double{
        return transferMotor.value.velocity
    }
    fun getGlobalHeading(): Double {
        val baseHeading = -Math.toDegrees(DriveHardware.getPoseEstimate().heading)
        val turretHeading = getAngle()
        return wrap360(baseHeading + turretHeading)
    }
    fun calcGlobalHeadingToTarget(target: Vector): Double{
        return target.theta
    }

    fun update(){
        when(state){
            TurretState.TrackingAprilTags ->{
                centerApriltag()
                setAngle(clamp(targetPosition,-servoRange/2,servoRange/2))
            }
            TurretState.ResetEncoder->
                setAngle(0.0)
            TurretState.Disabled -> setAngle(0.0)
        }
//        setPower(angleControl.calculate(targetPosition, getAngle())-follower.angularVelocity/loopTime*Kv)
    }

    override fun preInit() {
//        offset =  -transferMotor.value.currentPosition
    }
    override fun postUpdate() {
//        centerApriltag()
//        se(targetPosition)
        update()
        MyTelemetry.addData("Turret Servo Position", getPosition())
        MyTelemetry.addData("Turret Servo angle", getAngle())
        MyTelemetry.addData("Turret Servo realpos", getPositionServo())
        MyTelemetry.addData("Turret Servo angle2", ((getPosition()/8192)*360)/5)
        MyTelemetry.addData("Turret Servo vel", (getVel()))
//        MyTelemetry.addData("Turret Vol", turretEncoder.value.getVoltage())
        MyTelemetry.addData("Turret Target Position", targetPosition)
        MyTelemetry.addData("Turret state", state)
//        MyTelemetry.addData("Turret vol", getEncoderPosition())
        MyTelemetry.addData("Turret Global Heading", getGlobalHeading())
    }
}
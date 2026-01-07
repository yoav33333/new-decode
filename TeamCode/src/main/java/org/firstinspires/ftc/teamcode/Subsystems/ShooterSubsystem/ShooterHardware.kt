package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import dev.nextftc.core.components.Component
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.deltaThreshold
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.f
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodTarget
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.shootPowLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.targetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.veloControl
import org.firstinspires.ftc.teamcode.Util.VelocityPid
import kotlin.math.abs

object ShooterHardware: Component {
    val shooterMotor1 = lazy{MotorEx("High shooter")}
    val shooterMotor2 = lazy{MotorEx("Low shooter").reversed()}
    val hoodServo1 = lazy{ ServoEx("Hood servo")}
//    val controller = VelocityPid(
//        ShooterVars.p,
//        ShooterVars.i,
//        ShooterVars.d
//    )
    fun setHoodPosition(position: Double) {
        hoodServo1.value.position = position
    }
    fun getHoodPosition(): Double {
        return hoodServo1.value.position
    }
    fun setPower(power: Double) {
        shooterMotor1.value.power = power
        shooterMotor2.value.power = power
    }
    fun getVelocity(): Double {
        return -shooterMotor1.value.velocity
    }
    fun getPosition(): Double {
        return shooterMotor1.value.currentPosition.toDouble()
    }

    fun setVelocity(velocity: Double) {
        targetVelocity = velocity
//        veloControl.targetVelocity = velocity
    }
    fun atTargetVelocity(): Boolean {
        return deltaThreshold > abs(getVelocity()-targetVelocity)
    }
    fun shoot(distance: Double){
//        setVelocity(shootPowLUT.get(distance))
//        hoodTarget = hoodLUT.get(distance)
        setVelocity(1200.0)
    }
    fun stopShooting(){
        setVelocity(0.0)
    }

//    override fun preInit() {
//        controller.reset()
//
//    }
    fun update(){
        setHoodPosition(hoodTarget)
//        controller.setTolerance(deltaThreshold)
    if (targetVelocity.toInt() ==0) {
        setPower(0.0)
        return
    }
        setPower(veloControl.calculate(targetVelocity, getVelocity())+f)
//        controller.targetVelocity = targetVelocity
//        controller.setPID(ShooterVars.p, ShooterVars.i, ShooterVars.d)
//        setPower(controller.calculate(getVelocity()))
    }

    override fun preInit() {
        setPower(0.0)
    }
    override fun postUpdate() {
        update()

        MyTelemetry.addData("Shooter position", getPosition())
        MyTelemetry.addData("Shooter velocity", getVelocity())
        MyTelemetry.addData("Shooter target", targetVelocity)
        MyTelemetry.addData("Hood position", getHoodPosition())
        MyTelemetry.addData("at target vel", atTargetVelocity())
    }

}
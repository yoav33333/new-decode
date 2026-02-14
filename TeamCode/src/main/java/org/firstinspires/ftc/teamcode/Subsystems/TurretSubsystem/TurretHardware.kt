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
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.result
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware.transferMotor
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.Kv
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.angleControl
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.distP
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.encoderMul
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.offset
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.p
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.servoOffset
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
        setPosition((angle+servoOffset+servoRange/2)/servoRange)
    }
    fun centerApriltag(){
        val result = result
        if (result== null || !result.isValid)return
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
        targetPosition = clamp(position, -(servoRange/2), servoRange/2)
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
    fun getTargetAngle():Double{
        return ((getPositionServo())*360)
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
        // Compute the turret angle (relative to the robot) that corresponds to the desired global heading.
        // follower.heading.deg.value is the robot's current heading in degrees, so subtracting it from the
        // desired global heading gives the turret angle relative to the robot.
        val baseHeadingDeg = follower.heading.deg.value
        val turretDegrees = wrap360(globalDegrees - baseHeadingDeg)
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

    fun update(offset: Double) {
        // 1. Calculate the vector from the Robot to the Goal
        // Formula: Target - Current
        val robotPose = getPoseEstimate()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)
        val deltaVec = RobotVars.goalPos.minus(robotPose.asVector
            .minus(rotatedOffset).plus(Vector(follower.velocity.magnitude*loopTime/1000, follower.velocity.theta*loopTime/1000))
            )

        // 2. Get the Global Heading required to face that goal
        // .theta returns the angle of the vector in radians
        val globalTargetAngleRad = deltaVec.theta-offset

        // 3. Get the robot's current heading (and account for latency with angular velocity)
        // Ensure both are in Radians
        val robotHeadingRad = follower.heading.rad.value + (follower.angularVelocity * loopTime/1000)

        // 4. Calculate the relative (local) angle
        var localAngleRad = globalTargetAngleRad - robotHeadingRad

        // 5. Normalize the relative angle to [-PI, PI]
        // This ensures the turret takes the shortest path
        while (localAngleRad > Math.PI) localAngleRad -= 2 * Math.PI
        while (localAngleRad < -Math.PI) localAngleRad += 2 * Math.PI

        // 6. Convert to Degrees for the servo logic
        val localAngleDeg = Math.toDegrees(localAngleRad)

        // 7. Clamp to physical servo limits and set position
        // Assuming servoRange is the total travel (e.g., 300 degrees)
        val finalAngle = clamp(localAngleDeg, -servoRange / 2, servoRange / 2)

        setAngle(finalAngle)

        // Telemetry for debugging
        MyTelemetry.addData("Target Global Heading (Deg)", Math.toDegrees(globalTargetAngleRad))
        MyTelemetry.addData("Robot Current Heading (Deg)", Math.toDegrees(robotHeadingRad))
        MyTelemetry.addData("Turret Local Target (Deg)", localAngleDeg)
    }
    fun setTargetFromGlobal(globalDegrees: Double) {
        // Normalize the input to 0-360 for consistency
        targetPosition = wrap360(globalDegrees)
    }

    override fun preInit() {
        state = TurretState.TrackingAprilTags
        // Setting a default global target of 180 degrees.
        // The `vectorFromTarget` will override this in the update loop if it's being updated.
        setTargetFromGlobal(180.0)
//        offset =  -transferMotor.value.currentPosition
    }
    override fun postUpdate() {
//        centerApriltag()
//        se(targetPosition)
//        update()
        MyTelemetry.addData("Turret Servo Position", getPosition())
        MyTelemetry.addData("Turret Servo angle", getAngle())
        MyTelemetry.addData("Turret Servo realpos", getPositionServo())
//        MyTelemetry.addData("Turret Servo angle2", ((getPosition()/8192)*360)/5)
//        MyTelemetry.addData("Turret Servo vel", (getVel()))
//        MyTelemetry.addData("Turret Vol", turretEncoder.value.getVoltage())
        MyTelemetry.addData("Turret Target Position", clamp(wrap360(targetPosition+wrap360(follower.heading.deg.value)),-servoRange/2,servoRange/2))
//        MyTelemetry.addData("Turret Target Position", targetPosition)
        MyTelemetry.addData("Turret state", state)
//        MyTelemetry.addData("Turret vol", getEncoderPosition())
        MyTelemetry.addData("Turret Global Heading", getGlobalHeading())
    }
}

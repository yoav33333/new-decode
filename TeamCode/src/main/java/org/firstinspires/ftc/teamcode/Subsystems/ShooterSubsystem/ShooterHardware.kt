package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import androidx.core.math.MathUtils.clamp
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot.normalizePower
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.controlledSpeed
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.deltaThreshold
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.dy
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.entryAng
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.kv
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodCorrectionMul
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodTarget
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.hoodTargetAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.ks
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxPos
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minPos
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.mul
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.runShooter
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.shootPowLUT
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.targetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.veloControl
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import java.lang.Math.toDegrees
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

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
        hoodServo1.value.position = clamp(position,0.0,0.52)
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
    }

    fun atTargetVelocity(): Boolean {
        return deltaThreshold > abs(getVelocity()-targetVelocity)
    }
    fun shoot(vel: Double, angle: Double){
        setVelocity(controlledSpeed)
//        if (ShooterVars.disableAutoShooter) return
//        val distance = clamp(abs(distance), 61.4, 162.0)
//        setVelocity(shootPowLUT.get(distance))
//        hoodTarget = hoodLUT.get(distance)-abs(targetVelocity-getVelocity())*hoodCorrectionMul
        setVelocity(vel)
        hoodTargetAngle = angle
    }
    fun stopShooting(){
        runShooter = false
    }
    fun setAngle(angle:Double){
        hoodTarget = ((minPos-maxPos)/(minAngle-maxAngle))*(angle-minAngle)+minPos
    }
    fun velToTics(vel:Double): Double{
        return 5.9922*vel - 261.77
    }
    fun update(){
//    if (runShooter){
        val robotPose = getPoseEstimate()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)
        val deltaVec = RobotVars.goalPos.minus(robotPose.asVector
            .minus(rotatedOffset)
        )

        var res = calculateLowImpactParameters(
            deltaVec.magnitude,
            dy,
            follower.velocity.magnitude,
            follower.velocity.theta,
            deltaVec.theta
        )
        shoot(velToTics(res.launchVelocityInchesPerSec), toDegrees(90-res.launchAngleRad))
        MyTelemetry.addData("odo dist", deltaVec.magnitude)
        TurretHardware.update(res.turretOffsetRad)
//    }
//    else{
//        targetVelocity = 0.0
//    }
    setAngle(hoodTargetAngle)
        MyTelemetry.addData("hoodTarget", hoodTarget)
        MyTelemetry.addData("hoodTargetAngle", hoodTargetAngle)
    setHoodPosition(hoodTarget)
    if (targetVelocity.toInt() ==0) {
        setPower(0.0)
        return
    }
        setPower(
            normalizePower(
                veloControl.calculate(
                    targetVelocity, getVelocity()
                ) +kv*targetVelocity+ks
            )
        )
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


    /**
     * Data class to hold the calculated shooter parameters.
     */
    data class ShooterParameters(
        val launchAngleRad: Double,
        val launchVelocityInchesPerSec: Double,
        val turretOffsetRad: Double
    )

    fun calculateLowImpactParameters(
        x: Double,
        y: Double,
        robotVelocityMag: Double,
        robotVelocityAngleRad: Double,
        angleToGoalRad: Double
    ): ShooterParameters {
        val g = 386.1
        var bestAlpha = Math.toRadians(minAngle)
        var minLandingSpeed = Double.MAX_VALUE
        var foundValidTrajectory = false

        // 1. Iterate through possible hood angles
        val step = 0.5 // Higher precision for landing speed optimization
        var currentAngleDeg = minAngle

        while (currentAngleDeg <= maxAngle) {
            val alphaRad = Math.toRadians(currentAngleDeg)
            val tanAlpha = tan(alphaRad)

            // Constraint 1: The target must be physically reachable (denominator > 0)
            val denominator = 2.0 * cos(alphaRad).pow(2) * (x * tanAlpha - y)

            if (denominator > 0) {
                val v0 = sqrt((g * x.pow(2)) / denominator)

                // Constraint 2: Ensure the ball is falling (Apex is reached before the goal)
                // Mathematically: x > (v0^2 * sin(alpha) * cos(alpha)) / g
                // Simplified: The slope at x must be negative
                val isFalling = x * tanAlpha > 2 * y

                if (isFalling) {
                    // Landing speed calculation via Energy: Vf^2 = V0^2 - 2gh
                    val vf = sqrt(v0.pow(2) - 2 * g * y)

                    if (vf < minLandingSpeed) {
                        minLandingSpeed = vf
                        bestAlpha = alphaRad
                        foundValidTrajectory = true
                    }
                }
            }
            currentAngleDeg += step
        }

        // Fallback: If no falling trajectory found, use the last known best or minimum velocity
        if (!foundValidTrajectory) {
            // Handle error or default to a safe shot
        }

        // --- STEP B: Velocity Compensation (Vector Addition) ---
        // Time of flight = horizontal distance / horizontal velocity
        val stationaryV0 = sqrt((g * x.pow(2)) / (2.0 * cos(bestAlpha).pow(2) * (x * tan(bestAlpha) - y)))
        val timeOfFlight = x / (stationaryV0 * cos(bestAlpha))

        // Robot movement relative to the goal
        val relativeTheta = robotVelocityAngleRad - angleToGoalRad
        val vRadial = -cos(relativeTheta) * robotVelocityMag   // Towards/Away
        val vTangential = sin(relativeTheta) * robotVelocityMag // Sideways

        // Adjust horizontal components for robot movement
        val vxStationary = x / timeOfFlight
        val vxCompensated = vxStationary + vRadial

        // The turret must counteract tangential movement to keep the ball on the goal line
        val turretOffset = atan2(vTangential, vxCompensated)

        // Resultant horizontal velocity in the robot's frame
        val vxNew = sqrt(vxCompensated.pow(2) + vTangential.pow(2))
        val vy = stationaryV0 * sin(bestAlpha)

        // Final launch parameters
        val alphaCompensated = atan2(vy, vxNew)
        val vLaunchFinal = sqrt(vxNew.pow(2) + vy.pow(2))

        return ShooterParameters(alphaCompensated, vLaunchFinal, turretOffset)
    }
}


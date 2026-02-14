package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem

import androidx.core.math.MathUtils.clamp
import com.pedropathing.math.Vector
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot.normalizePower
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.controlledSpeed
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.dy
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.kv
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.ks
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.maxPos
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minAngle
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.minPos
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.runShooter
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.targetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.veloControl
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.math.*

object ShooterHardware : Component {
    val shooterMotor1 = lazy { MotorEx("High shooter") }
    val shooterMotor2 = lazy { MotorEx("Low shooter").reversed() }
    val hoodServo1 = lazy { ServoEx("Hood servo") }

    private var hoodTargetAngle = 0.0
    private var hoodTarget = 0.0

    fun setHoodPosition(position: Double) {
        hoodServo1.value.position = clamp(position, 0.0, 0.52)
    }

    fun getHoodPosition(): Double = hoodServo1.value.position
    fun getVelocity(): Double = -shooterMotor1.value.velocity
    fun getPosition(): Double = shooterMotor1.value.currentPosition.toDouble()

    fun setPower(power: Double) {
        shooterMotor1.value.power = power
        shooterMotor2.value.power = power
    }

    fun setVelocity(velocity: Double) {
        targetVelocity = velocity
    }

    fun atTargetVelocity(): Boolean = willReachGoal()

    fun shoot(vel: Double, angle: Double) {
        setVelocity(vel)
        hoodTargetAngle = angle
    }

    fun stopShooting() {
        runShooter = false
        targetVelocity = 0.0
    }

    fun setAngle(angle: Double) {
        // Linear mapping from degrees to servo position
        hoodTarget = ((minPos - maxPos) / (minAngle - maxAngle)) * (angle - minAngle) + minPos
    }

    fun velToTics(vel: Double): Double = 5.9922 * vel - 261.77
    fun ticsToInchesPerSec(tics: Double): Double = (tics + 261.77) / 5.9922

    /**
     * Mathematically corrects the hood angle based on the motor's actual velocity vs target.
     * Uses partial derivatives of the trajectory equation: h = x*tan(th) - (g*x^2)/(2*v^2*cos^2(th))
     */
    private fun calculateAngleCorrection(
        targetVeloInches: Double,
        currentVeloInches: Double,
        idealAngleRad: Double,
        x: Double,
        y: Double
    ): Double {
        val deltaV = currentVeloInches - targetVeloInches
        if (abs(deltaV) < 0.5 || currentVeloInches < 1.0) return 0.0

        val g = 386.1
        val cosSq = cos(idealAngleRad).pow(2)
        val tanTh = tan(idealAngleRad)

        // dh/dv: How much height changes per unit of velocity
        val dh_dv = (g * x.pow(2)) / (targetVeloInches.pow(3) * cosSq)

        // dh/dtheta: How much height changes per unit of angle (radians)
        val dh_dtheta = x * (1 / cosSq) - (g * x.pow(2) / targetVeloInches.pow(2)) * (tanTh / cosSq)

        // To keep height constant (dh = 0): dTheta = -(dh/dv / dh/dtheta) * dV
        return -(dh_dv / dh_dtheta) * deltaV
    }

    fun update() {
        val robotPose = getPoseEstimate()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)

        // Calculate lead-time vector to target
        val deltaVec = RobotVars.goalPos.minus(
            robotPose.asVector
                .minus(rotatedOffset)
                .plus(Vector(follower.velocity.magnitude * loopTime / 1000, follower.velocity.theta * loopTime / 1000))
        )

        val xDist = deltaVec.magnitude
        val res = calculateLowImpactParameters(
            xDist,
            dy,
            follower.velocity.magnitude,
            follower.velocity.theta,
            deltaVec.theta
        )

        // Set target velocity from the physics solver
        val theoreticalVelInches = res.launchVelocityInchesPerSec
        setVelocity(velToTics(theoreticalVelInches))

        // Get actual current velocity in physical units
        val currentVelInches = ticsToInchesPerSec(getVelocity())

        // Calculate the mathematical correction for the angle
        val correctionRad = calculateAngleCorrection(
            theoreticalVelInches,
            currentVelInches,
            res.launchAngleRad,
            xDist,
            dy
        )

        // Apply correction and update hood
        val finalAngleDeg = toDegrees(res.launchAngleRad + correctionRad)
        hoodTargetAngle = clamp(finalAngleDeg, minAngle, maxAngle)

        setAngle(hoodTargetAngle)
        setHoodPosition(hoodTarget)
        TurretHardware.update(res.turretOffsetRad)

        // Motor Control
        if (targetVelocity.toInt() == 0) {
            setPower(0.0)
        } else {
            val feedForward = kv * targetVelocity + ks
            val feedback = veloControl.calculate(targetVelocity, getVelocity())
            setPower(normalizePower(feedback + feedForward))
        }
    }

    override fun preInit() {
        setPower(0.0)
    }

    override fun postUpdate() {
        update()
        MyTelemetry.addData("Shooter velocity", getVelocity())
        MyTelemetry.addData("Shooter target", targetVelocity)
        MyTelemetry.addData("Hood Angle Target", hoodTargetAngle)
        MyTelemetry.addData("Hood Servo Pos", getHoodPosition())
    }

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
        var bestAlpha = toRadians(minAngle)
        var minLandingSpeed = Double.MAX_VALUE
        var foundValidTrajectory = false

        val step = 0.5
        var currentAngleDeg = minAngle

        while (currentAngleDeg <= maxAngle) {
            val alphaRad = toRadians(currentAngleDeg)
            val cosAlpha = cos(alphaRad)
            val denominator = 2.0 * cosAlpha.pow(2) * (x * tan(alphaRad) - y)

            if (denominator > 0) {
                val v0 = sqrt((g * x.pow(2)) / denominator)
                val isFalling = x * tan(alphaRad) > 2 * y

                if (isFalling) {
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

        val stationaryV0 = sqrt((g * x.pow(2)) / (2.0 * cos(bestAlpha).pow(2) * (x * tan(bestAlpha) - y)))
        val timeOfFlight = x / (stationaryV0 * cos(bestAlpha))

        val relativeTheta = robotVelocityAngleRad - angleToGoalRad
        val vRadial = -cos(relativeTheta) * robotVelocityMag
        val vTangential = sin(relativeTheta) * robotVelocityMag

        val vxStationary = x / timeOfFlight
        val vxCompensated = vxStationary + vRadial
        val turretOffset = atan2(vTangential, vxCompensated)

        val vxNew = sqrt(vxCompensated.pow(2) + vTangential.pow(2))
        val vy = stationaryV0 * sin(bestAlpha)

        return ShooterParameters(atan2(vy, vxNew), sqrt(vxNew.pow(2) + vy.pow(2)), turretOffset)
    }
    /**
     * Predicts if the ball will reach the goal based on ACTUAL current hardware states.
     * @param x The horizontal distance to the goal (inches)
     * @param y The vertical height of the goal (inches)
     * @return Boolean - true if the trajectory clears the goal height at distance x
     */
    fun willReachGoal(): Boolean {
        val robotPose = getPoseEstimate()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)

        // Calculate lead-time vector to target
        val deltaVec = RobotVars.goalPos.minus(
            robotPose.asVector
                .minus(rotatedOffset)
                .plus(Vector(follower.velocity.magnitude * loopTime / 1000, follower.velocity.theta * loopTime / 1000))
        )
        val x = deltaVec.magnitude
        val y = dy
        val currentVeloInches = ticsToInchesPerSec(getVelocity())
        val currentAngleRad = toRadians(((minAngle - maxAngle) / (minPos - maxPos)) * (getHoodPosition() - minPos) + minAngle)

        val g = 386.1

        // 1. Check if the ball has enough horizontal velocity to even reach the x distance
        // (Prevents division by zero or imaginary numbers in flight time)
        val vx = currentVeloInches * cos(currentAngleRad)
        if (vx <= 0.1) return false

        // 2. Calculate time to reach distance x: t = x / (v * cos(theta))
        val timeToGoal = x / vx

        // 3. Calculate the height of the ball at that specific time:
        // h(t) = v * sin(theta) * t - 0.5 * g * t^2
        val predictedHeight = (currentVeloInches * sin(currentAngleRad) * timeToGoal) - (0.5 * g * timeToGoal.pow(2))

        // 4. Determine if it's "falling" into the goal (optional but recommended)
        // A shot is usually only "good" if it hits the goal on the downward arc.
        val timeToPeak = (currentVeloInches * sin(currentAngleRad)) / g
        val isFalling = timeToGoal > timeToPeak

        // The ball reaches if the predicted height is within a tolerance of the goal height
        // and it hasn't hit the floor first (predictedHeight > 0)
        val heightTolerance = 2.0 // +/- 2 inches
        return predictedHeight >= (y - heightTolerance) && isFalling
    }
}
package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.pedropathing.math.Vector
import com.qualcomm.hardware.lynx.LynxModule
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware.getPoseEstimate
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.centerOfRotationOffset
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.allianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.deltaVec
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosBlue
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosRed
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.nominalVoltage
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.moveMul
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import kotlin.collections.forEach
import kotlin.math.pow

object Robot: Component {
    val controlHub = lazy{ ActiveOpMode.hardwareMap.getAll(LynxModule::class.java)[0] }
    private var cachedVoltage = 12.0
    private var lastVoltageUpdateTime = 0L

    fun getVoltage(): Double {
        val currentTime = System.currentTimeMillis()
        if (currentTime - lastVoltageUpdateTime >= 1000) {
            cachedVoltage = controlHub.value.getInputVoltage(VoltageUnit.VOLTS)
            lastVoltageUpdateTime = currentTime
        }
        return cachedVoltage
    }
    fun normalizePower(power: Double): Double{
        return power*nominalVoltage/getVoltage()
    }
    override fun preInit() {
        RobotVars.goalPos =( if (allianceColor == AllianceColor.RED) goalPosRed else goalPosBlue)
//        RobotVars.goalPos =( goalPosBlue)
    }

    override fun preUpdate() {
        MyTelemetry.addData("Goal Pos", RobotVars.goalPos)
        MyTelemetry.addData("Alliance Color", allianceColor)
        deltaVec = getDeltaVec()
        MyTelemetry.addData("distFromGoal", deltaVec.magnitude)
//        MyTelemetry.addData("distance from goal", RobotVars.vectorFromTarget.magnitude)
    }

    fun getDeltaVec(): Vector{
        val robotPose = getPoseEstimate()
//        val rotatedOffset = Vector().copy()
        val rotatedOffset = centerOfRotationOffset.copy()
        rotatedOffset.rotateVector(robotPose.heading.rad.value)
        val velPose = Vector()
        velPose.setOrthogonalComponents(follower.velocity.xComponent * loopTime / 1000, follower.velocity.yComponent * loopTime / 1000)
        // Calculate lead-time vector to target
        val deltaVec = RobotVars.goalPos.minus(
            robotPose.asVector
                .minus(rotatedOffset)
                .plus(Vector(follower.velocity.magnitude * loopTime / 1000+follower.velocity.magnitude*moveMul*smartDist, follower.velocity.theta))
                .plus(Vector(follower.acceleration.magnitude * (loopTime / 1000).pow(2), follower.acceleration.theta))
        )
        return deltaVec
    }
}
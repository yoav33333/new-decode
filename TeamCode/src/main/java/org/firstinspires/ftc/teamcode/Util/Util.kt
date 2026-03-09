package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommandAuto
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
import kotlin.time.Duration.Companion.seconds

object Util {

    fun wrap360(angle: Double): Double {
        return ((angle % 360) + 360) % 360
    }
    fun wrap360(angle: Int): Int {
        return ((angle % 360) + 360) % 360
    }
    fun pose3dToPose(pose3d: Pose): Pose {
        return Pose(pose3d.y+72,144-(pose3d.x+72),pose3d.heading)
    }
    fun createPath(pos1 :Pose, pos2:Pose, heading1: Double, heading2: Double): PathChain{
        return follower.pathBuilder()
            .addPath(BezierLine(pos1, pos2))
            .setLinearHeadingInterpolation(heading1, heading2)
            .build()
    }
    fun pose3DMetersToInches(pose3d: Pose3D): Pose {
        return Pose(pose3d.position.x * 39.3701,
            pose3d.position.y * 39.3701,
            pose3d.orientation.yaw,
            FTCCoordinates.INSTANCE)
    }

    @JvmStatic fun mmToInches(mm: Double): Double {
        return mm / 25.4
    }
    fun Cycle(intakePath: PathChain, shootingPath: PathChain, delay: Double = 0.0): SequentialGroup = SequentialGroup(
        UninteraptingCommand(intakeCommandAuto),
        FollowPath(intakePath).raceWith(WaitUntil{ isFull()}),
        Delay(delay.seconds).raceWith(WaitUntil{ isFull()}),
        FollowPath(shootingPath),
        InstantCommand{intakeCommandAuto.cancel()},
        outtake,
        shootingCommand,
    )
    fun createPath(pos1: Pose, pos2: Pose, tangent: Boolean = false, reversed:Boolean = false, vararg controlPoints: Pose): PathChain {
        val pathBuilder = follower.pathBuilder()
        if (controlPoints.isNotEmpty()) {
            pathBuilder.addPath(BezierCurve(pos1, *controlPoints, pos2))
        } else {
            pathBuilder.addPath(BezierLine(pos1, pos2))
        }
        if (tangent){
            pathBuilder.setTangentHeadingInterpolation()
            if (reversed){
                pathBuilder.setReversed()
            }
        }
        else{
            pathBuilder.setLinearHeadingInterpolation(pos1.heading, pos2.heading)
        }
        return pathBuilder.build()
    }
    fun angleWrap(radians: Double): Double {
        var radians = radians
        while (radians > Math.PI) {
            radians -= 2 * Math.PI
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI
        }

        // keep in mind that the result is in radians
        return radians
    }
}
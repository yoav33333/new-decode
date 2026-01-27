package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.calcGlobalHeadingToTarget
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.centerApriltag
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.zeroEnc
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.state
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object TurretCommands {
    fun moveToAngle(angle: Double) = InstantCommand{setAngle(angle)}
        .setRequirements(TurretHardware)
    fun moveToAngler(angle: Double) = InstantCommand{setAngle(angle)}
//        .setRequirements(TurretHardware)
    fun centerAprilTags() = InstantCommand{state = TurretState.TrackingAprilTags}
    fun turretSeq() = SequentialGroup(
    InstantCommand{setTargetPosition(0.0)
        state = TurretState.ResetEncoder},
    Delay(0.35.seconds),
    WaitUntil{abs(getVel())<4},
        InstantCommand{zeroEnc()},
//        Delay(0.3.seconds),
        InstantCommand{ centerAprilTags().schedule() }
    )
//    val resetSeq = LambdaCommand()
//        .setStart { moveToAngle(0.0) }
//    .setUpdate { MyTelemetry.addData("running","")
////        MyTelemetry.
//    }
//        .setIsDone { abs(getVel())<5 }
//        .setStop { zeroEnc()
//            centerAprilTags.schedule()}
//    .setInterruptible(false)
//    .setRequirements(TurretHardware)

    fun moveToGlobalAngle(angle: Double) = InstantCommand{TurretHardware.setTargetPositionFromGlobalDegrees(angle)}
        .setRequirements(TurretHardware)
    fun scan(stop: () -> Boolean, step: Double) =
        LambdaCommand()
            .setUpdate {
                var currentAngle = TurretHardware.getPosition()
                currentAngle += step
                setTargetPosition(currentAngle)
            }
            .setIsDone { stop() }
            .setRequirements(TurretHardware)
    val lockOnGoal = LambdaCommand()
        .setUpdate { calcGlobalHeadingToTarget(vectorFromTarget) }
        .setIsDone { false }
        .setRequirements(TurretHardware)

}
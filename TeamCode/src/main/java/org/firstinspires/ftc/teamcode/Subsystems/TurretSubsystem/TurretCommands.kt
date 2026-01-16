package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.calcGlobalHeadingToTarget
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.setTargetPositionFromDegrees

object TurretCommands {
    fun moveToAngle(angle: Double) = InstantCommand{setTargetPositionFromDegrees(angle)}
        .setRequirements(TurretHardware)
    fun moveToGlobalAngle(angle: Double) = InstantCommand{TurretHardware.setTargetPositionFromGlobalDegrees(angle)}
        .setRequirements(TurretHardware)
    fun scan(stop: () -> Boolean, step: Double) =
        LambdaCommand()
            .setUpdate {
                var currentAngle = TurretHardware.getPosition()
                currentAngle += step
                setTargetPositionFromDegrees(currentAngle)
            }
            .setIsDone { stop() }
            .setRequirements(TurretHardware)
    val lockOnGoal = LambdaCommand()
        .setUpdate { calcGlobalHeadingToTarget(vectorFromTarget) }
        .setIsDone { false }
        .setRequirements(TurretHardware)

}
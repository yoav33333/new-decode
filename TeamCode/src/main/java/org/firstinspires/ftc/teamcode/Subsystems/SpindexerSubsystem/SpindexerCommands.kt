package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import dev.nextftc.core.commands.Command
//import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.NullCommand
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.randomizationOffset
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.currentVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.deltaThreshold
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterVars.targetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.angleToServoPos
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.currentSteps
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getSpindexerVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.hasBallInTransfer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isAtTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isStuck
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.preShoot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexerEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.setPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.shoot
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.timeStuck
//import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.timeStuck
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.state
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
import org.firstinspires.ftc.teamcode.Util.ActiveDelay
import java.lang.Math.floorMod
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object SpindexerCommands {
    fun rotate(steps: Int) =
        InstantCommand{ SpindexerHardware.rotate(steps) }

    val checkColorAndUpdate = LambdaCommand()
        .setIsDone { checkIntakeColorAndUpdate() }

    val runIntakeSeq=
        SequentialGroup(
            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdate,
        )

    val fixSpindexSeq =
        SequentialGroup(
            SequentialGroup(
                    InstantCommand {
                        state = State.FIX
                        targetPosition = 2 * SpindexerVars.degreesPerSlot + SpindexerVars.offset
                        setPosition(angleToServoPos(targetPosition))
                    },
                    Delay(0.05.seconds),
                    InstantCommand {
                        targetPosition = 4 * SpindexerVars.degreesPerSlot + SpindexerVars.offset
                        setPosition(angleToServoPos(targetPosition))
                    },
                    Delay(0.1),
                    InstantCommand {
                        state = State.RUN
                    }),
            ).setRequirements(this)

    val resetingSeq = SequentialGroup(
        InstantCommand{
            resetSpindexer()
            state = State.RESET},
        Delay(4.5),
        WaitUntil{timeStuck()>1.2},
        InstantCommand{
            resetSpindexerEnc()
            state = State.RUN
        }
    )

    fun transferAll(startWhen: Command) =
        SequentialGroup(
            InstantCommand{preShoot()},
            WaitUntil { isAtTargetPosition() && (abs(currentVelocity - targetVelocity) < deltaThreshold) },
            startWhen,
            InstantCommand{shoot()},
            Delay(0.1.seconds),
            WaitUntil { isAtTargetPosition()},
            Delay(0.05.seconds),
        )
}
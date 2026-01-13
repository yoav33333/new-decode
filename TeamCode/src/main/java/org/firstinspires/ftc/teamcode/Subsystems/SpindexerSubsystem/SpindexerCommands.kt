package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import androidx.core.util.Supplier
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.intake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.smartIntake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.delayMul
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelDeadlineGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

object SpindexerCommands {
    fun rotate(steps: Int) =
        InstantCommand{ SpindexerHardware.rotate(steps) }
    val moveToIntakePosition =
        InstantCommand{ SpindexerHardware.moveEmptyToIntakePosition() }
    fun moveToTransferPosition(color: SpindexerSlotState) =
        InstantCommand{ SpindexerHardware.moveColorToTransferPosition(color) }


    fun moveToTransferPositionLocking(color: SpindexerSlotState) = LambdaCommand()
        .setStart{
            if(!SpindexerHardware.moveColorToTransferPosition(color)){
                when(color){
                SpindexerSlotState.GREEN-> SpindexerHardware.moveColorToTransferPosition(
                    SpindexerSlotState.PURPLE
                )
                SpindexerSlotState.PURPLE-> SpindexerHardware.moveColorToTransferPosition(
                    SpindexerSlotState.GREEN
                )
                else -> {}
                }
            }
        }
//        .setIsDone { SpindexerHardware.isAtTargetPosition()  }
        .setRequirements(SpindexerHardware)
    val checkColorAndUpdate = LambdaCommand()
        .setIsDone { checkIntakeColorAndUpdate() }


    val runIntakeCycle =
        SequentialGroup(
            checkColorAndUpdate,
            moveToIntakePosition,
//            IfElseCommand({ IntakeHardware.getVel()<400 }
//                ,outtake, intake),
//            smartIntake,
            Delay(0.5.seconds),
//            Delay(0.05.seconds)\\\\
        ).setRequirements(SpindexerHardware)
    val runIntakeSeq=
//        ParallelDeadlineGroupKill(
//            WaitUntil{isFull()},
            runIntakeCycle

    //        ).setRequirements(SpindexerHardware)
    fun transferAll(startWhen: Command) =
        SequentialGroup(

            moveToTransferPositionLocking(RobotVars.randomization.value[0]),
            startWhen,
//            Delay(SpindexerVars.spinDelay.seconds),
            WaitUntil{atTargetVelocity()},
            moveToTransferPositionLocking(RobotVars.randomization.value[1]),
            Delay(SpindexerVars.spinDelay.seconds*delayMul),
            WaitUntil{atTargetVelocity()},
            moveToTransferPositionLocking(RobotVars.randomization.value[2]),
            Delay(1.seconds),
//            Delay(SpindexerVars.spinDelay.seconds),
//            Delay(SpindexerVars.spinDelay.seconds),
//            Delay(SpindexerVars.spinDelay.seconds),
        )
}
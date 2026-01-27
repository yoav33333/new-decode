package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars.smartDist
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdateAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isAtTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.spinDelayIntake
import org.firstinspires.ftc.teamcode.Util.ActiveDelay
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
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
    val checkColorAndUpdateAuto = LambdaCommand()
        .setIsDone { checkIntakeColorAndUpdateAuto() }


    val runIntakeCycle =
        SequentialGroup(
//            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdate,
            ActiveDelay{spinDelayIntake.seconds},

            moveToIntakePosition,
//            IfElseCommand({ IntakeHardware.getVel()<400 }
//                ,outtake, intake),
//            smartIntake,
            Delay(0.30),

//            Delay(0.05.seconds)\\\\
        ).setRequirements(SpindexerHardware)
    val runIntakeCycleAuto =
        SequentialGroup(
//            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdateAuto,
            ActiveDelay{spinDelayIntake.seconds},
            moveToIntakePosition,
//            IfElseCommand({ IntakeHardware.getVel()<400 }
//                ,outtake, intake),
//            smartIntake,
            Delay(0.3),
            //            Delay(0.05.seconds)\\\\
        ).setRequirements(SpindexerHardware)
    val runIntakeSeq=
//        ParallelDeadlineGroupKill(
//            WaitUntil{isFull()},
            runIntakeCycle
    val runIntakeSeqAuto=
//        ParallelDeadlineGroupKill(
//            WaitUntil{isFull()},
        runIntakeCycleAuto

    //        ).setRequirements(SpindexerHardware)
    fun transferAll(startWhen: Command) =
        SequentialGroup(
            moveToTransferPositionLocking(RobotVars.randomization.value[0]),
            WaitUntil{isAtTargetPosition()},
            ActiveDelay { SpindexerVars.spinDelayShoot.seconds},
            startWhen,
//            Delay(SpindexerVars.spinDelayShoot.seconds),
            WaitUntil{atTargetVelocity()},
            moveToTransferPositionLocking(RobotVars.randomization.value[1]),
            ActiveDelay { (SpindexerVars.spinDelayShoot+smartDist*0.001).seconds} ,
            WaitUntil{isAtTargetPosition()},
            WaitUntil{atTargetVelocity()},
            moveToTransferPositionLocking(RobotVars.randomization.value[2]),
            ActiveDelay { (SpindexerVars.spinDelayShoot+smartDist*0.01).seconds},
            WaitUntil{isAtTargetPosition()},
            Delay(0.1.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
        )
}
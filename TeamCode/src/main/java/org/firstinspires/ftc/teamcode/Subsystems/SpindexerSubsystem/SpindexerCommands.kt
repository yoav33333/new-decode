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
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdateAuto
//import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdateAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.currentSteps
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getSpindexerVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.hasBallInTransfer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isAtTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isStuck
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexerEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.setPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.timeStuck
//import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.timeStuck
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.spinDelayIntake
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
    val moveToIntakePosition =
        InstantCommand{ SpindexerHardware.moveEmptyToIntakePosition() }
    fun moveToTransferPosition(color: SpindexerSlotState) =
        InstantCommand{ SpindexerHardware.moveColorToTransferPosition(color) }


    fun moveToTransferPositionLocking(color: ()->SpindexerSlotState) = LambdaCommand()
        .setStart{
            if(!SpindexerHardware.moveColorToTransferPosition(color())){
                when(color()){
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
//        .setRequirements(SpindexerHardware)
    val checkColorAndUpdate = LambdaCommand()
        .setIsDone { checkIntakeColorAndUpdate() }
    val checkColorAndUpdateAuto = LambdaCommand()
        .setIsDone { checkIntakeColorAndUpdateAuto() }


    val runIntakeCycle =
        SequentialGroup(
//            WaitUntil{isAtTargetPosition()},
            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdate,
//            ActiveDelay{spinDelayIntake.seconds},
            moveToIntakePosition,

//            Delay(0.05.seconds)\\\\
        )
    val runIntakeCycleAuto =
        SequentialGroup(

            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdateAuto,
//            ActiveDelay{spinDelayIntake.seconds},
            moveToIntakePosition,
//            IfElseCommand({ IntakeHardware.getVel()<400 }
//                ,outtake, intake),
//            smartIntake,
//            Delay(0.3),
            //            Delay(0.05.seconds)\\\\
        )
    val runIntakeSeq=
//        ParallelDeadlineGroupKill(
//            WaitUntil{isFull()},
            runIntakeCycle
    val runIntakeSeqAuto=
//        ParallelDeadlineGroupKill(
//            WaitUntil{isFull()},
        runIntakeCycle

//    //        ).setRequirements(SpindexerHardware)
//    val fixSpindex = lazy{
////        SequentialGroup(
////            Delay(0.1),
//            InstantCommand{
////                if (!isAtTargetPosition() && isStuck()){
//                    fixSpindexSeq.value.schedule()
////                }
//            }
////        )
//    }
    val fixSpindexSeq =
        SequentialGroup(
//            Delay(0.1),
//            IfElseCommand(
//                { !isAtTargetPosition() && abs(getVel()) < 5 },
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
//                NullCommand()
            ).setRequirements(this)
//        )
//    }
    val resetingSeq = SequentialGroup(
        InstantCommand{
            resetSpindexer()
            state = State.RESET},
//        WaitUntil{getSpindexerVel()>10},
        Delay(4.5),
//        WaitUntil{abs(getSpindexerVel())>100}.raceWith(Delay(0.5)),
        WaitUntil{timeStuck()>1.2},
        InstantCommand{
            resetSpindexerEnc()
            state = State.RUN
        }

    )
    fun transferAll(startWhen: Command) =
        SequentialGroup(
            moveToTransferPositionLocking({ RobotVars.randomization.value[floorMod(0 + randomizationOffset, 3)] }),
            WaitUntil { isAtTargetPosition() && (abs(currentVelocity - targetVelocity) < deltaThreshold) },
            startWhen,
            WaitUntil { atTargetVelocity()},
            Delay(0.055),

            moveToTransferPositionLocking({ RobotVars.randomization.value[floorMod(1 + randomizationOffset, 3)] }),
            WaitUntil { isAtTargetPosition() &&  atTargetVelocity() },
            Delay(0.055),

            moveToTransferPositionLocking {
                RobotVars.randomization.value[floorMod(
                    2 + randomizationOffset,
                    3
                )]
            },
            WaitUntil { isAtTargetPosition()},
            Delay(0.2.seconds),
        )
}
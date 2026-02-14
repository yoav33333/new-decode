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
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.angleToServoPos
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.checkIntakeColorAndUpdateAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.currentSteps
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getSpindexerVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.hasBallInTransfer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isAtTargetPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isStuck
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexerEnc
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.setPosition
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.spinDelayIntake
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.state
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.targetPosition
import org.firstinspires.ftc.teamcode.Util.ActiveDelay
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.UtilCommands.SequentialGroupFixed
import kotlin.math.abs
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
        SequentialGroupFixed(
//            WaitUntil{isAtTargetPosition()},
            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdate,
//            ActiveDelay{spinDelayIntake.seconds},
            moveToIntakePosition,

//            Delay(0.05.seconds)\\\\
        ).setRequirements(SpindexerHardware)
    val runIntakeCycleAuto =
        SequentialGroupFixed(

            WaitUntil{isAtTargetPosition()},
            checkColorAndUpdateAuto,
//            ActiveDelay{spinDelayIntake.seconds},
            moveToIntakePosition,
//            IfElseCommand({ IntakeHardware.getVel()<400 }
//                ,outtake, intake),
//            smartIntake,
//            Delay(0.3),
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
    val fixSpindex = lazy{
        SequentialGroupFixed(
            Delay(0.1),
            InstantCommand{
                if (!isAtTargetPosition() && isStuck()){
                    fixSpindexSeq.value.schedule()
                }
            }
        )
    }
    val fixSpindexSeq = lazy{
        SequentialGroupFixed(
//            Delay(0.1),
//            IfElseCommand(
//                { !isAtTargetPosition() && abs(getVel()) < 5 },
                SequentialGroupFixed(
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
            )
//        )
    }
    val resetingSeq = SequentialGroup(
        InstantCommand{
            resetSpindexer()
            state = State.RESET},
//        WaitUntil{getSpindexerVel()>10},
        Delay(0.2),
//        WaitUntil{abs(getSpindexerVel())>100}.raceWith(Delay(0.5)),
        WaitUntil{abs(getSpindexerVel())<1},
        InstantCommand{
            resetSpindexerEnc()
            state = State.RUN
        }

    )
    fun transferAll(startWhen: Command) =
        SequentialGroupFixed(
            moveToTransferPositionLocking(RobotVars.randomization.value[0]),
            WaitUntil{isAtTargetPosition()&&atTargetVelocity()},
//            WaitUntil { atTargetVelocity() },
//            ActiveDelay { (0.1).seconds},
            startWhen,
//            ActiveDelay { (SpindexerVars.spinDelayShoot+smartDist*0.0002).seconds},

//            Delay(SpindexerVars.spinDelayShoot.seconds),
            WaitUntil{atTargetVelocity()&&!hasBallInTransfer()},
            moveToTransferPositionLocking(RobotVars.randomization.value[1]),
//            ActiveDelay { (SpindexerVars.spinDelayShoot+smartDist*0.0002).seconds} ,
            WaitUntil{isAtTargetPosition()&&!hasBallInTransfer()&&atTargetVelocity()},
//            WaitUntil{atTargetVelocity()},
            moveToTransferPositionLocking(RobotVars.randomization.value[2]),
//            ActiveDelay { (SpindexerVars.spinDelayShoot+smartDist*0.0002).seconds},
            WaitUntil{isAtTargetPosition()&&!hasBallInTransfer()},
            Delay(0.4.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
//            Delay(SpindexerVars.spinDelayShoot.seconds),
        )
}
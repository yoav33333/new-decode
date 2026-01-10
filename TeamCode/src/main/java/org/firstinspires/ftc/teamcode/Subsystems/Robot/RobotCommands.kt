package org.firstinspires.ftc.teamcode.Subsystems.Robot

import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.NullCommand
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.smartIntake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.stopIntake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.outtakeThreshold
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLightVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.vectorFromTarget
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands.shoot
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.stopShooting
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getColorInIntake
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isEmpty
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelDeadlineGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelRaceGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object RobotCommands {
    val intakeCommand =
        SequentialGroup(
            IntakeCommands.stopIntake,
            ParallelDeadlineGroupKill(
                WaitUntil { isFull() },
//                smartIntake,
                InstantCommand { resetSpindexer() },
                RepeatCommand(runIntakeSeq),
                RepeatCommand(InstantCommand{
                    MyTelemetry.addData("running","")
                    if (abs(getVel()) <outtakeThreshold) setPower(-intakePower)
                else setPower(intakePower)})
            ),
            SequentialGroup(
                InstantCommand{smartIntake.cancel()},
                outtake,
                Delay(.5.seconds),
                InstantCommand { resetSpindexer() },
                stopIntake
            ),
        )

    val shootingCommand =
        ParallelDeadlineGroupKill(
            SequentialGroup(
                InstantCommand{intakeCommand.cancel()},
                stopIntake,
                WaitUntil { isEmpty() },
                Delay(0.5),
                InstantCommand { stopShooting() }
            ),
            SequentialGroup(
                stopTransfer,
                shoot,
                transferAll(
                    SequentialGroup(
                WaitUntil { atTargetVelocity() },
                        runTransfer
                    )
                ),
                InstantCommand { stopShooting() },
                reverseTransfer,
                Delay(0.2),
                stopTransfer,
            )
        ).then(InstantCommand { stopShooting() })

}
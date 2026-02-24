package org.firstinspires.ftc.teamcode.Subsystems.Robot

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.outtake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.smartIntake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands.stopIntake
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.getVel
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.intakePower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeVars.outtakeThreshold
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterCommands.shoot
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.stopShooting
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeqAuto
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isEmpty
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.resetSpindexer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.reverseTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Util.UtilCommands.ParallelDeadlineGroupKill
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import org.firstinspires.ftc.teamcode.Util.UtilCommands.UninteraptingCommand
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object RobotCommands {
    val intakeCommand =
        SequentialGroup(
            IntakeCommands.stopIntake,
            stopTransfer,
            ParallelDeadlineGroup(
                WaitUntil { isFull() },
                InstantCommand { resetSpindexer() },
                Delay(0.5),
                RepeatCommand(runIntakeSeq),
                RepeatCommand(InstantCommand{
                    MyTelemetry.addData("running","")
                    if (abs(getVel()) <outtakeThreshold|| Gamepads.gamepad1.b.get()) setPower(-intakePower)
                else setPower(intakePower)})
            ),
            InstantCommand{
                SequentialGroup(
                    InstantCommand { resetSpindexer() },
                    InstantCommand { smartIntake.cancel() },
                    outtake,
                    Delay(.5.seconds),
                    stopIntake
                ).schedule()
            },
        ).setRequirements(this)
    val intakeCommandAuto =
        SequentialGroup(
            IntakeCommands.stopIntake,
            stopTransfer,
            ParallelDeadlineGroup(
                WaitUntil { isFull() },
                InstantCommand { resetSpindexer() },
                Delay(0.5),
                RepeatCommand(runIntakeSeqAuto),
                RepeatCommand(InstantCommand{
                    MyTelemetry.addData("running","")
                    if (abs(getVel()) <outtakeThreshold|| Gamepads.gamepad1.b.get()) setPower(-intakePower)
                else setPower(intakePower)})
            ),
            InstantCommand{
                SequentialGroup(
                    InstantCommand { resetSpindexer() },
                    InstantCommand { smartIntake.cancel() },
                    outtake,
                    Delay(.5.seconds),
                    stopIntake
                ).schedule()
            },
        ).setRequirements(this)

    val scanCommand =
        SequentialGroup(
//                smartIntake,
            Delay(0.1),
            RepeatCommand(runIntakeSeqAuto) { isFull() },
        )
    val shootingCommand =
        ParallelGroup(
            SequentialGroup(
                InstantCommand { intakeCommand.cancel() },
                stopIntake,
//                WaitUntil { isEmpty() },
//                Delay(0.5),
//                InstantCommand { stopShooting() }
            ),
            SequentialGroup(
                stopTransfer,
//                shoot,
                transferAll(
                    SequentialGroup(
                        runTransfer
                    )
                ),
            UninteraptingCommand(

//                InstantCommand { stopShooting() },
                SequentialGroup (reverseTransfer,
                Delay(0.2),
                stopTransfer,)
            ))
        )
    val shootingCommandAuto =
        ParallelDeadlineGroup(
            SequentialGroup(
                InstantCommand { intakeCommand.cancel() },
                stopIntake,
                WaitUntil { isEmpty() },
                Delay(0.5),
//                InstantCommand { stopShooting() }
            ),
            SequentialGroup(
                stopTransfer,
//                shoot,
                transferAll(
                    SequentialGroup(
                        runTransfer
                    )
                ),
//                InstantCommand { stopShooting() },
                reverseTransfer,
                Delay(0.2),
                stopTransfer,
            )
        )
    val cancelShooting = SequentialGroup(
        InstantCommand{ shootingCommand.cancel() },
        reverseTransfer,
        Delay(0.2),
        stopTransfer,
    )
}
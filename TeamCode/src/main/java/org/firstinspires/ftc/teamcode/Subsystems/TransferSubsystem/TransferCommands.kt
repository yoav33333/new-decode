package org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware.atTargetVelocity
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferVars.transferPower

object TransferCommands {
    val runTransfer = InstantCommand { setPower(transferPower) }
    val runTransferSmart = LambdaCommand()
        .setUpdate {
            if (atTargetVelocity())
            setPower(transferPower)
            else
                setPower(0.0)
        }.setRequirements(
        TransferHardware)
    val reverseTransfer = InstantCommand { setPower(-transferPower) }.setRequirements(
        TransferHardware)
    val stopTransfer = InstantCommand { setPower(0.0) }.setRequirements(
        TransferHardware)
}
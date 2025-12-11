package org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem

import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware.setPower
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferVars.transferPower

object TransferCommands {
    val runTransfer = InstantCommand { setPower(transferPower) }.setRequirements(TransferHardware)
    val stopTransfer = InstantCommand { setPower(0.0) }.setRequirements(TransferHardware)

}
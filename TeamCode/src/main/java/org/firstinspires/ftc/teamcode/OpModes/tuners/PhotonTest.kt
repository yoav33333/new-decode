package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Pedro.Constants.createFollower
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Photon
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeCycle
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getColorInIntake
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.time.Duration.Companion.seconds

@TeleOp(group = "tuning")
class PhotonTest: TunerOpMode(SpindexerHardware, Photon, PedroComponent(Constants::createFollower)) {
    init {
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(rotate(1))
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(rotate(-1))
        Gamepads.gamepad2.a.whenBecomesTrue (intakeCommand )
        Gamepads.gamepad2.b.whenBecomesTrue (runIntakeCycle )
        Gamepads.gamepad2.leftStickButton.whenBecomesTrue{tracker = SpindexerTracker()}
        Gamepads.gamepad2.dpadDown.whenBecomesTrue(
            ParallelGroup(
                runTransfer,
                transferAll(Delay(0.5.seconds)),

                ).then(stopTransfer))
        RepeatCommand(InstantCommand { getColorInIntake() }).schedule()
        Gamepads . gamepad2 . dpadUp . whenBecomesTrue ((IntakeCommands.intake))
            .whenBecomesFalse(IntakeCommands.stopIntake)

    }

}
package org.firstinspires.ftc.teamcode.OpModes.tuners

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getColorInIntake
import org.firstinspires.ftc.teamcode.Util.LoopTimer
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand

open class TunerOpMode(vararg components: Component): NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            MyTelemetry,
            LoopTimer,
            *components
        )
    }
}
package org.firstinspires.ftc.teamcode.OpModes.tuners

import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Util.LoopTimer

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
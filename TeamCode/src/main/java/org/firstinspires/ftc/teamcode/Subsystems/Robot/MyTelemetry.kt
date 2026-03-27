package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Util.LoopTimer.loopTime
import java.util.Objects

object MyTelemetry : Component {
    val joined by lazy { JoinedTelemetry(PanelsTelemetry.ftcTelemetry, ActiveOpMode.telemetry) }
    override fun postUpdate() {
        joined.update()    }

    override fun postWaitForStart() {
        joined.update()
    }

    fun addData(label: String, data: Any) {
        joined.addData(label, data)
    }
}
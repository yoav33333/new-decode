package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode
import java.util.Objects

object MyTelemetry : Component{
    val joinedTelemetry = lazy{JoinedTelemetry(PanelsTelemetry.ftcTelemetry, ActiveOpMode.telemetry)}
    val data: MutableMap<String, Any> = mutableMapOf()
    public fun addData(key: String, value: Any) {
        data[key] = value
    }

    override fun preInit() {
        CommandManager.cancelAll()

    }
    override fun postWaitForStart() {
        update()
    }
    override fun postUpdate() {
        update()
    }
    private fun update() {
        data.forEach { (key, value) ->
            joinedTelemetry.value.addData(key, value)
        }
        joinedTelemetry.value.update()
        data.clear()
    }

}
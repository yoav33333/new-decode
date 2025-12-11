package org.firstinspires.ftc.teamcode.Util

import dev.nextftc.core.components.Component
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import kotlin.time.ComparableTimeMark
import kotlin.time.TimeSource.Monotonic.markNow

object LoopTimer: Component {
    private var lastTime: ComparableTimeMark? = null

    override fun preWaitForStart() = update()
    override fun preUpdate() = update()

    private fun update() {
        val currentTime = markNow()
        if (lastTime != null) {
            MyTelemetry.addData("Loop Time", currentTime - lastTime!!)
        }
        lastTime = currentTime
    }
}
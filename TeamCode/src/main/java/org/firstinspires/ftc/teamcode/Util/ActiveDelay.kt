package org.firstinspires.ftc.teamcode.Util

import androidx.core.util.Supplier
import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.parseDuration
import kotlin.time.ComparableTimeMark
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.TimeSource.Monotonic.markNow

class ActiveDelay(
    private val time: () -> Duration
) : Command() {
    init {
        named("Delay(${time().toDouble(DurationUnit.SECONDS)}s)")
    }

    /**
     * @param time the desired duration of this command, in seconds
     */
//    constructor(time: ()-> Double ) : this({ time().seconds })

    private lateinit var startTime: ComparableTimeMark

    override val isDone: Boolean
        get() = markNow() - startTime >= time()

    override fun start() {
        startTime = markNow()
    }
}
package org.firstinspires.ftc.teamcode.Subsystems

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker

object Antony : Component {
    // Lazy initialize hardware
    private val blinkin by lazy { hardwareMap.get(RevBlinkinLedDriver::class.java, "LED") }

    // Use a backing field to store the last known state
    private var lastAmount = -1

    // Constants for better readability and performance (avoiding repeated object lookups)
    private val PATTERNS = arrayOf(
        BlinkinPattern.RED,         // 0
        BlinkinPattern.RED_ORANGE,  // 1
        BlinkinPattern.BLUE_GREEN,  // 2
        BlinkinPattern.GREEN        // 3
    )

    override fun postUpdate() {
        val currentAmount = tracker.getAmount()

        // OPTIMIZATION: Only update hardware if the value has changed
        if (currentAmount != lastAmount) {
            val pattern = when (currentAmount) {
                in 0..3 -> PATTERNS[currentAmount]
                else -> BlinkinPattern.RED
            }

            blinkin.setPattern(pattern)
            lastAmount = currentAmount
        }
    }

    /**
     * Optional: Force a specific pattern (e.g., for signaling endgame)
     * This updates lastAmount so the loop doesn't immediately overwrite it.
     */
    fun forcePattern(pattern: BlinkinPattern) {
        blinkin.setPattern(pattern)
        lastAmount = -99 // Ensures the next postUpdate detects a change
    }
}
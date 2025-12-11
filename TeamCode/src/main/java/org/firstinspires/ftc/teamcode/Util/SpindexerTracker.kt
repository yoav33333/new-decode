package org.firstinspires.ftc.teamcode.Util

public enum class SpindexerSlotState {
    EMPTY,
    PURPLE,
    GREEN,
}
//0-2
class SpindexerTracker {
    private val data = Array<SpindexerSlotState>(3) { SpindexerSlotState.EMPTY }
    private var offset = 0  // which physical chamber is "logical slot 0"

    val size get() = data.size

    // Read logical slot
    operator fun get(i: Int): SpindexerSlotState =
        data[(i + offset).mod(size)]

    // Write logical slot
    operator fun set(i: Int, value: SpindexerSlotState) {
        data[(i + offset).mod(size)] = value
    }

    // Rotate +1 = CW, -1 = CCW
    fun rotate(steps: Int) {
        offset = (offset + steps).mod(size)
    }
    fun isFull(): Boolean {
        for (i in 0 until size) {
            if (this[i] == SpindexerSlotState.EMPTY) {
                return false
            }
        }
        return true
    }
    fun isEmpty(): Boolean {
        for (i in 0 until size) {
            if (this[i] != SpindexerSlotState.EMPTY) {
                return false
            }
        }
        return true
    }
    // How many steps to get to the nearest slot with the given state
    fun stepsToState(state: SpindexerSlotState, pos: Int): Int? {
        var bestSteps: Int? = null

        for (i in 0 until size) {
            if (this[i] == state) {
                // distance from pos to i in both directions
                val cw  = (i - pos).mod(size)        // clockwise steps
                val ccw = cw - size                  // counterclockwise steps (negative)

                // pick the shorter of the two
                val steps = if (kotlin.math.abs(ccw) < cw) ccw else cw

                // track the best solution
                if (bestSteps == null || kotlin.math.abs(steps) < kotlin.math.abs(bestSteps)) {
                    bestSteps = steps
                }
            }
        }

        return bestSteps
    }


    // Convenience helpers:
    fun logicalToPhysical(i: Int): Int =
        (i + offset).mod(size)

    fun physicalToLogical(p: Int): Int =
        (p - offset).mod(size)

    // Debug print
    override fun toString(): String {
        return (0 until size).joinToString(prefix = "[", postfix = "]") {
            when (this[it]) {
                SpindexerSlotState.EMPTY -> "Empty"
                SpindexerSlotState.PURPLE -> "Purple"
                SpindexerSlotState.GREEN -> "Green"
            }
        }
    }
}

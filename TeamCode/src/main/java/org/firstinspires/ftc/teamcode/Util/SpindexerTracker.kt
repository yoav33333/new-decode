package org.firstinspires.ftc.teamcode.Util

import kotlin.math.abs

public enum class SpindexerSlotState {
    EMPTY,
    PURPLE,
    GREEN,
}

class SpindexerTracker {
    // HARDWARE CONSTANTS
    private val TOTAL_SLOTS = 3
    // The servo can only move between these absolute steps relative to startup (0)
    private val MIN_LIMIT =2
    private val MAX_LIMIT = 4

    // Stores the state of the PHYSICAL slots (these don't move in the array, only the head moves)
    private val data = Array<SpindexerSlotState>(TOTAL_SLOTS) { SpindexerSlotState.EMPTY }

    // Tracks where the "intake head" is relative to the physical slots.
    // 0 = Center, positive = CW, negative = CCW
    private var currentHeadPos = 2

    val size get() = data.size

    // ACCESS LOGIC:
    // We map a "relative" index (like 0 for the one in front of us) to the actual physical slot
    private fun getPhysicalIndex(relativeIndex: Int): Int {
        // We use modulo to wrap around the array, regardless of absolute position
        return (currentHeadPos + relativeIndex).mod(size)
    }

    // Get state relative to the current head position
    operator fun get(relativeIndex: Int): SpindexerSlotState =
        data[getPhysicalIndex(relativeIndex)]

    // Set state relative to the current head position
    operator fun set(relativeIndex: Int, value: SpindexerSlotState) {
        data[getPhysicalIndex(relativeIndex)] = value
    }

    /**
     * Updates the tracker with a physical move.
     * @param steps Positive (CW) or Negative (CCW) movement.
     */
    fun move(steps: Int) {
        val newPos = currentHeadPos + steps
        // Safety clamp (though your hardware class should also prevent this)
        if (newPos in MIN_LIMIT..MAX_LIMIT) {
            currentHeadPos = newPos
        }
    }

    fun setPose(pose: Int){
        if (pose in MIN_LIMIT..MAX_LIMIT) {
            currentHeadPos = pose
        }
    }

    fun isFull(): Boolean = data.all { it != SpindexerSlotState.EMPTY }
    fun isEmpty(): Boolean = data.all { it == SpindexerSlotState.EMPTY }

    /**
     * Finds the best VALID move to get a specific state to the target position.
     * Unlike the old version, this respects the -2 to +2 limit.
     */
    fun stepsToState(state: SpindexerSlotState, targetRelativePos: Int): Int? {
        var bestMove: Int? = null

        // Iterate through all physical slots to find ones that match the requested state
        for (physicalIndex in 0 until size) {
            if (data[physicalIndex] == state) {

                // We found a slot with the right color. Now we need to see if we can reach it.
                // We need a 'move' such that:
                // (currentHeadPos + move + targetRelativePos) wraps to physicalIndex

                // We test every possible valid absolute position (-2 to 2)
                for (possibleAbsPos in MIN_LIMIT..MAX_LIMIT) {

                    // Does this absolute position align the head correctly?
                    if ((possibleAbsPos + targetRelativePos).mod(size) == physicalIndex) {

                        // It aligns! Now, what is the move required?
                        val requiredMove = possibleAbsPos - currentHeadPos

                        // Pick the shortest move if multiple valid options exist
                        if (bestMove == null || abs(requiredMove) < abs(bestMove)) {
                            bestMove = requiredMove
                        }
                    }
                }
            }
        }
        return bestMove
    }

    // Debug print
    override fun toString(): String {
        return "Head:$currentHeadPos " + (0 until size).joinToString(prefix = "[", postfix = "]") {
            // Display purely based on physical slots for debugging clarity
            val state = data[it]
            val marker = if (it == getPhysicalIndex(0)) "*" else "" // Mark current head
            "$marker${state.name.take(1)}"
        }
    }
}

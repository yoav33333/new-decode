package org.firstinspires.ftc.teamcode.Util

import androidx.core.math.MathUtils.clamp
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Randomization
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerSlotState
import java.lang.Math.floorMod

class NewSpindexerTracker {
    val spindexerSlots = 3
    val minPos = 0
    val maxPos = 5
    val arr = Array(spindexerSlots) { SpindexerSlotState.EMPTY }
    private var currentPos = 5
    //right - shoot - up
    //left - index/sort - down
    fun init(): Int{
        arr.fill(SpindexerSlotState.EMPTY)
        currentPos = maxPos
        return currentPos
    }
    inline fun <reified T> Array<T>.rotateRight(k: Int) {
        val n = this.size
        val kMod = ((k % n) + n) % n
        if (kMod == 0) return
        val temp = this.copyOf()
        for (i in 0 until n) {
            this[(i + kMod) % n] = temp[i]
        }
    }
    fun insert(color: SpindexerSlotState): Int{
        arr[0] = color
        arr.rotateRight(1)
        currentPos--
        currentPos = clamp(currentPos, maxPos-spindexerSlots+1, maxPos)
        return currentPos
    }
    fun getCurrentPos(): Int{
        return currentPos
    }
    fun getAmount(): Int{
        return arr.count { it != SpindexerSlotState.EMPTY }
    }
    fun isFull(): Boolean{
        return getAmount() == spindexerSlots
    }
    fun isEmpty(): Boolean{
        return getAmount() == 0
    }

    fun preShoot(pattern: Randomization): Int{
        var bestSteps = 0
        var bestScore = 0
        for (i in 0..<spindexerSlots){
            var score = 0
            for (j in 0..<pattern.value.size){
                score += if (arr[j] == pattern.value[floorMod(j+i+1, spindexerSlots)]) 1 else 0
            }
            if (score > bestScore){
                bestSteps = i
                bestScore = score
            }
        }
        arr.rotateRight(bestSteps)
        println("best steps: $bestSteps")
        currentPos -= bestSteps
        return currentPos
    }
    fun rotate(steps: Int): Int{
        arr.rotateRight(steps)
        currentPos -= steps
        currentPos = clamp(currentPos, minPos, maxPos)
        return currentPos
    }
    fun shoot(): Int{
        arr.fill(SpindexerSlotState.EMPTY)
        currentPos = maxPos
        return currentPos
    }

    override fun toString(): String {
        return ("""
        state: 
            ${arr[2].toString()} ${arr[1].toString()}
             ${arr[0].toString()}
        currentPos: $currentPos
        amount: ${getAmount()}
    """.trimIndent())
    }
}
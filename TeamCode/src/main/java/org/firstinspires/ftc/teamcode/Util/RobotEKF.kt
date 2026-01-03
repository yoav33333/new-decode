package org.firstinspires.ftc.teamcode.Util

import kotlin.math.*
import java.util.*
//fun main() {
//    val ekf = RobotEKF(dt = 0.05) // 20Hz loop
//    var time = System.currentTimeMillis()
//
//    println("Starting Robot Fusion Example...")
//
//    // 1. Simulate 10 steps of driving forward
//    for (i in 1..10) {
//        time += 50 // increment time by dt
//        ekf.predict(v = 1.0, w = 0.1, timestamp = time)
//
//        val pos = ekf.currentState.x
//        println("T=$time | Est: x=${"%.2f".format(pos[0])}, y=${"%.2f".format(pos[1])}, θ=${"%.2f".format(pos[2])}")
//    }
//
//    // 2. Simulate a Camera update arriving NOW, but it was recorded 150ms ago
//    val cameraLag = 150L
//    val recordedTime = time - cameraLag
//
//    println("\n--- CAMERA UPDATE ARRIVED (Lag: ${cameraLag}ms) ---")
//    // Camera says at T-150ms we were actually at x=0.2, y=0.01, theta=0.02
//    ekf.updateCamera(camX = 0.2, camY = 0.01, camTheta = 0.02, cameraTimestamp = recordedTime)
//
//    val finalPos = ekf.currentState.x
//    println("T=$time | Corrected: x=${"%.2f".format(finalPos[0])}, y=${"%.2f".format(finalPos[1])}, θ=${"%.2f".format(finalPos[2])}")
//}
//============================================================
//EKF TUNING CHEAT SHEET
//============================================================
//
//MATRIX Q (Process Noise): How much you trust your physics.
//- Increase Q if: The robot is agile, changes speed/direction
//instantly, or floors are slippery.
//- Decrease Q if: The robot's movement is very predictable
//and you want a smoother estimated path.
//
//MATRIX R (Sensor Noise): How much you trust your camera.
//- Increase R if: The camera is noisy, flickers, or gives
//jittery coordinates.
//- Decrease R if: The camera is high-precision and you want
//it to correct odometry drift instantly.
//
//------------------------------------------------------------
//SYMPTOM                        | ACTION
//------------------------------------------------------------
//Laggy / slow to react          | Decrease R or Increase Q
//Jittery / shaky estimate       | Increase R or Decrease Q
//Drifts too much between frames | Decrease Q
//Ignores camera markers         | Decrease R
//Teleports / Glitches on frames | Increase R
//------------------------------------------------------------
/**
 * Extended Kalman Filter for Robot Odometry and Camera Fusion.
 * Handles non-linear motion and camera latency via historical replay.
 */
class RobotEKF(val dt: Double, private val bufferSize: Int = 100) {

    // State: [x, y, theta, v, omega]
    data class State(val x: DoubleArray, val P: Array<DoubleArray>) {
        fun copy() = State(x.copyOf(), Array(P.size) { P[it].copyOf() })
    }

    data class HistoryEntry(val timestamp: Long, val state: State, val v: Double, val w: Double)

    var currentState = State(
        DoubleArray(5) { 0.0 }, // Initial position/velocity at 0
        Array(5) { i -> DoubleArray(5) { j -> if (i == j) 0.1 else 0.0 } }
    )

    private val history = LinkedList<HistoryEntry>()

    // Process Noise (How much we trust our kinematic model)
    @JvmField
    var Q = Array(5) { i -> DoubleArray(5) { j ->
        when(i) {
            0, 1 -> 0.05  // X and Y position uncertainty
            2    -> 0.02  // Heading (Theta) uncertainty
            3, 4 -> 0.1   // Velocity and Omega uncertainty
            else -> 0.0
        }
    }}
    // Find this inside the updateCamera function:
    @JvmField
    var R = arrayOf(
        doubleArrayOf(0.1, 0.0, 0.0), // Trust in X (meters)
        doubleArrayOf(0.0, 0.1, 0.0), // Trust in Y (meters)
        doubleArrayOf(0.0, 0.0, 0.05) // Trust in Theta (radians)
    )
    /**
     * Prediction Step: Call this at your high-frequency loop (e.g., 50Hz)
     */
    fun predict(v: Double, w: Double, timestamp: Long) {
        val theta = currentState.x[2]

        // 1. Non-linear state transition f(x)
        val nextX = DoubleArray(5)
        nextX[0] = currentState.x[0] + v * cos(theta) * dt
        nextX[1] = currentState.x[1] + v * sin(theta) * dt
        nextX[2] = normalizeAngle(currentState.x[2] + w * dt)
        nextX[3] = v
        nextX[4] = w

        // 2. Jacobian F = df/dx
        val F = Array(5) { DoubleArray(5) }.apply {
            this[0] = doubleArrayOf(1.0, 0.0, -v * sin(theta) * dt, cos(theta) * dt, 0.0)
            this[1] = doubleArrayOf(0.0, 1.0,  v * cos(theta) * dt, sin(theta) * dt, 0.0)
            this[2] = doubleArrayOf(0.0, 0.0, 1.0, 0.0, dt)
            this[3] = doubleArrayOf(0.0, 0.0, 0.0, 1.0, 0.0)
            this[4] = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 1.0)
        }

        // 3. P = F * P * F^T + Q
        val nextP = matrixAdd(matrixMultiply(matrixMultiply(F, currentState.P), transpose(F)), Q)

        currentState = State(nextX, nextP)

        // Save to buffer
        history.addLast(HistoryEntry(timestamp, currentState.copy(), v, w))
        if (history.size > bufferSize) history.removeFirst()
    }

    /**
     * Camera Update: Call this when your Vision system returns a result.
     * Uses historical replay to handle processing lag.
     */
    fun updateCamera(camX: Double, camY: Double, camTheta: Double, cameraTimestamp: Long) {
        // Find index in history closest to the camera's timestamp
        val historicalIdx = history.indices.minByOrNull { abs(history[it].timestamp - cameraTimestamp) } ?: return

        // 1. Roll back to that specific moment
        var rollingState = history[historicalIdx].state.copy()

        // 2. Apply Correction to that old state
        val H = Array(3) { DoubleArray(5) }.apply {
            this[0][0] = 1.0; this[1][1] = 1.0; this[2][2] = 1.0
        }
//        val R = arrayOf(
//            doubleArrayOf(0.1, 0.0, 0.0), // Trust in X (meters)
//            doubleArrayOf(0.0, 0.1, 0.0), // Trust in Y (meters)
//            doubleArrayOf(0.0, 0.0, 0.05) // Trust in Theta (radians)
//        )
        val z = doubleArrayOf(camX, camY, camTheta)
        val zPred = doubleArrayOf(rollingState.x[0], rollingState.x[1], rollingState.x[2])

        rollingState = applyUpdateMath(rollingState, z, zPred, H, R, true)

        // 3. Replay history forward to the present
        for (i in historicalIdx until history.size) {
            val h = history[i]
            // Re-predict step i+1 using corrected state i
            val theta = rollingState.x[2]
            val nextX = DoubleArray(5)
            nextX[0] = rollingState.x[0] + h.v * cos(theta) * dt
            nextX[1] = rollingState.x[1] + h.v * sin(theta) * dt
            nextX[2] = normalizeAngle(rollingState.x[2] + h.w * dt)
            nextX[3] = h.v
            nextX[4] = h.w

            // Simplify: Reuse same F calculation logic
            val F = Array(5) { DoubleArray(5) }.apply {
                this[0] = doubleArrayOf(1.0, 0.0, -h.v * sin(theta) * dt, cos(theta) * dt, 0.0)
                this[1] = doubleArrayOf(0.0, 1.0,  h.v * cos(theta) * dt, sin(theta) * dt, 0.0)
                this[2] = doubleArrayOf(0.0, 0.0, 1.0, 0.0, dt)
                this[3] = doubleArrayOf(0.0, 0.0, 0.0, 1.0, 0.0)
                this[4] = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 1.0)
            }
            val nextP = matrixAdd(matrixMultiply(matrixMultiply(F, rollingState.P), transpose(F)), Q)

            rollingState = State(nextX, nextP)
            // Update history entries with the new "better" truth
            history[i] = h.copy(state = rollingState.copy())
        }

        // 4. Update Current Live State
        currentState = rollingState
    }

    private fun applyUpdateMath(s: State, z: DoubleArray, zPred: DoubleArray, H: Array<DoubleArray>, R: Array<DoubleArray>, isAngular: Boolean): State {
        val y = DoubleArray(z.size) { i -> z[i] - zPred[i] }
        if (isAngular) y[2] = normalizeAngle(y[2])

        val HT = transpose(H)
        val S = matrixAdd(matrixMultiply(matrixMultiply(H, s.P), HT), R)
        val Si = invert3x3(S) ?: return s // Guard against singular matrix
        val K = matrixMultiply(matrixMultiply(s.P, HT), Si)

        val newX = s.x.copyOf()
        val Ky = matrixVectorMultiply(K, y)
        for (i in 0..4) newX[i] += Ky[i]

        val I = Array(5) { i -> DoubleArray(5) { j -> if (i == j) 1.0 else 0.0 } }
        val newP = matrixMultiply(matrixSubtract(I, matrixMultiply(K, H)), s.P)

        return State(newX, newP)
    }

    // --- MATH UTILITIES ---

    private fun normalizeAngle(a: Double) = atan2(sin(a), cos(a))

    private fun transpose(m: Array<DoubleArray>) = Array(m[0].size) { i -> DoubleArray(m.size) { j -> m[j][i] } }

    private fun matrixMultiply(A: Array<DoubleArray>, B: Array<DoubleArray>): Array<DoubleArray> {
        val out = Array(A.size) { DoubleArray(B[0].size) }
        for (i in A.indices) for (k in B.indices) for (j in B[0].indices) out[i][j] += A[i][k] * B[k][j]
        return out
    }

    private fun matrixVectorMultiply(A: Array<DoubleArray>, V: DoubleArray) = DoubleArray(A.size) { i ->
        var sum = 0.0; for (j in V.indices) sum += A[i][j] * V[j]; sum
    }

    private fun matrixAdd(A: Array<DoubleArray>, B: Array<DoubleArray>) = Array(A.size) { i -> DoubleArray(A[0].size) { j -> A[i][j] + B[i][j] } }

    private fun matrixSubtract(A: Array<DoubleArray>, B: Array<DoubleArray>) = Array(A.size) { i -> DoubleArray(A[0].size) { j -> A[i][j] - B[i][j] } }

    private fun invert3x3(m: Array<DoubleArray>): Array<DoubleArray>? {
        val det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1]) -
                m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
                m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0])
        if (abs(det) < 1e-10) return null
        val id = 1.0 / det
        return arrayOf(
            doubleArrayOf((m[1][1]*m[2][2]-m[1][2]*m[2][1])*id, (m[0][2]*m[2][1]-m[0][1]*m[2][2])*id, (m[0][1]*m[1][2]-m[0][2]*m[1][1])*id),
            doubleArrayOf((m[1][2]*m[2][0]-m[1][0]*m[2][2])*id, (m[0][0]*m[2][2]-m[0][2]*m[2][0])*id, (m[1][0]*m[0][2]-m[0][0]*m[1][2])*id),
            doubleArrayOf((m[1][0]*m[2][1]-m[1][1]*m[2][0])*id, (m[2][0]*m[0][1]-m[0][0]*m[2][1])*id, (m[0][0]*m[1][1]-m[1][0]*m[0][1])*id)
        )
    }
}


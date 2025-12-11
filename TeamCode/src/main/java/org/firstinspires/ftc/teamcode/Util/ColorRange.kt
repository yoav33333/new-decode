package org.firstinspires.ftc.teamcode.Util

class ColorRange(@JvmField var rMin: Double, @JvmField var rMax: Double,
                 @JvmField var gMin: Double, @JvmField var gMax: Double,
                 @JvmField var bMin: Double, @JvmField var bMax: Double) {
    fun inRange(r: Float, g: Float, b: Float): Boolean {
        return r in rMin..rMax && g in gMin..gMax && b in bMin..bMax
    }
}
package org.firstinspires.ftc.teamcode.Util

class HSVRange(@JvmField var hMin: Double, @JvmField var hMax: Double,
                 @JvmField var sMin: Double, @JvmField var sMax: Double,
                 @JvmField var vMin: Double, @JvmField var vMax: Double) {
    fun inRange(h: Double, s: Double, v: Double): Boolean {
        return h in hMin..hMax && s in sMin..sMax && v in vMin..vMax
    }
    fun inRange(hsv: HSV): Boolean{
        return inRange(hsv.h, hsv.s, hsv.v)
    }
}
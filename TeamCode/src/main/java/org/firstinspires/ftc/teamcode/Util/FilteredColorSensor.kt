package org.firstinspires.ftc.teamcode.Util

import android.graphics.Color
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
class HSV(var h: Double, var s: Double, var v: Double){
    override fun toString(): String {
        return "H: $h, S: $s, V: $v"
    }
}
class FilteredColorSensor(val colorSensor: ColorRangeSensor)  {
    var hGain = 0.5
    var sGain = 0.5
    var vGain = 0.5
    @JvmField public var filterH = MyLowPassFilter(hGain)
    @JvmField public var filterS = MyLowPassFilter(sGain)
    @JvmField public var filterV = MyLowPassFilter(vGain)
    var hsv = FloatArray(3)
    fun getHSV(): HSV {
//        colorSensor.
        Color.colorToHSV(colorSensor.normalizedColors.toColor(), hsv)
        return HSV(filterH.estimate(hsv[0].toDouble()),
            filterS.estimate(hsv[1].toDouble()),
            filterV.estimate(hsv[2].toDouble()))
    }

}
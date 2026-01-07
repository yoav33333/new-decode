package org.firstinspires.ftc.teamcode.Util

import android.graphics.Color
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
class HSV(var h: Double, var s: Double, var v: Double){
    override fun toString(): String {
        return "H: $h, S: $s, V: $v"
    }
}
class FilteredColorSensor(val colorSensor: RevColorSensorV3)  {
    var hGain = 0.99
    var sGain = 0.99
    var vGain = 0.99
    @JvmField public var filterH = MyLowPassFilter(hGain)
    @JvmField public var filterS = MyLowPassFilter(sGain)
    @JvmField public var filterV = MyLowPassFilter(vGain)
    var hsv = FloatArray(3)
    fun getHSV(): HSV {
//        colorSensor.
        Color.colorToHSV(colorSensor.normalizedColors.toColor(), hsv)
        return HSV(hsv[0].toDouble(),
            hsv[1].toDouble(),
            hsv[2].toDouble())
    }
    public fun resetFilter(){
        filterH = MyLowPassFilter(hGain)
        filterS = MyLowPassFilter(sGain)
        filterV = MyLowPassFilter(vGain)
    }

}
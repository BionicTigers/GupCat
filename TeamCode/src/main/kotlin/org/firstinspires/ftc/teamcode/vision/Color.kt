package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Scalar
import kotlin.math.pow
import kotlin.math.sqrt

data class Color(val r: Int, val g: Int, val b: Int) {
    constructor(hex: String) : this(
        Integer.valueOf(hex.substring(1, 3), 16),
        Integer.valueOf(hex.substring(3, 5), 16),
        Integer.valueOf(hex.substring(5, 7), 16)
    )

    fun distanceTo(other: Color): Double {
        return sqrt(
            (r - other.r).toDouble().pow(2.0)
                + (g - other.g).toDouble().pow(2.0)
                + (b - other.b).toDouble().pow(2.0)
        )
    }

    val hsv: FloatArray
        get() {
            val hsv = FloatArray(3)
            android.graphics.Color.RGBToHSV(r, g, b, hsv)
            return hsv
        }

    val hex: String
        get() = "#${r.toString(16)}${g.toString(16)}${b.toString(16)}"

    val scalar: Scalar
        get() = Scalar(r.toDouble(), g.toDouble(), b.toDouble())
}

data class ColorRange(val lower: Color, val upper: Color) {
    operator fun contains(color: Color): Boolean {
        return color.distanceTo(lower) <= color.distanceTo(upper)
    }
}

data class ColorRangeGroup(val list: List<ColorRange>) {
    constructor(vararg ranges: ColorRange) : this(ranges.toList())

    operator fun contains(color: Color): Boolean {
        return list.any { (a, b) -> color.distanceTo(a) <= color.distanceTo(b) }
    }

    operator fun iterator(): Iterator<ColorRange> {
        return list.iterator()
    }
}

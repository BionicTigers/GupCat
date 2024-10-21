package org.firstinspires.ftc.teamcode.utils

class InterpolatedMap : HashMap<Double, Double>() {
    override fun get(key: Double): Double {
        val keys = keys.toList()
        val values = values.toList()
        if (keys.isEmpty()) return 0.0
        if (keys.size == 1) return values[0]
        if (key <= keys[0]) return values[0]
        if (key >= keys[keys.size - 1]) return values[keys.size - 1]
        for (i in 0 until keys.size - 1) {
            if (key >= keys[i] && key <= keys[i + 1]) {
                val x1 = keys[i]
                val x2 = keys[i + 1]
                val y1 = values[i]
                val y2 = values[i + 1]
                return y1 + (key - x1) * (y2 - y1) / (x2 - x1)
            }
        }
        return 0.0
    }
}

fun interpolatedMapOf(vararg pairs: Pair<Double, Double>): InterpolatedMap {
    val map = InterpolatedMap()
    for (pair in pairs) {
        map[pair.first] = pair.second
    }
    return map
}
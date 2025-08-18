package org.firstinspires.ftc.teamcode.motion

import edu.wpi.first.math.DARE
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.Num
import org.jetbrains.kotlinx.multik.api.linalg.eig
import org.jetbrains.kotlinx.multik.api.linalg.inv
import org.jetbrains.kotlinx.multik.api.mk
import org.jetbrains.kotlinx.multik.api.ndarray
import org.jetbrains.kotlinx.multik.ndarray.data.D2Array
import org.jetbrains.kotlinx.multik.ndarray.data.get
import org.jetbrains.kotlinx.multik.ndarray.operations.*

/**
 * Discrete Linear Quadratic Regulator
 *
 * @param aJacobian Function that returns the A matrix
 * @param bJacobian Function that returns the B matrix
 * @param q State cost matrix
 * @param r Control cost matrix
 */
class LQR(
    val aJacobian: () -> D2Array<Double>,
    val bJacobian: () -> D2Array<Double>,
    val q: D2Array<Double>,
    val r: D2Array<Double>
) {
    var k: D2Array<Double>? = null
    init {
        require(q.shape[0] == q.shape[1]) { "Q must be a square matrix." }
        require(r.shape[0] == r.shape[1]) { "R must be a square matrix." }
        require(q.isPositiveSemiDefinite()) { "Q must be positive semi-definite." }
        require(r.isPositiveDefinite()) { "R must be positive definite." }
        linearize()
        k = computeK()
    }

    /**
     * Linearizes the system by returning the A and B Jacobians.
     *
     * @return Pair of A and B Jacobians as D2Arrays.
     */
    private fun linearize(): Pair<D2Array<Double>, D2Array<Double>> {
        val a = aJacobian()
        val b = bJacobian()

        // Ensure A and B are valid
        require(a.shape[0] == a.shape[1]) { "A Jacobian must be square." }
        require(b.shape[0] == a.shape[0]) { "B Jacobian must match the number of rows in A Jacobian." }

        return Pair(a, b)
    }

    fun computeK(): D2Array<Double> {
        val (a, b) = linearize()
        val p = DARE.dareNoPrecond(
            a.toWpiMatrix(),
            b.toWpiMatrix(),
            q.toWpiMatrix(),
            r.toWpiMatrix(),
        ).toMultiK()

        var newK = -mk.linalg.inv(r + b.transpose() * p * b) * (b.transpose() * p * a)

        if (mk.linalg.eig(a - b * newK).first.any { it.re >= 0 }) {
            // If the eigenvalues are not all negative, we need to recompute K
            // This is a fallback to ensure stability
            newK = k!!
        }

        return newK
    }
}

private fun D2Array<Double>.isPositiveDefinite(tol: Double = 1e-8): Boolean {
    if (!this.isSymmetric(tol)) return false
    val eig = mk.linalg.eig(this).first

    val realParts = DoubleArray(eig.size) { idx ->
        eig[idx].re
    }

    return realParts.all { it > tol }
}

private fun D2Array<Double>.isPositiveSemiDefinite(tol: Double = 1e-8): Boolean {
    if (!this.isSymmetric(tol)) return false
    val eig = mk.linalg.eig(this).first

    val realParts = DoubleArray(eig.size) { idx ->
        eig[idx].re
    }

    return realParts.all { it >= tol }
}

private fun D2Array<Double>.isSymmetric(tol: Double = 1e-8): Boolean {
    val rows = this.shape[0]
    val cols = this.shape[1]
    if (rows != cols) return false

    for (i in 0 until rows) {
        for (j in 0 until cols) {
            if (kotlin.math.abs(this[i, j] - this[j, i]) > tol) {
                return false
            }
        }
    }
    return true
}

private fun <R: Num, C: Num> D2Array<Double>.toWpiMatrix(): Matrix<R, C> {
    val (rows, cols) = this.shape
    val wpiMatrix = Matrix(rows.toNat(), cols.toNat())

    for (i in 0 until rows) {
        for (j in 0 until cols) {
            wpiMatrix.set(i, j, this[i, j])
        }
    }
    @Suppress("UNCHECKED_CAST")
    return wpiMatrix as Matrix<R, C>
}

private fun Int.toNat(): Nat<*> {
    return when (this) {
        0 -> Nat.N0()
        1 -> Nat.N1()
        2 -> Nat.N2()
        3 -> Nat.N3()
        4 -> Nat.N4()
        5 -> Nat.N5()
        6 -> Nat.N6()
        7 -> Nat.N7()
        8 -> Nat.N8()
        9 -> Nat.N9()
        10 -> Nat.N10()
        else -> throw IllegalArgumentException("Unsupported size: $this")
    }
}

private fun Matrix<*, *>.toMultiK(): D2Array<Double> {
    val rows = this.numRows
    val cols = this.numCols
    val data = Array(rows) { DoubleArray(cols) }

    for (i in 0 until rows) {
        for (j in 0 until cols) {
            data[i][j] = this.get(i, j)
        }
    }

    return mk.ndarray(data)
}

fun DoubleArray.toMK2D(): D2Array<Double> {
    return mk.ndarray(this).reshape(this.size, 1)
}

fun Array<DoubleArray>.toMK2D(): D2Array<Double> {
    return mk.ndarray(this)
}

fun createTuningMatrix(vararg stateCosts: Double) : D2Array<Double> =
    Array(stateCosts.size) { row ->
        DoubleArray(stateCosts.size) { col ->
            if (row == col) stateCosts[row] else 0.0
        }
    }.toMK2D()
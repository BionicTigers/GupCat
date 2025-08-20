package org.firstinspires.ftc.teamcode.motion

import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import org.jetbrains.kotlinx.multik.api.mk
import org.jetbrains.kotlinx.multik.ndarray.data.D2Array
import org.jetbrains.kotlinx.multik.api.ndarray
import org.jetbrains.kotlinx.multik.ndarray.data.get

/**
 * Solves the discrete LQR via the DARE using Kleinman iteration.
 * Assumes (A,B) stabilizable and (Q^(1/2), A) detectable.
 *
 * Returns K (feedback) and P (Riccati solution).
 */
data class LqrSolution(val K: SimpleMatrix, val P: SimpleMatrix)

fun dare(
    A: SimpleMatrix,
    B: SimpleMatrix,
    Q: SimpleMatrix,
    R: SimpleMatrix,
    maxIters: Int = 200,
    tol: Double = 1e-9
): LqrSolution {
    require(A.numRows() == A.numCols()) { "A must be square" }
    require(B.numRows() == A.numRows()) { "B rows must match A" }
    require(Q.numRows() == A.numRows() && Q.numCols() == A.numCols()) { "Q must be same size as A" }
    require(R.numRows() == R.numCols()) { "R must be square" }

    var P = Q.copy() // good starting guess for many problems
    var K = SimpleMatrix(B.numCols(), A.numRows()) // zeros

    var lastChange = Double.POSITIVE_INFINITY
    var it = 0
    while (it++ < maxIters && lastChange > tol) {
        // K = (R + Bᵀ P B)⁻¹ (Bᵀ P A)
        val BtP = B.transpose().mult(P)
        val S = R.plus(BtP.mult(B))               // R + Bᵀ P B
        val F = BtP.mult(A)                       // Bᵀ P A
        val Knew = S.solve(F)                     // S⁻¹ F

        // P_new = Q + (A - B K)ᵀ P (A - B K) + Kᵀ R K - Kᵀ Bᵀ P B K  (expanded form below simplified)
        val Acl = A.minus(B.mult(Knew))
        val Pnew = Q.plus(Acl.transpose().mult(P).mult(Acl)).plus(
            Knew.transpose().mult(R).mult(Knew)
        )

        lastChange = Pnew.minus(P).elementMaxAbs()
        P = Pnew
        K = Knew
    }
    return LqrSolution(K, P)
}

private fun SimpleMatrix.elementMaxAbs(): Double {
    var m = 0.0
    for (i in 0 until numRows())
        for (j in 0 until numCols())
            m = maxOf(m, kotlin.math.abs(get(i, j)))
    return m
}

// ---------- Multik <-> EJML adapters ----------

fun D2Array<Double>.toSimple(): SimpleMatrix {
    val (m, n) = this.shape
    val sm = SimpleMatrix(m, n)
    for (i in 0 until m) for (j in 0 until n) sm.set(i, j, this[i, j])
    return sm
}

fun SimpleMatrix.toMK(): D2Array<Double> {
    val m = numRows(); val n = numCols()
    val data = Array(m) { DoubleArray(n) { j -> 0.0 } }
    for (i in 0 until m) for (j in 0 until n) data[i][j] = this.get(i, j)
    return mk.ndarray(data)
}

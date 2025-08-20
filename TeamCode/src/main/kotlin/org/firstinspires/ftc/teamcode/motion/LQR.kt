package org.firstinspires.ftc.teamcode.motion

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

        // Convert to EJML
        val A = a.toSimple()
        val B = b.toSimple()
        val Q = q.toSimple()
        val Rm = r.toSimple()

        val (K, P) = dare(A, B, Q, Rm)

        var newK = (K.negative()).toMK() // your code uses K with a leading minus: u = -Kx

        // Optional conservative stability check (no eigenvalues):
        val acl = (A.minus(B.mult(K))).toMK()
        val stableEnough = acl.spectralNorm() < 1.0   // from my earlier message
        if (!stableEnough && k != null) newK = k!!

        return newK
    }
}

/** Approx spectral norm ||A||_2 via power iteration on A^T A */
private fun D2Array<Double>.spectralNorm(maxIters: Int = 200, tol: Double = 1e-8): Double {
    val (m, n) = this.shape
    fun matmulAtA(x: DoubleArray): DoubleArray {
        // y = A^T (A x)
        val Ax = DoubleArray(m)
        for (i in 0 until m) {
            var s = 0.0
            for (j in 0 until n) s += this[i, j] * x[j]
            Ax[i] = s
        }
        val y = DoubleArray(n)
        for (j in 0 until n) {
            var s = 0.0
            for (i in 0 until m) s += this[i, j] * Ax[i]
            y[j] = s
        }
        return y
    }
    var v = DoubleArray(n) { if (it % 2 == 0) 1.0 else -1.0 }
    fun normalize(a: DoubleArray): DoubleArray {
        var s = 0.0; for (z in a) s += z*z
        val r = kotlin.math.sqrt(s); if (r == 0.0) return a
        for (i in a.indices) a[i] /= r
        return a
    }
    v = normalize(v)
    var lambda = 0.0
    repeat(maxIters) {
        val w = matmulAtA(v)
        val wn = normalize(w)
        val num = w.indices.sumOf { w[it] * wn[it] }
        if (kotlin.math.abs(num - lambda) < tol) {
            lambda = num; return@repeat
        }
        lambda = num; v = wn
    }
    return kotlin.math.sqrt(lambda.coerceAtLeast(0.0))
}


private fun D2Array<Double>.isPositiveDefinite(tol: Double = 1e-8): Boolean {
    if (!this.isSymmetric(tol)) return false
    return this.isPD_Cholesky(tol)
}

private fun D2Array<Double>.isPositiveSemiDefinite(tol: Double = 1e-8): Boolean {
    if (!this.isSymmetric(tol)) return false
    return this.isPSD_LDLt(tol)
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

private fun D2Array<Double>.isPD_Cholesky(tol: Double = 1e-10): Boolean =
    choleskyLowerOrNull(this, tol) != null

private fun D2Array<Double>.isPSD_LDLt(tol: Double = 1e-10): Boolean =
    ldltOrNull(this, tol)?.let { (_, D, _) -> D.all { it >= -tol } } ?: false

private fun choleskyLowerOrNull(A: D2Array<Double>, tol: Double = 1e-12): Array<DoubleArray>? {
    val n = A.shape[0]; if (n != A.shape[1]) return null
    val L = Array(n) { DoubleArray(n) }
    for (i in 0 until n) {
        for (j in 0..i) {
            var s = 0.0
            for (k in 0 until j) s += L[i][k]*L[j][k]
            val v = A[i, j] - s
            if (i == j) {
                if (v <= tol) return null
                L[i][j] = kotlin.math.sqrt(v)
            } else {
                L[i][j] = v / L[j][j]
            }
        }
    }
    return L
}

/** Very small/no-pivot LDLᵀ good enough for small symmetric Q. */
private fun ldltOrNull(A: D2Array<Double>, tol: Double = 1e-12): Triple<Array<DoubleArray>, DoubleArray, Array<DoubleArray>>? {
    val n = A.shape[0]; if (n != A.shape[1]) return null
    val L = Array(n) { DoubleArray(n) }
    val D = DoubleArray(n)
    for (i in 0 until n) L[i][i] = 1.0

    for (j in 0 until n) {
        var dj = A[j, j]
        for (k in 0 until j) dj -= D[k]*L[j][k]*L[j][k]
        D[j] = dj
        // If dj ~ 0, allow semidefinite; if strongly negative, fail.
        if (dj < -1e6*tol) return null
        val denom = if (kotlin.math.abs(dj) < tol) Double.POSITIVE_INFINITY else dj
        for (i in j+1 until n) {
            var lij = A[i, j]
            for (k in 0 until j) lij -= D[k]*L[i][k]*L[j][k]
            L[i][j] = if (denom.isInfinite()) 0.0 else lij / denom
        }
    }
    return Triple(L, D, L) // return L, D, (implicitly Lᵀ)
}

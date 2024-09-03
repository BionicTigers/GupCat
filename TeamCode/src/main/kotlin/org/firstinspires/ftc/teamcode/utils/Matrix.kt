package org.firstinspires.ftc.teamcode.utils

import kotlin.math.pow

class Matrix(private val rows: Int, private val columns: Int, value: Double = 0.0) {
    // Create a 2d array of rows x columns with the default value being 0
    private val matrix: Array<Array<Double>> = Array(rows) { Array(columns) { value } }

    /**
     * Get the entire row
     */
    operator fun get(i: Int): Array<Double> {
        return matrix[i]
    }

    /**
     * Get the value of an element in the matrix
     */
    operator fun get(i: Int, j: Int): Double {
        return matrix[i][j]
    }

    /**
     * Get the entire column
     */
    fun getColumn(i: Int): Array<Double> {
        if (columns - 1 <= i)
            throw IndexOutOfBoundsException()

        val column: Array<Double> = Array(rows) { 0.0 }

        for (row in matrix.iterator().withIndex()) {
            column[row.index] = row.value[i]
        }

        return column
    }

    /**
     * Set an entire row
     */
    operator fun set(i: Int, b: Array<Double>) {
        if (rows - 1 >= i) {
            if (b.size != columns)
                throw RuntimeException("Size Mismatch: Size of array does not equal columns")

            matrix[i] = b
        } else {
            throw IndexOutOfBoundsException()
        }
    }

    /**
     * Set a value
     */
    operator fun set(i: Int, j: Int, b: Double) {
        if (rows - 1 >= i && columns - 1 >= j) {
            matrix[i][j] = b
        } else {
            throw IndexOutOfBoundsException()
        }
    }

    /**
     * Set an entire column
     */
    fun setColumn(i: Int, b: Array<Double>) {
        if (columns - 1 <= i)
            throw IndexOutOfBoundsException()

        for (row in matrix.iterator().withIndex()) {
            row.value[i] = b[row.index]
        }
    }

    fun getSize(): Pair<Int, Int> {
        return Pair(rows, columns)
    }

    operator fun plus(b: Matrix): Matrix {
        if (this.getSize() != b.getSize())
            throw RuntimeException("Matrix size does not match")

        val result = Matrix(rows, columns)

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                result[row, column] = matrix[row][column] + b[row, column]
            }
        }

        return result
    }

    operator fun minus(b: Matrix): Matrix {
        if (this.getSize() != b.getSize())
            throw RuntimeException("Matrix size does not match")

        val result = Matrix(rows, columns)

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                result[row, column] = matrix[row][column] - b[row, column]
            }
        }

        return result
    }

    operator fun times(b: Double): Matrix {
        val result = Matrix(rows, columns)

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                result[row, column] = matrix[row][column] * b
            }
        }

        return result
    }

    operator fun times(b: Matrix): Matrix {
        if (columns != b.rows)
            throw RuntimeException("Cannot multiply matrices of incompatible sizes")

        val result = Matrix(rows, b.columns)

        for (i in 0 until rows) {
            for (j in 0 until b.columns) {
                var sum = 0.0
                for (k in 0 until columns) {
                    sum += matrix[i][k] * b[k, j]
                }
                result[i, j] = sum
            }
        }

        return result
    }

    operator fun div(b: Double): Matrix {
        if (b == 0.0)
            throw ArithmeticException("Division by zero")

        val result = Matrix(rows, columns)

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                result[row, column] = matrix[row][column] / b
            }
        }

        return result
    }

    operator fun div(b: Matrix): Matrix {
        return this * b.inverse()
    }

    fun determinant(): Double {
        if (rows != columns)
            throw RuntimeException("Matrix is not square")

        if (rows == 1) {
            return matrix[0][0]
        }

        var det = 0.0

        for (i in 0 until rows) {
            det += matrix[0][i] * cofactor(0, i)
        }

        return det
    }

    private fun subMatrix(row: Int, col: Int): Matrix {
        val sub = Matrix(rows - 1, columns - 1)
        var subRow = 0
        var subCol: Int
        for (i in 0 until rows) {
            if (i != row) {
                subCol = 0
                for (j in 0 until columns) {
                    if (j != col) {
                        sub[subRow, subCol++] = matrix[i][j]
                    }
                }
                subRow++
            }
        }
        return sub
    }

    private fun cofactor(row: Int, col: Int): Double {
        return (-1.0).pow(row + col) * subMatrix(row, col).determinant()
    }

    fun inverse(): Matrix {
        val det = determinant()
        if (det == 0.0)
            throw ArithmeticException("Matrix is not invertible")

        val inverse = Matrix(rows, columns)
        for (i in 0 until rows) {
            for (j in 0 until columns) {
                inverse[j, i] = cofactor(i, j) / det
            }
        }
        return inverse
    }
}

package org.firstinspires.ftc.teamcode.utils

class Matrix(private val rows: Int, private val columns: Int, value: Double = 0.0) {
    //Create a 2d array of rows x columns with the default value being 0
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
        if (rows-1 >= i) {
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
        if (rows-1 >= i && columns-1 >= j) {
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

    operator fun plus(b: Matrix) {
        if (this.getSize() == b.getSize())
            throw RuntimeException("Matrix size does not match")

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] -= b[row][column]
            }
        }
    }

    operator fun minus(b: Matrix) {
        if (this.getSize() == b.getSize())
            throw RuntimeException("Matrix size does not match")

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] -= b[row][column]
            }
        }
    }

    operator fun times(b: Double) {
        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] *= b
            }
        }
    }

    operator fun times(b: Matrix) {
        if (this.getSize() == b.getSize())
            throw RuntimeException("Matrix size does not match")

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] *= b[row][column]
            }
        }
    }

    operator fun div(b: Double) {
        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] /= b
            }
        }
    }

    operator fun div(b: Matrix) {
        if (this.getSize() == b.getSize())
            throw RuntimeException("Matrix size does not match")

        for (row in matrix.indices) {
            for (column in matrix[row].indices) {
                matrix[row][column] /= b[row][column]
            }
        }
    }
}
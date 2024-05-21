#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <omp.h>
#include "mpi.h"

/**************************************************************************************************************
  Matrix Class

  Purpose: Definition and operations for matrices.

  Member Functions:
  - Matrix: Constructor for the Matrix class.
  - getRows: Get the number of rows in the matrix.
  - getCols: Get the number of columns in the matrix.
  - operator: Access a specific element in the matrix.
  - MatrixAddition: Matrix addition.
  - MatrixSubtraction: Matrix subtraction.
  - MatrixMultiplication: Matrix multiplication.
  - DotProduct: Vector dot product.
  - MatrixTransposition: Matrix transposition.
  - InverseMatrix: Matrix inversion.
  - Magnitude: Vector magnitude.
**************************************************************************************************************/

class Matrix
{
private:
    int rows;
    int cols;
    std::vector<double> elements;

public:
    Matrix(int r, int c);

    int getRows() const;

    int getCols() const;

    double& operator()(int i, int j);

    double operator()(int i, int j) const;

    void MatrixAddition(const Matrix& A, const Matrix& B);

    void MatrixSubtraction(const Matrix& A, const Matrix& B);

    void MatrixMultiplication(const Matrix& A, const Matrix& B);

    double DotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2);

    void MatrixTransposition(const Matrix& input);

    int InverseMatrix(const Matrix& inputMatrix);

    double Magnitude();
};

#endif 

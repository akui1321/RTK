#include "Matrix.h"

//Matrix Definition
Matrix::Matrix(int r, int c) : rows(r), cols(c)
{
    if (r <= 0 || c <= 0) 
    {
        throw std::invalid_argument("Rows and columns must be greater than zero");
    }
    elements.resize(r * c, 0);
}

int Matrix::getRows() const
{
    return rows;
}

int Matrix::getCols() const
{
    return cols;
}

double& Matrix::operator()(int i, int j)
{
    if (i < 0 || i >= rows || j < 0 || j >= cols)
    {
        throw std::out_of_range("Invalid indices");
    }
    return elements[i * cols + j];
}

double Matrix::operator()(int i, int j) const
{
    if (i < 0 || i >= rows || j < 0 || j >= cols)
    {
        throw std::out_of_range("Invalid indices");
    }
    return elements[i * cols + j];
}

//Matrix Addition
void Matrix::MatrixAddition(const Matrix& A, const Matrix& B)
{
    if (A.getRows() != B.getRows() || A.getCols() != B.getCols())
    {
        throw std::invalid_argument("Matrices must have the same dimensions for addition");
    }

    if (A.getRows() != rows || A.getCols() != cols)
    {
        throw std::invalid_argument("Result matrix must have the same dimensions as input matrices");
    }

    int Arows = A.getRows();
    int Acols = A.getCols();

    for (int i = 0; i < Arows; ++i)
    {
        for (int j = 0; j < Acols; ++j)
        {
            elements[i * cols + j] = A(i, j) + B(i, j);
        }
    }
}

//Matrix Subtraction
void Matrix::MatrixSubtraction(const Matrix& A, const Matrix& B)
{
    if (A.getRows() != B.getRows() || A.getCols() != B.getCols())
    {
        throw std::invalid_argument("Matrices must have the same dimensions for subtraction");
    }

    if (A.getRows() != rows || A.getCols() != cols)
    {
        throw std::invalid_argument("Result matrix must have the same dimensions as input matrices");
    }

    int Arows = A.getRows();
    int Acols = A.getCols();

    for (int i = 0; i < Arows; ++i)
    {
        for (int j = 0; j < Acols; ++j)
        {
            elements[i * cols + j] = A(i, j) - B(i, j);
        }
    }
}

//Matrix Multiplication
void Matrix::MatrixMultiplication(const Matrix& A, const Matrix& B)
{
    if (A.getCols() != B.getRows())
    {
        throw std::invalid_argument("Number of columns in first matrix must be equal to number of rows in second matrix");
    }

    if (A.getRows() != rows || B.getCols() != cols)
    {
        throw std::invalid_argument("Result matrix must have correct dimensions for multiplication");
    }

    int rowsA = A.getRows();
    int colsA = A.getCols();
    int colsB = B.getCols();

    omp_set_num_threads(64);

    if (rowsA >= 20 || colsB >= 20 || colsA >= 20)
    {
#pragma omp parallel for
        for (int i = 0; i < rowsA; ++i)
        {
            for (int j = 0; j < colsB; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < colsA; ++k)
                {
                    sum += A(i, k) * B(k, j);
                }
                elements[i * cols + j] = sum;
            }
        }
    }
    else
    {
        for (int i = 0; i < rowsA; ++i)
        {
            for (int j = 0; j < colsB; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < colsA; ++k)
                {
                    sum += A(i, k) * B(k, j);
                }
                elements[i * cols + j] = sum;
            }
        }
    }
}

//Dot Product
double Matrix::DotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2)
{
    if (vec1.size() != vec2.size())
    {
        throw std::invalid_argument("Vectors must have the same size for dot product");
    }

    double result = 0.0;

    for (size_t i = 0; i < vec1.size(); ++i)
    {
        result += vec1[i] * vec2[i];
    }

    return result;
}

//Matrix Transposition
void Matrix::MatrixTransposition(const Matrix& input)
{
    int inputRows = input.getRows();
    int inputCols = input.getCols();

    if (inputRows != cols || inputCols != rows)
    {
        throw std::invalid_argument("Result matrix must have dimensions swapped for transpose");
    }


    for (int i = 0; i < cols; ++i)
    {
        for (int j = 0; j < rows; ++j)
        {
            elements[j * cols + i] = input(i, j);
        }
    }
}

//Matrix Inversion
int Matrix::InverseMatrix(const Matrix& inputMatrix)
{
    std::vector<double> a, b;
    int n = rows;
    int num = rows * cols;

    for (int i = 0; i < num; ++i)
    {
        a.push_back(inputMatrix.elements[i]);
    }


    int i, j, k, l, u, v, is[250], js[250];   /* matrix dimension <= 250 */
    double d, p;

    if (n <= 0)
    {
        printf("Error dimension in MatrixInv!\n");
        return 0;
    }

    b = a;
    
    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   /* Find the positions of the leading elements in the bottom-right corner matrix */
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < DBL_EPSILON)   /* The matrix is not invertible */
        {
            printf("Divided by 0 in MatrixInv!\n");
            return 0;
        }

        if (is[k] != k) 
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        if (js[k] != k) 
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l]; 
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    for (i = 0; i < n; i++) 
    {
        for (j = 0; j < n; j++) 
        {
            elements[i * cols + j] = b[i * n + j];
        }
    }
    
    return (1);
}

//Calculate the Vector's Magnitude
double Matrix::Magnitude()
{
    if (cols != 1)
    {
        throw std::invalid_argument("Unable to calculate the magnitude of the vector!");
    }
    
    double sq = 0;
    for (int i = 0; i < rows; ++i)
    {
        sq += (elements[i] * elements[i]);
    }
    
    return sqrt(sq);
}

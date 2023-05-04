//
// Created by 39894 on 2023/3/27.
//

#include "Algorithm.h"
#include "Vec.hpp"
#include <vector>
#include <tuple>


std::optional<std::tuple<Matrix, Vec>> LDLDecompose(Matrix Q)
{
    int n = Q.Row();
    Matrix L = Matrix::Zeros(n, n);
    Vec D = Vec::Zeros(n);
    for (int i = n - 1; i >= 0; i--)
    {
        D(i + 1) = Q(i + 1, i + 1);
        if (D(i + 1) <= 0.0)
        {
            return std::nullopt;
        }
        double a = sqrt(D(i + 1));
        for (int j = 0; j <= i; j++)
        {
            L(i + 1, j + 1) = Q(i + 1, j + 1) / a;
        }
        for (int j = 0; j <= i - 1; j++)
        {
            for (int k = 0; k <= j; k++)
            {
                Q(j + 1, k + 1) -= L(i + 1, k + 1) * L(i + 1, j + 1);
            }
        }
        for (int j = 0; j <= i; j++)
        {
            L(i + 1, j + 1) /= L(i + 1, i + 1);
        }
    }
    return std::make_optional(std::make_tuple(L, D));
}

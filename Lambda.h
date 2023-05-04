//
// Created by 39894 on 2023/3/27.
//

#pragma once

#include "LinerAlgebra/Algorithm.h"
#include "LinerAlgebra/Matrix.hpp"
#include "LinerAlgebra/Vec.hpp"

#define SGN(x) ((x) <= 0.0 ? -1.0 : 1.0)
#define SWAP(x,y) do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

inline void Gauss(Matrix& L, Matrix& Z, int i, int j)
{
    auto mu = lround(L(i + 1, j + 1));
    int n = L.Row();
    if (mu != 0)
    {
        for (int k = i; k < n; k++)
            L(k + 1, j + 1) -= mu * L(k + 1, i + 1);
        for (int k = 0; k < n; k++)
            Z(k + 1, j + 1) -= mu * Z(k + 1, i + 1);
    }
}

inline void Perm(Matrix& L, Vec& D, int j, double del, Matrix& Z)
{
    double eta = D(j + 1) / del;
    double lam = D(j + 2) * L(j + 2, j + 1) / del;
    D(j + 1) = eta * D(j + 2);
    D(j + 2) = del;
    int n = L.Row();
    for (int k = 0; k <= j - 1; k++)
    {
        double a0 = L(j + 1, k + 1);
        double a1 = L(j + 2, k + 1);
        L(j + 1, k + 1) = -L(j + 2, j + 1) * a0 + a1;
        L(j + 2, k + 1) = eta * a0 + lam * a1;
    }
    L(j + 2, j + 1) = lam;
    for (int k = j + 2; k < n; k++)
        SWAP(L(k + 1, j + 1), L(k + 1, j + 2));
    for (int k = 0; k < n; k++)
        SWAP(Z(k + 1, j + 1), Z(k + 1, j + 2));
}

inline void Reduction(Matrix& L, Vec& D, Matrix& Z)
{
    int n = L.Row();
    int j = n - 2;
    int k = n - 2;
    while (j >= 0)
    {
        if (j <= k)
            for (int i = j + 1; i < n; i++)
                Gauss(L, Z, i, j);
        double del = D(j + 1) + L(j + 2, j + 1) * L(j + 2, j + 1) * D(j + 2);
        if (del + 1e-6 < D(j + 2))
        {
            Perm(L, D, j, del, Z);
            k = j;
            j = n - 2;
        }
        else
            j--;
    }
}

inline bool Search(int m, Matrix& L, Vec& D, Vec& zs, Matrix& zn, Vec& s)
{
    int i, j, k, c, nn = 0, imax = 0;
    double newdist, maxdist = DBL_MAX, y;
    int n = L.Row();
    Vec dist(n), zb(n), z(n, 0), step(n);
    Matrix S = Matrix::Zeros(n, n);

    k = n - 1;
    dist(k + 1) = 0;
    zb(k + 1) = zs(k + 1);
    z(k + 1) = std::round(zb(k + 1));
    y = zb(k + 1) - z(k + 1);
    step(k + 1) = (y <= 0.0) ? -1 : 1;
    for (c = 0; c < 100000; c++)
    {
        newdist = dist(k + 1) + y * y / D(k + 1);
        if (newdist < maxdist)
        {
            if (k != 0)
            {
                dist((--k) + 1) = newdist;
                for (i = 0; i <= k; i++)
                    S(k + 1, i + 1) = S(k + 2, i + 1) + (z(k + 2) - zb(k + 2)) * L(k + 2, i + 1);
                zb(k + 1) = zs(k + 1) + S(k + 1, k + 1);
                z(k + 1) = std::round(zb(k + 1));
                y = zb(k + 1) - z(k + 1);
                step(k + 1) = (y <= 0.0) ? -1 : 1;
            } else
            {
                if (nn < m)
                {
                    if (nn == 0 || newdist > s(imax + 1))
                        imax = nn;
                    for (i = 0; i < n; i++)
                        zn(i + 1, nn + 1) = z(i + 1);
                    s((nn++) + 1) = newdist;
                } else
                {
                    if (newdist < s(imax + 1))
                    {
                        for (i = 0; i < n; i++)
                            zn(i + 1, imax + 1) = z(i + 1);
                        s(imax + 1) = newdist;
                        for (i = imax = 0; i < m; i++)
                            if (s(imax + 1) < s(i + 1))
                                imax = i;
                    }
                    maxdist = s(imax + 1);
                }
                z(1) += step(1);
                y = zb(1) - z(1);
                step(1) = -step(1) - SGN(step(1));
            }
        } else
        {
            if (k == n - 1)
                break;
            else
            {
                k++;
                z(k + 1) += step(k + 1);
                y = zb(k + 1) - z(k + 1);
                step(k + 1) = -step(k + 1) - SGN(step(k + 1));
            }
        }
    }
    for (i = 0; i < m - 1; i++)
    {
        for (j = i + 1; j < m; j++)
        {
            if (s(i + 1) < s(j + 1))
                continue;
            SWAP(s(i + 1), s(j + 1));
            for (k = 0; k < n; k++)
                SWAP(zn(k + 1, i + 1), zn(k + 1, j + 1));
        }
    }
    return c < 100000;
}

inline std::optional<std::tuple<Matrix, double>> Lambda(int m, Vec& a, Matrix& Q)
{
    bool info = false;
    int n = Q.Row();
    Matrix Z = Matrix::Identity(n);
    auto ld = LDLDecompose(Q);
    Vec s(2);
    if (ld.has_value())
    {
        auto [L, D] = ld.value();
        Reduction(L, D, Z);
        Vec z = Z.Transpose() * a;
        Matrix E(n, m);
        info = Search(m, L, D, z, E, s);
        if (info)
        {
            Matrix F = Z.Inverse().Transpose() * E;
            return std::make_optional(std::make_tuple(F, s(2) / s(1)));
        }
    }
    return std::nullopt;
}
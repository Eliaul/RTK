//
// Created by 39894 on 2023/3/27.
//

#pragma once

#include <optional>
#include "Matrix.hpp"

std::optional<std::tuple<Matrix, Vec>> LDLDecompose(Matrix Q);
//
// Created by 39894 on 2023/4/10.
//

#pragma once

#include <string>
#include "FixedSolution.h"

std::string ExportCSV(FixedSolution& fixedSolution, DoubleDifferenceOBS& ddOBS, FloatSolution& floatSolution, int rtk, int total);

std::string ExportCSV(FixedSolution &fixedSolution, DoubleDifferenceOBS& ddOBS, FloatSolution& floatSolution);

std::string ExportCSV(FloatSolution& floatSolution);


#pragma once

#include <vector>

namespace Eloy {
namespace SolverUtils {
    void countingSort(
        std::vector<int>& initial_indices,
        std::vector<int>& sorted_indices,
        const int num_cells
    );
}
}
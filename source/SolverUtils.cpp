
#include "SolverUtils.hpp"

#include <string.h>

namespace Eloy {
namespace SolverUtils {

static std::vector<int> counts;

void countingSort(
    std::vector<int>& initial_indices,
    std::vector<int>& sorted_indices,
    const int num_cells
) {
    counts.resize(num_cells + 2);
    std::fill(counts.begin(), counts.end(), 0);
    //memset(counts, 0, (num_cells + 1) * sizeof(int));
    for (int i = 0; i < initial_indices.size(); i++) {
        counts[initial_indices[i]]++;
    }

    for (int i = 1; i <= num_cells; i++) {
        counts[i] += counts[i - 1];
    }

    int length = initial_indices.size();
    for (int i = length - 1; i >= 0; i--) {
        sorted_indices[counts[initial_indices[i]] - 1] = i;
        counts[initial_indices[i]]--;
    }

}

}
}
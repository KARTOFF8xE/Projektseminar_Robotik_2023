#pragma once

#include <vector>
#include "filters/filters.hpp"

namespace visualize_path {
    /**
     * Get the Streetview from the recorded Data
     * 
     * @param limits_vec: a Vector where the Limits of all records are inside
     * 
     * @returns nothing
    */
    void visualize_street_view(
        std::vector<filters::limit> limits_vec,
        double wheel_inside,
        double wheel_width
    );
}
// Copyright 2023 The VGC Developers
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VGC_GEOMETRY_INTERSECT_H
#define VGC_GEOMETRY_INTERSECT_H

#include <optional>
#include <utility>

#include <vgc/geometry/api.h>
#include <vgc/geometry/vec2d.h>

namespace vgc::geometry {

/// Returns whether the segments `(a1, a2)` and `(b1, b2)` intersect.
///
/// It is a fast variant that considers collinear overlaps as non
/// intersecting.
///
inline bool fastSegmentIntersects(
    const geometry::Vec2d& a1,
    const geometry::Vec2d& a2,
    const geometry::Vec2d& b1,
    const geometry::Vec2d& b2) {

    geometry::Vec2d d1 = (a2 - a1);
    geometry::Vec2d d2 = (b2 - b1);

    // Solve 2x2 system using Cramer's rule.
    double delta = d1.det(d2);
    if (std::abs(delta) > core::epsilon) {
        geometry::Vec2d a1b1 = b1 - a1;
        double inv_delta = 1 / delta;
        double t1 = a1b1.det(d2) * inv_delta;
        double t2 = a1b1.det(d1) * inv_delta;
        if (t1 >= 0. && t1 <= 1. && t2 >= 0. && t2 <= 1.) {
            return true;
        }
    }

    return false;
}

/// Returns whether the segments `(a1, a2)` and `(b1, b2)` intersect
/// excluding `a2` and `b2`.
///
/// It is a fast variant that considers collinear overlaps as non
/// intersecting.
///
inline bool fastSemiOpenSegmentIntersects(
    const geometry::Vec2d& a1,
    const geometry::Vec2d& a2,
    const geometry::Vec2d& b1,
    const geometry::Vec2d& b2) {

    geometry::Vec2d d1 = (a2 - a1);
    geometry::Vec2d d2 = (b2 - b1);

    // Solve 2x2 system using Cramer's rule.
    double delta = d1.det(d2);
    if (std::abs(delta) > core::epsilon) {
        geometry::Vec2d a1b1 = b1 - a1;
        double inv_delta = 1 / delta;
        double t1 = a1b1.det(d2) * inv_delta;
        double t2 = a1b1.det(d1) * inv_delta;
        if (t1 >= 0. && t1 < 1. && t2 >= 0. && t2 < 1.) {
            return true;
        }
    }

    return false;
}

/// If the segments `(a1, a2)` and `(b1, b2)` intersect (excluding `a2` and `b2`),
/// returns a pair `(ta, tb)` where `ta` (resp. `tb`) is the linear parametric
/// position of the intersection on the segment `(a1, a2)` (resp. `(b1, b2)`).
/// Otherwise returns std::nullopt.
///
inline std::optional<std::pair<double, double>> semiOpenSegmentIntersection(
    const geometry::Vec2d& a1,
    const geometry::Vec2d& a2,
    const geometry::Vec2d& b1,
    const geometry::Vec2d& b2) {

    geometry::Vec2d da = (a2 - a1);
    geometry::Vec2d db = (b2 - b1);

    // Solve 2x2 system using Cramer's rule.
    geometry::Vec2d a1b1 = b1 - a1;
    double xa = a1b1.det(da);
    double xb = a1b1.det(db);
    double delta = da.det(db);
    if (std::abs(delta) > core::epsilon) {
        // non-parallel
        double inv_delta = 1 / delta;
        double ta = xb * inv_delta;
        double tb = xa * inv_delta;
        if (ta >= 0. && ta < 1. && tb >= 0. && tb < 1.) {
            return std::pair(ta, tb);
        }
    }
    else if (std::abs(xa) < core::epsilon || std::abs(xb) < core::epsilon) {
        // colinear
        // tb1 = a1b1 · da / (da · da)
        // tb2 = t0 + db · da / (da · da)
        // TODO
    }

    return std::nullopt;
}

} // namespace vgc::geometry

#endif // VGC_GEOMETRY_INTERSECT_H

#pragma once

#include <Eigen/Dense>

// alias per i tipi più usati
using IntPoint = Eigen::Vector2i;  // punto con coordinate intere (pixel)
using Point = Eigen::Vector2f;     // punto con coordinate float (metri)
using Pose = Eigen::Isometry2f;    // posa 2D (posizione + orientamento)
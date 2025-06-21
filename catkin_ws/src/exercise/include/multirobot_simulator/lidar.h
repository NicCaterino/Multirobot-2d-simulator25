#pragma once

#include <vector>
#include "multirobot_simulator/world.h"

class Lidar : public WorldItem {
public:
    // costruttori: lidar attaccato al mondo o a un parent (robot)
    Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<World> w,
          const Pose& pose_ = Pose::Identity());
    Lidar(float fov_, float max_range_, int num_beams_,
          std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose::Identity());

    void timeTick(float dt) override;
    void draw() override;

    float fov, max_range;     // campo visivo e range massimo
    int num_beams;            // numero di raggi
    std::vector<float> ranges; // distanze misurate
};
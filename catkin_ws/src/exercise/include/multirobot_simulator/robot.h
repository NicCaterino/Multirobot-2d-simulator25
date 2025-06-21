#pragma once

#include "multirobot_simulator/types.h"
#include "multirobot_simulator/world.h"

struct Robot : public WorldItem {
    // costruttori: robot attaccato al mondo o a un parent
    Robot(float radius_, std::shared_ptr<World> w_,
          const Pose& pose_ = Pose::Identity());
    Robot(float radius_, std::shared_ptr<WorldItem> parent_,
          const Pose& pose_ = Pose::Identity());

    void draw() override;
    void timeTick(float dt) override;

    float radius;          // raggio del robot
    float tv = 0, rv = 0;  // velocità lineare e angolare
};
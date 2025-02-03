
#ifndef BOID_MANAGER_HPP
#define BOID_MANAGER_HPP
#pragma once

#include <array>

#include "boid.hpp"

struct Vector2;

class BoidManager {
public:
    BoidManager( const Vector2 Bounds_ );
    void update();
    void draw() const;

private:
    Vector2 accumulatePosition() const;
    Vector2 accumulateVelocity() const;

    Vector2 Bounds;

    static const size_t MAX = 10;
    std::array< BoidPtr, MAX > BoidList;
};

#endif

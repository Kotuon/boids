
#ifndef BOID_HPP
#define BOID_HPP
#pragma once

#include <memory>

#include "raylib.h"

class Boid {
public:
    Boid( const Vector2& Position_, const float Scale_ );
    Boid( const Vector2& Position_, const Vector2& Velocity_,
          const float Scale_, const float SimScale_ );

    void update();
    void draw() const;

    const Vector2 boundPosition( const Vector2& Bounds ) const;

    void setVelocity( const Vector2& Velocity_ );
    void setPosition( const Vector2& Velocity_ );

    const Vector2& getPosition() const;
    const Vector2& getVelocity() const;

private:
    Vector2 Position = { 0.f };
    Vector2 Velocity = { 0.f };

    float Scale = 7.5f;
    float SimScale = 1.f;

    float BoundCorrection = 1.f;

    const Vector2 Fwd = { 1.f, 0.f };
};

using BoidPtr = std::unique_ptr< Boid >;

#endif

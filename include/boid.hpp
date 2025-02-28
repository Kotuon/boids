
#ifndef BOID_HPP
#define BOID_HPP
#pragma once

#include <limits>
#include <memory>

#include "raylib.h"

struct BoidsUpdateValues {
    BoidsUpdateValues()
        : AvgVelocity( 0.f ), AvgPosition( 0.f ), AvgAvoid( 0.f ), Count( 0 ) {}

    Vector2 AvgVelocity;
    Vector2 AvgPosition;
    Vector2 AvgAvoid;
    size_t Count;
};

class Boid {
public:
    Boid( const Vector2& Position_, const float Scale_ );
    Boid( const Vector2& Position_, const Vector2& Velocity_,
          const float Scale_, const float SimScale_, const size_t Id_ );

    void update();
    void draw() const;

    const Vector2 boundPosition( const Vector2& Bounds ) const;

    void setVelocity( const Vector2& Velocity_ );
    void setPosition( const Vector2& Velocity_ );

    const Vector2& getPosition() const;
    const Vector2& getVelocity() const;

    const size_t getId() const;

private:
    Vector2 Position = { 0.f };
    Vector2 Velocity = { 0.f };

    float Scale = 7.5f;
    float SimScale = 1.f;

    float BoundCorrection = 1.f;

    const Vector2 Fwd = { 1.f, 0.f };

    size_t Id = std::numeric_limits< int >::max();
};

using BoidPtr = std::unique_ptr< Boid >;

#endif

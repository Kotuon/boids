
#include "boid.hpp"

#include "raymath.h"

#include <fmt/core.h>
#include "trace.hpp"

Boid::Boid()
    : Position( 0.f ), Scale( 7.5f ), Speed( 0.f ), Acceleration( 0.f ),
      Rotation( 0.f ) {}

Boid::Boid( Vector2 Position_ )
    : Position( Position_ ), Scale( 7.5f ), Speed( 0.f ), Acceleration( 0.f ),
      Rotation( 0.f ) {}

Boid::Boid( Vector2 Position_, float Scale_ )
    : Position( Position_ ), Scale( Scale_ ), Speed( 0.f ), Acceleration( 0.f ),
      Rotation( 0.f ) {}

void Boid::update() {}

void Boid::draw() const {
    const float Angle = Vector2Angle( Fwd, Velocity );

    DrawTriangle(
        Vector2Add( Position,
                    Vector2Rotate( Vector2{ Scale * 2.f, 0.f }, Angle ) ),
        Vector2Add( Position,
                    Vector2Rotate( Vector2{ -Scale, -Scale }, Angle ) ),
        Vector2Add( Position,
                    Vector2Rotate( Vector2{ -Scale, Scale }, Angle ) ),
        GREEN );
}

const Vector2 Boid::boundPosition( const Vector2& Bounds ) const {
    Vector2 Result( 0.f );

    if ( Position.x < 0.f )
        Result.x = BoundCorrection;
    else if ( Position.x > Bounds.x )
        Result.x = -BoundCorrection;

    if ( Position.y < 0.f )
        Result.y = BoundCorrection;
    else if ( Position.y > Bounds.y )
        Result.y = -BoundCorrection;

    return Result;
}

void Boid::setSpeed( const float Speed_ ) { Speed = Speed_; }

void Boid::setVelocity( const Vector2 Velocity_ ) { Velocity = Velocity_; }
void Boid::setPosition( const Vector2 Position_ ) { Position = Position_; }

const Vector2& Boid::getPosition() const { return Position; }

const Vector2& Boid::getVelocity() const { return Velocity; }

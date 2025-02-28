
#include "boid.hpp"

#include "raymath.h"

#include <fmt/core.h>
#include "trace.hpp"

Boid::Boid( const Vector2& Position_, const float Scale_ )
    : Position( Position_ ), Scale( Scale_ ) {}

Boid::Boid( const Vector2& Position_, const Vector2& Velocity_,
            const float Scale_, const float SimScale_, const size_t Id_ )
    : Position( Position_ ), Velocity( Velocity_ ), Scale( Scale_ ),
      SimScale( SimScale_ ), Id( Id_ ) {}

void Boid::update() {}

void Boid::draw() const {
    const float Angle = Vector2Angle( Fwd, Velocity );

    DrawRectangleLines( static_cast< int >( Position.x - ( 50.f * SimScale ) ),
                        static_cast< int >( Position.y - ( 50.f * SimScale ) ),
                        static_cast< int >( 100.f * SimScale ),
                        static_cast< int >( 100.f * SimScale ), BLUE );

    DrawTriangle(
        Vector2Add(
            Position,
            Vector2Rotate( Vector2{ SimScale * Scale * 2.f, 0.f }, Angle ) ),
        Vector2Add( Position, Vector2Rotate( Vector2{ SimScale * -Scale,
                                                      SimScale * -Scale },
                                             Angle ) ),
        Vector2Add( Position, Vector2Rotate( Vector2{ SimScale * -Scale,
                                                      SimScale * Scale },
                                             Angle ) ),
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

void Boid::setVelocity( const Vector2& Velocity_ ) { Velocity = Velocity_; }
void Boid::setPosition( const Vector2& Position_ ) { Position = Position_; }

const Vector2& Boid::getPosition() const { return Position; }

const Vector2& Boid::getVelocity() const { return Velocity; }

const size_t Boid::getId() const { return Id; }

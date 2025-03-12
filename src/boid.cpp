
#include "boid.hpp"

#include <glm/ext/matrix_transform.hpp>

#include <fmt/core.h>
#include "trace.hpp"

#include "graphics.hpp"

Boid::Boid( const glm::vec2& Position_ ) : Position( Position_ ) {}

Boid::Boid( const glm::vec2& Position_, const glm::vec2& Velocity_,
            const size_t Id_ )
    : Position( Position_ ), Velocity( Velocity_ ), Id( Id_ ) {}

void Boid::init( const glm::vec2& Pos_, const glm::vec2& Vel_,
                 const size_t Id_ ) {
    Position = Pos_;
    Velocity = Vel_;
    Id = Id_;
}

void Boid::update() {}

const glm::vec2 Boid::boundPosition( const glm::vec2& Bounds,
                                     const float BoundCorrection ) const {
    glm::vec2 Result( 0.f );

    if ( Position.x < Bounds.x )
        Result.x = BoundCorrection;
    else if ( Position.x > Bounds.y )
        Result.x = -BoundCorrection;

    if ( Position.y < Bounds.x )
        Result.y = BoundCorrection;
    else if ( Position.y > Bounds.y )
        Result.y = -BoundCorrection;

    return Result;
}

void Boid::setVelocity( const glm::vec2& Velocity_ ) { Velocity = Velocity_; }
void Boid::setPosition( const glm::vec2& Position_ ) { Position = Position_; }

const glm::vec2& Boid::getPosition() const { return Position; }

const glm::vec2& Boid::getVelocity() const { return Velocity; }

const glm::vec2& Boid::getFwd() const { return Fwd; }

const size_t Boid::getId() const { return Id; }

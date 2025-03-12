
#ifndef BOID_HPP
#define BOID_HPP
#pragma once

#include <limits>
#include <memory>

#include <glm/glm.hpp>

struct BoidsUpdateValues {
    BoidsUpdateValues()
        : AvgVelocity( 0.f ), AvgPosition( 0.f ), AvgAvoid( 0.f ), Count( 0 ) {}

    glm::vec2 AvgVelocity;
    glm::vec2 AvgPosition;
    glm::vec2 AvgAvoid;
    size_t Count;
};

class Boid {
public:
    Boid( const glm::vec2& Position_ );
    Boid( const glm::vec2& Position_, const glm::vec2& Velocity_,
          const size_t Id_ );

    void init( const glm::vec2& Pos_, const glm::vec2& Vel_, const size_t Id_ );

    void update();

    const glm::vec2 boundPosition( const glm::vec2& Bounds,
                                   const float BoundCorrection ) const;

    void setVelocity( const glm::vec2& Velocity_ );
    void setPosition( const glm::vec2& Velocity_ );

    const glm::vec2& getPosition() const;
    const glm::vec2& getVelocity() const;

    const glm::vec2& getFwd() const;

    const size_t getId() const;

private:
    glm::vec2 Position{ 0.f };
    glm::vec2 Velocity{ 0.f };

    const glm::vec2 Fwd = { 1.f, 0.f };

    size_t Id = std::numeric_limits< int >::max();
};

using BoidPtr = std::unique_ptr< Boid >;

#endif


#include "boid_manager.hpp"

#include <numeric>

#include "raymath.h"

#include <fmt/core.h>
#include "trace.hpp"

BoidManager::BoidManager( const Vector2 Bounds_ ) : Bounds( Bounds_ ) {
    const float Scale = LocalSize / 13.f;

    for ( size_t i = 0; i < MAX; ++i ) {
        const Vector2 Pos( static_cast< float >( GetRandomValue(
                               0, static_cast< int >( Bounds.x ) ) ),
                           static_cast< float >( GetRandomValue(
                               0, static_cast< int >( Bounds.y ) ) ) );

        const Vector2 Vel( static_cast< float >( GetRandomValue( -5, 5 ) ),
                           static_cast< float >( GetRandomValue( -5, 5 ) ) );

        BoidList[i] = std::make_unique< Boid >( Pos, Vel, Scale );
    }
}

void BoidManager::update() {
    for ( auto& Boid1 : BoidList ) {
        Vector2 AvgPosition( 0.f ); // Cohesion
        Vector2 AvgVelocity( 0.f ); // Alignment
        Vector2 AvgAvoid( 0.f );    // Seperation
        size_t Count = 0;

        for ( auto& Boid2 : BoidList ) {
            const float Distance =
                Vector2Distance( Boid1->getPosition(), Boid2->getPosition() );
            if ( Distance >= LocalSize ) continue;

            Count += 1;
            // Alignment
            AvgVelocity = Vector2Add( AvgVelocity, Boid2->getVelocity() );
            // Cohesion
            AvgPosition = Vector2Add( AvgPosition, Boid2->getPosition() );
            // Separation
            if ( Distance >= LocalSize * 0.4f ) continue;
            AvgAvoid = Vector2Subtract(
                AvgAvoid, Vector2Normalize( Vector2Subtract(
                              Boid2->getPosition(), Boid1->getPosition() ) ) );
        }

        AvgVelocity = Vector2Scale( AvgVelocity, 1.f / Count );
        AvgVelocity = Vector2Scale( AvgVelocity, 1.f / 8.f );

        AvgPosition = Vector2Scale( AvgPosition, 1.f / Count );
        AvgPosition = Vector2Subtract( AvgPosition, Boid1->getPosition() );
        AvgPosition = Vector2Scale( AvgPosition, 1.f / 100.f );

        Boid1->setVelocity( Vector2Add(
            Boid1->getVelocity(),
            Vector2Add(
                AvgVelocity,
                Vector2Add( AvgPosition,
                            Vector2Add( AvgAvoid, Boid1->boundPosition(
                                                      Bounds ) ) ) ) ) );

        if ( Vector2Length( Boid1->getVelocity() ) > SpeedLimit ) {
            Boid1->setVelocity( Vector2Scale(
                Vector2Normalize( Boid1->getVelocity() ), SpeedLimit ) );
        }
    }

    for ( auto& Boid1 : BoidList ) {
        Boid1->setPosition(
            Vector2Add( Boid1->getPosition(), Boid1->getVelocity() ) );
    }
}

void BoidManager::draw() const {
    for ( auto& BoidInstance : BoidList ) {
        BoidInstance->draw();
    }
}

Vector2 BoidManager::accumulatePosition() const {
    Vector2 Result( 0.f );
    for ( auto& BoidInstance : BoidList ) {
        Result = Vector2Add( Result, BoidInstance->getPosition() );
    }

    return Result;
}

Vector2 BoidManager::accumulateVelocity() const {
    Vector2 Result( 0.f );
    for ( auto& BoidInstance : BoidList ) {
        Result = Vector2Add( Result, BoidInstance->getVelocity() );
    }

    return Result;
}

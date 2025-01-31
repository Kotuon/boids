
#include "boid_manager.hpp"

#include <numeric>

#include "raymath.h"

#include <fmt/core.h>
#include "trace.hpp"

BoidManager::BoidManager( const Vector2 Bounds_ ) : Bounds( Bounds_ ) {
    for ( size_t i = 0; i < MAX; ++i ) {
        BoidList[i] = std::make_unique< Boid >(
            Vector2( static_cast< float >(
                         GetRandomValue( 0, static_cast< int >( Bounds.x ) ) ),
                     static_cast< float >( GetRandomValue(
                         0, static_cast< int >( Bounds.y ) ) ) ) );
    }
}

void BoidManager::update() {
    Vector2 AvgPosition = accumulatePosition();
    // AvgPosition =
    //     Vector2Scale( AvgPosition, 1.f / static_cast< float >( MAX ) );

    Vector2 AvgVelocity = accumulateVelocity();
    AvgVelocity =
        Vector2Scale( AvgVelocity, 1.f / static_cast< float >( MAX ) );

    int i = 0;
    for ( auto& Boid1 : BoidList ) {
        const Vector2 AvgPositionInstance = Vector2Scale(
            Vector2Subtract(
                Vector2Scale(
                    Vector2Subtract( AvgPosition, Boid1->getPosition() ),
                    1.f / ( MAX - 1.f ) ),
                Boid1->getPosition() ),
            1.f / 100.f );

        const Vector2 AvgVelocityInstance = Vector2Scale(
            Vector2Subtract( AvgVelocity, Boid1->getVelocity() ), 1.f / 8.f );

        Vector2 BufferDistance( 0.f );
        for ( auto& Boid2 : BoidList ) {
            if ( Boid1 == Boid2 ) continue;

            const Vector2 Between =
                Vector2Subtract( Boid1->getPosition(), Boid2->getPosition() );

            if ( Vector2Length( Between ) >= 100.f ) continue;

            BufferDistance = Vector2Subtract( BufferDistance, Between );
        }

        const Vector2 BoundedVelocity = Boid1->boundPosition( Bounds );

        Boid1->setVelocity( Vector2Add(
            Boid1->getVelocity(),
            Vector2Add( AvgPosition,
                        Vector2Add( AvgVelocityInstance,
                                    Vector2Add( BufferDistance,
                                                BoundedVelocity ) ) ) ) );

        Boid1->setPosition(
            Vector2Add( Boid1->getPosition(), Boid1->getVelocity() ) );

        Trace::message( fmt::format( "       Velocity({}): ({}, {})", i,
                                     Boid1->getVelocity().x,
                                     Boid1->getVelocity().y ) );
        Trace::message( fmt::format( "    AvgPosition({}): ({}, {})", i,
                                     AvgPosition.x, AvgPosition.y ) );
        Trace::message( fmt::format( "    AvgVelocity({}): ({}, {})", i,
                                     AvgVelocityInstance.x,
                                     AvgVelocityInstance.y ) );
        Trace::message( fmt::format( " BufferDistance({}): ({}, {})", i,
                                     BufferDistance.x, BufferDistance.y ) );
        Trace::message( fmt::format( "BoundedVelocity({}): ({}, {})", i,
                                     BoundedVelocity.x, BoundedVelocity.y ) );
        ++i;
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

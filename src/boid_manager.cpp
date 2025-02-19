
#include "boid_manager.hpp"

#include <numeric>

#include "raymath.h"

#include <fmt/core.h>
#include "trace.hpp"

BoidManager::BoidManager( const Vector2 Bounds_ ) : Bounds( Bounds_ ) {
    const float Scale = LocalSize / 13.f;

    LocalSize *= SimScale;
    SpeedLimit *= SimScale;

    QInstance = std::make_unique< Quadtree >();

    Stp = std::make_unique< StaticThreadPool >();
    ThreadCount = Stp->getThreadCount();

    Stp->initialize( &BoidManager::updateThreadWorker, this );

    for ( size_t i = 0; i < MAX; ++i ) {
        const Vector2 Pos( static_cast< float >( GetRandomValue(
                               0, static_cast< int >( Bounds.x ) ) ),
                           static_cast< float >( GetRandomValue(
                               0, static_cast< int >( Bounds.y ) ) ) );

        const Vector2 Vel( static_cast< float >( GetRandomValue( -5, 5 ) ),
                           static_cast< float >( GetRandomValue( -5, 5 ) ) );

        BoidList[i] = std::make_unique< Boid >( Pos, Vel, Scale, SimScale, i );
    }
}

void BoidManager::buildTree() {
    QInstance->clear();
    QInstance->initialize( BoidList );

    for ( auto& ThisBoid : BoidList ) {
        QInstance->insert( ThisBoid.get() );
    }
}

void BoidManager::updateTreeThread() {
    buildTree();

    UStatus = S_Velocity;
    Stp->runTask();

    UStatus = S_Position;
    Stp->runTask();
}

void BoidManager::updateTree() {
    buildTree();

    for ( size_t i = 0; i < MAX; ++i ) {
        auto& ThisBoid = BoidList[i];

        auto Targets =
            QInstance->query( ThisBoid->getPosition(), LocalSize / 2.f );

        BoidsUpdateValues Values;
        for ( auto* OtherBoid : Targets ) {
            Values.Count += 1;
            Values.AvgVelocity =
                Vector2Add( Values.AvgVelocity, OtherBoid->getVelocity() );
            Values.AvgPosition =
                Vector2Add( Values.AvgPosition, OtherBoid->getPosition() );

            const float Distance = Vector2Distance( ThisBoid->getPosition(),
                                                    OtherBoid->getPosition() );

            if ( Distance < ( LocalSize * 0.4f ) ) {
                Values.AvgAvoid = Vector2Subtract(
                    Values.AvgAvoid,
                    Vector2Scale( Vector2Normalize( Vector2Subtract(
                                      OtherBoid->getPosition(),
                                      ThisBoid->getPosition() ) ),
                                  10.f / Clamp( Distance, 0.001f, 100.f ) ) );
            }
        }

        // BoidsUpdateValues Values =
        //     QInstance->calculateVelocity( BoidList, Boid1, LocalSize );

        Values.AvgVelocity =
            Vector2Scale( Values.AvgVelocity, 1.f / ( Values.Count * 8.f ) );

        Values.AvgPosition =
            Vector2Scale( Values.AvgPosition, 1.f / Values.Count );
        Values.AvgPosition =
            Vector2Subtract( Values.AvgPosition, ThisBoid->getPosition() );
        Values.AvgPosition = Vector2Scale( Values.AvgPosition, 1.f / 100.f );

        Values.AvgVelocity = Vector2Scale( Values.AvgVelocity, SimScale );
        Values.AvgPosition = Vector2Scale( Values.AvgPosition, SimScale );
        Values.AvgAvoid = Vector2Scale( Values.AvgAvoid, SimScale );

        ThisBoid->setVelocity( Vector2Add(
            ThisBoid->getVelocity(),
            Vector2Add( Values.AvgVelocity,
                        Vector2Add( Values.AvgPosition,
                                    Vector2Add( Values.AvgAvoid,
                                                ThisBoid->boundPosition(
                                                    Bounds ) ) ) ) ) );

        if ( Vector2Length( ThisBoid->getVelocity() ) > SpeedLimit ) {
            ThisBoid->setVelocity( Vector2Scale(
                Vector2Normalize( ThisBoid->getVelocity() ), SpeedLimit ) );
        }
    }

    for ( size_t i = 0; i < MAX; ++i ) {
        auto& ThisBoid = BoidList[i];

        ThisBoid->setPosition(
            Vector2Add( ThisBoid->getPosition(), ThisBoid->getVelocity() ) );
    }
}

void BoidManager::updateThread() {

    // buildTree();

    UStatus = S_Velocity;
    Stp->runTask();

    UStatus = S_Position;
    Stp->runTask();
}

void BoidManager::updateTreeThreadWorker( const size_t ThreadId ) {
    const size_t Stride = MAX / ThreadCount;

    const size_t Start = ThreadId * Stride;

    size_t End = ( ThreadId + 1 ) * Stride;
    if ( ThreadId == ThreadCount - 1 ) End = MAX;

    if ( UStatus == S_Velocity ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Vector2 AvgPosition( 0.f ); // Cohesion
            Vector2 AvgVelocity( 0.f ); // Alignment
            Vector2 AvgAvoid( 0.f );    // Seperation
            size_t Count = 0;

            for ( auto& Boid2 : BoidList ) {
                const float Distance = Vector2Distance( Boid1->getPosition(),
                                                        Boid2->getPosition() );
                if ( Distance >= LocalSize ) continue;

                Count += 1;
                // Alignment
                AvgVelocity = Vector2Add( AvgVelocity, Boid2->getVelocity() );
                // Cohesion
                AvgPosition = Vector2Add( AvgPosition, Boid2->getPosition() );
                // Separation
                if ( Distance >= LocalSize * 0.4f ) continue;
                AvgAvoid = Vector2Subtract(
                    AvgAvoid,
                    Vector2Scale(
                        Vector2Normalize( Vector2Subtract(
                            Boid2->getPosition(), Boid1->getPosition() ) ),
                        10.f / Clamp( Distance, 0.001f, 100.f ) ) );
            }

            AvgVelocity = Vector2Scale( AvgVelocity, 1.f / ( Count * 8.f ) );

            AvgPosition = Vector2Scale( AvgPosition, 1.f / Count );
            AvgPosition = Vector2Subtract( AvgPosition, Boid1->getPosition() );
            AvgPosition = Vector2Scale( AvgPosition, 1.f / 100.f );

            AvgVelocity = Vector2Scale( AvgVelocity, SimScale );
            AvgPosition = Vector2Scale( AvgPosition, SimScale );
            AvgAvoid = Vector2Scale( AvgAvoid, SimScale );

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
    } else if ( UStatus == S_Position ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Boid1->setPosition(
                Vector2Add( Boid1->getPosition(), Boid1->getVelocity() ) );
        }
    }
}

void BoidManager::updateThreadWorker( const size_t ThreadId ) {
    const size_t Stride = MAX / ThreadCount;

    const size_t Start = ThreadId * Stride;

    size_t End = ( ThreadId + 1 ) * Stride;
    if ( ThreadId == ThreadCount - 1 ) End = MAX;

    if ( UStatus == S_Velocity ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Vector2 AvgPosition( 0.f ); // Cohesion
            Vector2 AvgVelocity( 0.f ); // Alignment
            Vector2 AvgAvoid( 0.f );    // Seperation
            size_t Count = 0;

            for ( auto& Boid2 : BoidList ) {
                const float Distance = Vector2Distance( Boid1->getPosition(),
                                                        Boid2->getPosition() );
                if ( Distance >= LocalSize ) continue;

                Count += 1;
                // Alignment
                AvgVelocity = Vector2Add( AvgVelocity, Boid2->getVelocity() );
                // Cohesion
                AvgPosition = Vector2Add( AvgPosition, Boid2->getPosition() );
                // Separation
                if ( Distance >= LocalSize * 0.4f ) continue;
                AvgAvoid = Vector2Subtract(
                    AvgAvoid,
                    Vector2Scale(
                        Vector2Normalize( Vector2Subtract(
                            Boid2->getPosition(), Boid1->getPosition() ) ),
                        10.f / Clamp( Distance, 0.001f, 100.f ) ) );
            }

            AvgVelocity = Vector2Scale( AvgVelocity, 1.f / ( Count * 8.f ) );

            AvgPosition = Vector2Scale( AvgPosition, 1.f / Count );
            AvgPosition = Vector2Subtract( AvgPosition, Boid1->getPosition() );
            AvgPosition = Vector2Scale( AvgPosition, 1.f / 100.f );

            AvgVelocity = Vector2Scale( AvgVelocity, SimScale );
            AvgPosition = Vector2Scale( AvgPosition, SimScale );
            AvgAvoid = Vector2Scale( AvgAvoid, SimScale );

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
    } else if ( UStatus == S_Position ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Boid1->setPosition(
                Vector2Add( Boid1->getPosition(), Boid1->getVelocity() ) );
        }
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
                AvgAvoid,
                Vector2Scale(
                    Vector2Normalize( Vector2Subtract( Boid2->getPosition(),
                                                       Boid1->getPosition() ) ),
                    10.f / Clamp( Distance, 0.001f, 100.f ) ) );
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

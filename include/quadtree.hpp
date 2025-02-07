
#ifndef QUADTREE_HPP
#define QUADTREE_HPP
#pragma once

#include <array>
#include <functional>
#include <vector>

#include "raylib.h"
#include "raymath.h"

#include "boid.hpp"
#include "memory_bank.hpp"

struct Quad {
    Quad() {}

    void init();

    template < std::size_t SIZE >
    Quad* createRoot( const std::array< BoidPtr, SIZE >& ParticleList ) {
        Vector2 Min = Vector2{ std::numeric_limits< float >::max(),
                               std::numeric_limits< float >::max() };
        Vector2 Max = Vector2{ std::numeric_limits< float >::min(),
                               std::numeric_limits< float >::min() };

        for ( auto& ThisParticle : ParticleList ) {
            Min.x = std::min( Min.x, ThisParticle->getPosition().x );
            Min.y = std::min( Min.y, ThisParticle->getPosition().y );
            Max.x = std::max( Max.x, ThisParticle->getPosition().x );
            Max.y = std::max( Max.y, ThisParticle->getPosition().y );
        }

        Center = Vector2Add( Min, Max );
        Center = Vector2Scale( Center, 0.5f );

        Size = std::max( Max.x - Min.x, Max.y - Min.y );

        return this;
    }

    unsigned findQuad( const Vector2& Pos );

    bool hasChildren() const;
    bool isEmpty() const;

    void subdivide( Quad* Parent, unsigned QuadrantId );

    float getMass( const std::vector< std::unique_ptr< Quad > >& Nodes );

    Vector2
    getCenterOfMass( const std::vector< std::unique_ptr< Quad > >& Nodes );

    void print( const unsigned Id ) const;

    unsigned Children = 0;
    unsigned Next = 0;

    Vector2 Center;
    Vector2 CenterOfMass;

    float Size = 0.f;
    float Mass = 0.f;
};

class Quadtree {
public:
    Quadtree();
    ~Quadtree();
    Quadtree( const Quadtree& );
    Quadtree( Quadtree&& );

    template < std::size_t SIZE >
    void initialize( const std::array< BoidPtr, SIZE >& ParticleList ) {
        Nodes.push_back( std::move( Mb->get() ) );

        auto& RootNode = Nodes.front();
        RootNode->createRoot( ParticleList );
    }

    void insert( Boid* ThisBody );

    unsigned subdivide( unsigned NodeId );

    void propagate();

    Vector2 accelerate( const BoidPtr& ThisBody );

    void clear();

    const std::vector< std::unique_ptr< Quad > >& getNodes();

    template < typename TCallback >
    void setFindForceCallback( TCallback Callback ) {
        FindForceCallback = Callback;
    }

private:
    std::vector< std::unique_ptr< Quad > > Nodes;
    std::vector< unsigned > Parents;

    std::function< const Vector2( const Vector2&, const Vector2& ) >
        FindForceCallback;

    std::unique_ptr< MemoryBank > Mb;

    float SquareTheta;
    float Theta;

    const unsigned Root = 0;
};

#endif

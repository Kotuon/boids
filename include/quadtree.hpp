
#ifndef QUADTREE_HPP
#define QUADTREE_HPP
#pragma once

#include <array>
#include <functional>
#include <vector>

#include <glm/glm.hpp>

#include "boid.hpp"
#include "memory_bank.hpp"

#include <fmt/core.h>
#include "trace.hpp"

struct Quad {
    Quad() {}

    void init();

    template < std::size_t SIZE >
    Quad* createRoot( const std::array< BoidPtr, SIZE >& ParticleList ) {
        glm::vec2 Min = glm::vec2{ std::numeric_limits< float >::max(),
                                   std::numeric_limits< float >::max() };
        glm::vec2 Max = glm::vec2{ std::numeric_limits< float >::min(),
                                   std::numeric_limits< float >::min() };

        for ( auto& ThisParticle : ParticleList ) {
            Min.x = std::min( Min.x, ThisParticle->getPosition().x );
            Min.y = std::min( Min.y, ThisParticle->getPosition().y );
            Max.x = std::max( Max.x, ThisParticle->getPosition().x );
            Max.y = std::max( Max.y, ThisParticle->getPosition().y );
        }

        Center = ( Min + Max ) * 0.5f;

        Size = std::max( Max.x - Min.x, Max.y - Min.y );
        HalfSize = Size * 0.5f;

        return this;
    }

    unsigned findQuad( const glm::vec2& Pos );

    bool intersects( const glm::vec2& Pos, const float HalfSize_ ) const;

    bool hasChildren() const;
    bool isEmpty() const;

    void subdivide( Quad* Parent, unsigned QuadrantId );

    void printSimple( const unsigned Id ) const;

    unsigned Children = 0;
    unsigned Next = 0;

    glm::vec2 Center{ 0.f };

    Boid* Body = nullptr;

    float Size = 0.f;
    float HalfSize = 0.f;

    int BodyId = -1;
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

    std::vector< Boid* > query( const glm::vec2& Pos, const float HalfSize );

    void insert( Boid* ThisBody );

    unsigned subdivide( unsigned NodeId );

    void clear();

    template < std::size_t SIZE >
    BoidsUpdateValues
    calculateVelocity( const std::array< BoidPtr, SIZE >& ParticleList,
                       const BoidPtr& ThisBody, const float LocalSize ) {
        BoidsUpdateValues Values;

        size_t NodeId = Root;

        while ( true ) {
            const auto& Node = Nodes[NodeId];

            const float Distance =
                glm::distance( ThisBody->getPosition, Node->Center );

            if ( !Node->hasChildren() ||
                 ( Node->Size ) < Distance * SquareTheta ) {
                // TODO: Compute velocity

                if ( !Node->isEmpty() ) {
                    auto& OtherBoid = ParticleList[Node->BodyId];

                    if ( Distance < LocalSize ) {
                        Values.Count += 1;
                        // Alignment
                        Values.AvgVelocity += OtherBoid->getVelocity();
                        // Cohesion
                        Values.AvgPosition += OtherBoid->getPosition();
                        // Separation
                        if ( Distance < ( LocalSize * 0.4f ) ) {
                            Values.AvgAvoid -=
                                glm::normalize( OtherBoid->getPosition() -
                                                ThisBody->getPosition() ) *
                                ( 10.f /
                                  glm::clamp( Distance, 0.001f, 100.f ) );
                        }
                    }
                }
                if ( Node->Next == 0 ) break;

                NodeId = Node->Next;
            } else {
                NodeId = Node->Children;
            }
        }

        return Values;
    }

    const std::vector< std::unique_ptr< Quad > >& getNodes();

private:
    void query( std::vector< Boid* >& Targets, const Quad* Node,
                const glm::vec2& Pos, const float HalfSize );

    std::vector< std::unique_ptr< Quad > > Nodes;
    std::vector< unsigned > Parents;

    std::unique_ptr< MemoryBank > Mb;

    float SquareTheta;
    float Theta;

    const unsigned Root = 0;
};

#endif

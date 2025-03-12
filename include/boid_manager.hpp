
#ifndef BOID_MANAGER_HPP
#define BOID_MANAGER_HPP
#pragma once

#include <array>

#include <glm/glm.hpp>

#include "boid.hpp"

#include "static_thread_pool.hpp"
#include "quadtree.hpp"

class Model;

enum UpdateStatus { S_Velocity, S_Position };

struct Container {
    Model* Modelnstance;
    glm::mat4 Matrix;
    float ModelRadius;
    float CollisionRadius = 6.f;
};

class BoidManager {
public:
    BoidManager( const glm::vec2 Bounds_ );
    void updateTreeThread();
    void updateTree();
    void updateThread();
    void update();
    void draw();

    const std::unique_ptr< Quadtree >& getQuadtree() const { return QInstance; }

    void displayEditorWindow();

private:
    void setupBounds( const glm::vec2 Bounds_ );

    void buildTree();

    void getVelocityInfo( Boid* ThisBoid, Boid* OtherBoid,
                          BoidsUpdateValues& Values, const float Distance ) {
        Values.Count += 1;
        // Alignment
        Values.AvgVelocity += OtherBoid->getVelocity();
        // Cohesion
        Values.AvgPosition += OtherBoid->getPosition();
        // Seperation
        if ( Distance >= LocalSize * 0.4f ) return;

        Values.AvgAvoid -=
            glm::normalize( OtherBoid->getPosition() -
                            ThisBoid->getPosition() ) *
            ( LocalSize * 0.001f /
              glm::clamp( Distance, LocalSize * 0.00001f, LocalSize ) );
    }

    void adjustVelocityInfo( Boid* ThisBoid, BoidsUpdateValues& Values ) {
        if ( Values.Count <= 0 ) return;

        Values.AvgVelocity *= 1.f / ( Values.Count * 8.f );

        Values.AvgPosition *= 1.f / Values.Count;
        Values.AvgPosition -= ThisBoid->getPosition();
        Values.AvgPosition *= 1.f / 100.f;

        Values.AvgVelocity *= SimScale;
        Values.AvgPosition *= SimScale;
        Values.AvgAvoid *= SimScale;
    }

    void updateBoidVelocity( Boid* ThisBoid, BoidsUpdateValues& Values ) {
        ThisBoid->setVelocity(
            ThisBoid->getVelocity() + Values.AvgVelocity + Values.AvgPosition +
            Values.AvgAvoid +
            ThisBoid->boundPosition( Bounds, BoundCorrection ) );

        if ( glm::length( ThisBoid->getVelocity() ) > SpeedLimit ) {
            ThisBoid->setVelocity( glm::normalize( ThisBoid->getVelocity() ) *
                                   SpeedLimit );
        }
    }

    void updateTreeThreadWorker( const size_t ThreadId );
    void updateThreadWorker( const size_t ThreadId );

    glm::vec2 Bounds;

    float Scale;
    
    float LocalSize = 4.8f;
    float SpeedLimit = 0.2f;

    float SimScale = 1.f;

    float BoundCorrection = 0.04f;

    static const size_t MAX = 10;
    std::array< BoidPtr, MAX > BoidList;

    std::array< float, MAX * 3 > Positions{ 0.f };
    std::array< float, MAX > Angles{ 0.f };

    std::unique_ptr< StaticThreadPool > Stp;
    std::unique_ptr< Quadtree > QInstance;

    size_t ThreadCount;

    UpdateStatus UStatus = S_Velocity;

    Container ContainerInstance;

    Model* BoidModel;
    unsigned Shader;
};

#endif

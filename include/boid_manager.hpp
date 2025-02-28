
#ifndef BOID_MANAGER_HPP
#define BOID_MANAGER_HPP
#pragma once

#include <array>

#include "boid.hpp"

#include "static_thread_pool.hpp"
#include "quadtree.hpp"

struct Vector2;

enum UpdateStatus { S_Velocity, S_Position };

class BoidManager {
public:
    BoidManager( const Vector2 Bounds_ );
    void updateTreeThread();
    void updateTree();
    void updateThread();
    void update();
    void draw() const;

    const std::unique_ptr< Quadtree >& getQuadtree() const { return QInstance; }

private:
    void buildTree();

    void updateTreeThreadWorker( const size_t ThreadId );
    void updateThreadWorker( const size_t ThreadId );

    Vector2 accumulatePosition() const;
    Vector2 accumulateVelocity() const;

    Vector2 Bounds;

    float LocalSize = 100.f;
    float SpeedLimit = 7.f;

    float SimScale = 0.25f;

    static const size_t MAX = 5000;
    std::array< BoidPtr, MAX > BoidList;

    std::unique_ptr< StaticThreadPool > Stp;
    std::unique_ptr< Quadtree > QInstance;

    size_t ThreadCount;

    UpdateStatus UStatus = S_Velocity;
};

#endif

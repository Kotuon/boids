
#include "boid_manager.hpp"

#include <numeric>

#include <glm/gtc/random.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <fmt/core.h>
#include "trace.hpp"

#include "model_manager.hpp"
#include "shader_manager.hpp"

#include "graphics.hpp"

BoidManager::BoidManager( const glm::vec2 Bounds_ ) : Bounds( Bounds_ ) {
    Scale = LocalSize / 13.f;

    LocalSize *= SimScale;
    SpeedLimit *= SimScale;

    Shader = ShaderManager::instance().getShader( "shaders/boid_vert.glsl",
                                                  "shaders/boid_frag.glsl" );

    BoidModel = ModelManager::instance().getModel( "models/boid3dfull.obj",
                                                   Shader, true );

    QInstance = std::make_unique< Quadtree >();

    Stp = std::make_unique< StaticThreadPool >();
    ThreadCount = Stp->getThreadCount();

    Stp->initialize( &BoidManager::updateThreadWorker, this );

    for ( size_t i = 0; i < MAX; ++i ) {
        const glm::vec2 Pos( static_cast< float >( glm::linearRand(
                                 static_cast< int >( Bounds.x ),
                                 static_cast< int >( Bounds.y ) ) ),
                             static_cast< float >( glm::linearRand(
                                 static_cast< int >( Bounds.x ),
                                 static_cast< int >( Bounds.y ) ) ) );

        // const glm::vec2 Pos( 0.f, 0.f );

        const glm::vec2 Vel( static_cast< float >( glm::linearRand( -5, 5 ) ),
                             static_cast< float >( glm::linearRand( -5, 5 ) ) );

        BoidList[i] = std::make_unique< Boid >( Pos, Vel, i );
    }

    setupBounds( Bounds );
}

void BoidManager::setupBounds( const glm::vec2 Bounds_ ) {
    unsigned ContainerShader = ShaderManager::instance().getShader(
        "shaders/base_vertex.glsl", "shaders/base_fragment.glsl" );

    ContainerInstance.Modelnstance = ModelManager::instance().getModel(
        "models/cube.obj", GL_TRIANGLES, ContainerShader, false );
    ContainerInstance.ModelRadius = abs( Bounds_.x ) + abs( Bounds_.y );

    ContainerInstance.Matrix =
        glm::scale( glm::mat4( 1.f ), { ContainerInstance.ModelRadius,
                                        ContainerInstance.ModelRadius,
                                        ContainerInstance.ModelRadius } );
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
        auto* ThisBoid = BoidList[i].get();

        auto Targets =
            QInstance->query( ThisBoid->getPosition(), LocalSize / 2.f );

        BoidsUpdateValues Values;
        for ( auto* OtherBoid : Targets ) {
            if ( OtherBoid == ThisBoid ) continue;

            const float Distance = glm::distance( ThisBoid->getPosition(),
                                                  OtherBoid->getPosition() );

            getVelocityInfo( ThisBoid, OtherBoid, Values, Distance );
        }

        adjustVelocityInfo( ThisBoid, Values );
        updateBoidVelocity( ThisBoid, Values );
    }

    for ( size_t i = 0; i < MAX; ++i ) {
        auto& ThisBoid = BoidList[i];

        ThisBoid->setPosition( ThisBoid->getPosition() +
                               ThisBoid->getVelocity() );
    }
}

void BoidManager::updateThread() {
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

            BoidsUpdateValues Values;
            for ( auto& Boid2 : BoidList ) {
                const float Distance =
                    glm::distance( Boid1->getPosition(), Boid2->getPosition() );
                if ( Distance >= LocalSize ) continue;

                getVelocityInfo( Boid1.get(), Boid2.get(), Values, Distance );
            }

            adjustVelocityInfo( Boid1.get(), Values );
            updateBoidVelocity( Boid1.get(), Values );
        }
    } else if ( UStatus == S_Position ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Boid1->setPosition( Boid1->getPosition() + Boid1->getVelocity() );
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

            BoidsUpdateValues Values;
            for ( auto& Boid2 : BoidList ) {
                const float Distance =
                    glm::distance( Boid1->getPosition(), Boid2->getPosition() );
                if ( Distance >= LocalSize ) continue;

                getVelocityInfo( Boid1.get(), Boid2.get(), Values, Distance );
            }

            adjustVelocityInfo( Boid1.get(), Values );

            updateBoidVelocity( Boid1.get(), Values );
        }
    } else if ( UStatus == S_Position ) {
        for ( size_t i = Start; i < End; ++i ) {
            auto& Boid1 = BoidList[i];

            Boid1->setPosition( Boid1->getPosition() + Boid1->getVelocity() );
        }
    }
}

void BoidManager::update() {
    for ( auto& Boid1 : BoidList ) {
        BoidsUpdateValues Values;
        for ( auto& Boid2 : BoidList ) {
            const float Distance =
                glm::distance( Boid1->getPosition(), Boid2->getPosition() );
            if ( Distance >= LocalSize ) continue;

            getVelocityInfo( Boid1.get(), Boid2.get(), Values, Distance );
        }

        adjustVelocityInfo( Boid1.get(), Values );
        updateBoidVelocity( Boid1.get(), Values );
    }

    for ( auto& Boid1 : BoidList ) {
        Boid1->setPosition( Boid1->getPosition() + Boid1->getVelocity() );
    }
}

void BoidManager::draw() {
    // Trace::message( "Drawing." );

    // for ( auto& BoidInstance : BoidList ) {
    //     BoidInstance->draw();
    // }

    size_t PosCounter = 0;
    size_t AngleCounter = 0;

    // const float HalfLocal = LocalSize * 0.5f;

    for ( size_t i = 0; i < MAX; ++i ) {
        auto& BoidInstance = BoidList[i];

        const glm::vec2& Pos = BoidInstance->getPosition();
        // const glm::vec2 Pos = { 0.f, 0.f };

        Positions[PosCounter++] = 0.f;
        Positions[PosCounter++] = Pos.y;
        Positions[PosCounter++] = Pos.x;

        const glm::vec2& Fwd = BoidInstance->getFwd();
        const glm::vec2 NormVel =
            glm::normalize( -BoidInstance->getVelocity() );

        const float Angle = glm::atan( NormVel.x * Fwd.y - NormVel.y * Fwd.x,
                                       NormVel.x * Fwd.x + Fwd.y * NormVel.y );

        Angles[AngleCounter++] = Angle;
    }

    glBindBuffer( GL_ARRAY_BUFFER, BoidModel->getMesh()->PositionVBO );
    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( float ) * 3 * MAX,
                     Positions.data() );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );

    glBindBuffer( GL_ARRAY_BUFFER, BoidModel->getMesh()->AngleVBO );
    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( float ) * MAX, Angles.data() );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );

    glUseProgram( BoidModel->getShader() );

    glUniformMatrix4fv(
        glGetUniformLocation( BoidModel->getShader(), "projection" ), 1,
        GL_FALSE, &Graphics::instance().getProjection()[0][0] );

    glUniform1f( glGetUniformLocation( BoidModel->getShader(), "scale" ),
                 Scale );

    glBindVertexArray( BoidModel->getMesh()->VAO );

    glDrawArraysInstanced( BoidModel->getRenderMethod(), 0,
                           BoidModel->getMesh()->NumVertices, MAX );

    glUseProgram( 0 );
    glBindVertexArray( 0 );

    Graphics::instance().drawNormal( ContainerInstance.Modelnstance,
                                     ContainerInstance.Matrix );
}

#include "imgui.h"
void BoidManager::displayEditorWindow() {
    ImGui::Begin( "BoidManager##1" );

    ImGui::Text( fmt::format( "Boid count: {}", MAX ).c_str() );

    if ( ImGui::Button( "Reset##1" ) ) {
        Scale = ( LocalSize / SimScale ) / 13.f;

        for ( size_t i = 0; i < MAX; ++i ) {
            const glm::vec2 Pos( static_cast< float >( glm::linearRand(
                                     static_cast< int >( Bounds.x ),
                                     static_cast< int >( Bounds.y ) ) ),
                                 static_cast< float >( glm::linearRand(
                                     static_cast< int >( Bounds.x ),
                                     static_cast< int >( Bounds.y ) ) ) );

            const glm::vec2 Vel(
                static_cast< float >( glm::linearRand( -5, 5 ) ),
                static_cast< float >( glm::linearRand( -5, 5 ) ) );

            BoidList[i]->init( Pos, Vel, i );
        }
    }

    if ( ImGui::SliderFloat( "SimScale", &SimScale, 0.01f, 5.f ) ) {
        LocalSize *= SimScale;
        SpeedLimit *= SimScale;
    }

    if ( ImGui::SliderFloat( "LocalSize", &LocalSize, 0.01f, 100.f ) ) {
    }

    if ( ImGui::SliderFloat( "BoundCorrection", &BoundCorrection, 0.f,
                             0.375f ) ) {
    }

    if ( ImGui::SliderFloat( "SpeedLimit", &SpeedLimit, 0.f, 0.35f ) ) {
    }

    ImGui::End();
}

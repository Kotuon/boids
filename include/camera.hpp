
#ifndef CAMERA_HPP
#define CAMERA_HPP
#pragma once

#include <glm/glm.hpp>

class Camera {
public:
    bool initialize( glm::vec3 Position_ );
    void update();
    void updateVectors();

    glm::mat4& getViewMatrix();

    float getOrbitRadius() const;
    void setOrbitRadius( float OrbitRadius_ );

    static Camera& instance();

    static void movement( glm::vec3 MovementInput );

private:
    Camera();

    inline static constexpr glm::vec3 GLOBAL_UP{ 0.f, 1.f, 0.f };

    glm::mat4 ViewMatrix;

    glm::vec3 Position;
    glm::vec3 Forward;
    glm::vec3 Right;
    glm::vec3 Up;

    glm::vec3 Rotation;

    float UniversalYawAngle;
    float UniversalPitchAngle;

    float OrbitRadius = 24.f;

    float Speed = 1.f;
};

#endif

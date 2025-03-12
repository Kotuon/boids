
// System includes
#include <glm/gtc/matrix_transform.hpp>

// Local includes
#include "camera.hpp"
#include "input.hpp"

Camera::Camera() {}

bool Camera::initialize( glm::vec3 Position_ ) {
    Position = Position_;
    Rotation = { -90.f, 0.f, 0.f };
    OrbitRadius = Position_.z;

    Input::instance().addWASDCallback( movement );

    updateVectors();

    return true;
}

void Camera::update() {
    Position = { glm::cos( glm::radians( UniversalYawAngle ) ) * OrbitRadius,
                 Position.y,
                 glm::sin( glm::radians( UniversalYawAngle ) ) * OrbitRadius };

    Rotation.x = UniversalYawAngle + 180.f;
}

void Camera::movement( glm::vec3 MovementInput ) {
    // Camera::instance().Rotation.y += -1.f * MovementInput.y * 65.f;
    // Camera::instance().Position += GLOBAL_UP * MovementInput.y * 30.f;
    Camera::instance().OrbitRadius -= MovementInput.y * 0.25f;

    Camera::instance().UniversalYawAngle += MovementInput.x * -1.f * 0.35f;
}

void Camera::updateVectors() {
    Forward = glm::normalize(
        glm::vec3( glm::cos( glm::radians( Rotation.x ) ) *
                       glm::cos( glm::radians( Rotation.y ) ),
                   glm::sin( glm::radians( Rotation.y ) ),
                   glm::sin( glm::radians( Rotation.x ) ) *
                       glm::cos( glm::radians( Rotation.y ) ) ) );

    Right = glm::normalize( glm::cross( Forward, GLOBAL_UP ) );
    Up = glm::normalize( glm::cross( Right, Forward ) );
}

glm::mat4& Camera::getViewMatrix() {
    updateVectors();

    glm::vec3 LookDirection = Position + Forward;
    ViewMatrix = glm::lookAt( Position, LookDirection, Up );

    return ViewMatrix;
}

float Camera::getOrbitRadius() const { return OrbitRadius; }

void Camera::setOrbitRadius( float OrbitRadius_ ) {
    OrbitRadius = OrbitRadius_;
}

Camera& Camera::instance() {
    static Camera CameraInstance;
    return CameraInstance;
}

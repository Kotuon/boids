#version 330 core

layout (location = 0) in vec3 vertexPos;
layout (location = 1) in vec3 vertexNormal;
layout (location = 2) in vec2 vertexTexCoord;
layout (location = 3) in vec3 instancePos;
layout (location = 4) in float instanceRotation;

uniform float scale;
uniform mat4 view;
uniform mat4 projection;

out vec3 fragmentPos;
out vec3 fragmentVertexNormal;

const float PI = 3.14;

mat4 translationMatrix(vec3 translation, float angle) {
    return mat4(
        vec4(scale,0.0, 0.0, 0.0),
        vec4(0.0, scale, 0.0, 0.0),
        vec4(0.0, 0.0, scale, 0.0),
        vec4(translation, 1.0)
    );
}

mat4 transformMatrix(vec3 translation, float angle) {
    mat4 scaleMat = mat4(
        vec4(scale,0.0, 0.0, 0.0),
        vec4(0.0, scale, 0.0, 0.0),
        vec4(0.0, 0.0, scale, 0.0),
        vec4(translation, 1.0)
    );

    mat4 translationMat = mat4(
        vec4(1.0,0.0, 0.0, 0.0),
        vec4(0.0, 1.0, 0.0, 0.0),
        vec4(0.0, 0.0, 1.0, 0.0),
        vec4(translation, 1.0)
    );

    mat4 rotMat = mat4(
        vec4(1.0, 0.0, 0.0, 0.0),
        vec4(0.0, cos(angle), sin(angle), 0.0),
        vec4(0.0, -sin(angle), cos(angle), 0.0),
        vec4(0.0, 0.0, 0.0, 1.0)
    );

    // mat4 rotMat = mat4(
    //     vec4(1.0, 0.0, 0.0, 0.0),
    //     vec4(0.0, cos(angle), sin(angle), 0.0),
    //     vec4(0.0, 0.0, -sin(angle), cos(angle)),
    //     vec4(0.0, 0.0, 0.0, 1.0)
    // );

    return scaleMat * rotMat ;
}

void main(){
    // mat4 model = translationMatrix(instancePos, instanceRotation);
    mat4 model = transformMatrix(instancePos, instanceRotation);
    fragmentPos = vec3(model * vec4(vertexPos, 1.0));
    fragmentVertexNormal = mat3(transpose(inverse(model))) * vertexNormal;
    
    gl_Position = projection * view * vec4(fragmentPos, 1.0);
}
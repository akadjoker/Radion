#ifndef RADION_CAMERA_BLOCK_H
#define RADION_CAMERA_BLOCK_H

#include <glm/glm.hpp>

namespace Radion
{

// Matches the Camera UBO layout every forward shader agrees on (binding 0 -
// see unlit.vert, water.vert). std140 needs no manual padding here: mat4 is
// already a 16-byte multiple, so the two vec4s follow it directly. A shader
// that only reads a prefix (unlit.vert never touches cameraPos) is fine -
// GL ignores the trailing bytes of a bound range past what the block declares.
struct CameraBlock
{
    Math::Mat4 viewProj;
    Math::Vec4 clipPlane; // (0,0,0,0) = no clip
    Math::Vec4 cameraPos; // xyz used; w unused
    Math::Mat4 view;
    // Appended, so every shader that already declares a prefix of this block
    // keeps working untouched. A pass that writes motion vectors and has no
    // block of its own reads the jitter-free pair from here instead of
    // claiming another binding - see tree.vert. Identity by default: a pass
    // that never fills them produces a zero motion vector, which costs a
    // pixel its history but can never reproject it somewhere wrong.
    Math::Mat4 viewProjectionNoJitter = Math::Mat4(1.0f);
    Math::Mat4 prevViewProjectionNoJitter = Math::Mat4(1.0f);
};

// Matches the TemporalCamera UBO (binding 7) every vertex shader that writes
// a motion vector declares. Both matrices exclude the temporal jitter: the
// jitter belongs to rasterisation, and a motion vector built from jittered
// clip space would carry that sub-pixel offset into the reprojection.
struct TemporalCameraBlock
{
    Math::Mat4 viewProjectionNoJitter;
    Math::Mat4 prevViewProjectionNoJitter;
};

} // namespace Radion

#endif // RADION_CAMERA_BLOCK_H

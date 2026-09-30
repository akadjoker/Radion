#ifndef RADION_CAMERA_BLOCK_H
#define RADION_CAMERA_BLOCK_H

#include "Math.h"

namespace Radion
{

// Camera UBO layout shared by all forward shaders (binding 0); std140 needs no padding here.
// A shader may declare only a prefix; GL ignores the trailing bytes.
struct CameraBlock
{
    Math::mat4 viewProj;
    Math::vec4 clipPlane; // (0,0,0,0) = no clip
    Math::vec4 cameraPos; // xyz used; w unused
    Math::mat4 view;
    // Appended so shaders declaring a prefix keep working. Jitter-free pair for motion vectors (see tree.vert).
    // Identity by default: an unfilled pass yields zero motion, never a wrong reprojection.
    Math::mat4 viewProjectionNoJitter = Math::mat4(1.0f);
    Math::mat4 prevViewProjectionNoJitter = Math::mat4(1.0f);
};

// TemporalCamera UBO (binding 7). Both matrices exclude jitter so motion vectors carry no sub-pixel offset.
struct TemporalCameraBlock
{
    Math::mat4 viewProjectionNoJitter;
    Math::mat4 prevViewProjectionNoJitter;
};

} // namespace Radion

#endif // RADION_CAMERA_BLOCK_H

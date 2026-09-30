#ifndef RADION_BLEND_GLTF_EXPORTER_H
#define RADION_BLEND_GLTF_EXPORTER_H

#include <string>
#include <vector>

namespace Radion
{

struct MeshData;

// Writes a mesh as a binary glTF 2.0 (.glb): one node, one mesh, one primitive
// per submesh, each with its own PBR material (base colour, roughness,
// metalness, two-sided, blending). Positions, normals and UVs are exported;
// texture images, tangents, vertex colours, skinning and animation are not -
// this is for static models built here (props, vehicles, projectiles).
//
// The engine's own axes already match glTF's (right-handed, +Y up, counter-
// clockwise front faces), so nothing is converted.
class GltfExporter
{
public:
    // The .glb bytes. False with `error` set when there is nothing to write or
    // the mesh is inconsistent (an index outside the vertex arrays).
    static bool build(const MeshData& mesh, const std::string& name, std::vector<unsigned char>& glb,
                      std::string* error = nullptr);

    // build() and write to `path`.
    static bool save(const MeshData& mesh, const std::string& path, std::string* error = nullptr);
};

} // namespace Radion

#endif // RADION_BLEND_GLTF_EXPORTER_H

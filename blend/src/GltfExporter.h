#ifndef RADION_BLEND_GLTF_EXPORTER_H
#define RADION_BLEND_GLTF_EXPORTER_H

#include <string>
#include <vector>

namespace Radion
{

struct MeshData;

// Writes a mesh as a binary glTF 2.0 (.glb): one primitive per submesh with its PBR material and embedded PNG/JPEG
// textures; no tangents, skinning or animation.
// The engine's axes already match glTF's (right-handed, +Y up, counter-clockwise front faces), so nothing is converted.
class GltfExporter
{
public:
    // False with `error` set when there is nothing to write or the mesh is inconsistent.
    static bool build(const MeshData& mesh, const std::string& name, std::vector<unsigned char>& glb,
                      std::string* error = nullptr, std::vector<std::string>* warnings = nullptr);

    static bool save(const MeshData& mesh, const std::string& path, std::string* error = nullptr,
                     std::vector<std::string>* warnings = nullptr);
};

} // namespace Radion

#endif // RADION_BLEND_GLTF_EXPORTER_H

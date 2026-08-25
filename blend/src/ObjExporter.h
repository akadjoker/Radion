#ifndef RADION_BLEND_OBJ_EXPORTER_H
#define RADION_BLEND_OBJ_EXPORTER_H

#include <string>

namespace Radion
{

struct MeshData;

class ObjExporter
{
public:
    static bool save(const MeshData& mesh, const std::string& path);
};

} // namespace Radion

#endif // RADION_BLEND_OBJ_EXPORTER_H

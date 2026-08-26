#ifndef RADION_MANUAL_MESH_H
#define RADION_MANUAL_MESH_H

#include "Component.h"
#include "Mesh.h"

#include <string>
#include <vector>

namespace Radion
{

class MeshRenderer;

 
class ManualMesh final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ManualMesh;

    void begin(const std::string& material = std::string());
    void beginSubMesh(const std::string& material = std::string());
    void position(const Math::Vec3& value);
    void normal(const Math::Vec3& value);
    void uv(const Math::Vec2& value);
    // setColor(), not color(u32) - position/normal/uv's "value" setter and
    // "by index" getter never collide, their parameter types differ, but a
    // colour is a u32 both ways: same signature as the getter below, which
    // C++ cannot resolve by return type alone.
    void setColor(u32 value);
    void index(u32 value);
    void triangle(u32 a, u32 b, u32 c);
    bool end();
    void clear();

    u32 vertexCount() const;
    Math::Vec3& position(u32 index);
    const Math::Vec3& position(u32 index) const;
    Math::Vec3& normal(u32 index);
    const Math::Vec3& normal(u32 index) const;
    Math::Vec2& uv(u32 index);
    const Math::Vec2& uv(u32 index) const;
    u32& color(u32 index);
    const u32& color(u32 index) const;

    MeshHandle mesh() const;

private:
    friend class GameObject;
    explicit ManualMesh();

    MeshData mData;
    MeshHandle mMesh;
    MeshRenderer* mRenderer = nullptr;
    u32 mCurrentMaterial = 0;
    bool mBuilding = false;
};

} // namespace Radion

#endif

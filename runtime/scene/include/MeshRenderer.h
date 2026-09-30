#ifndef RADION_MESH_RENDERER_H
#define RADION_MESH_RENDERER_H

#include "Component.h"
#include "Mesh.h"

#include <vector>

namespace Radion
{

class MeshRenderer final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::MeshRenderer;

    ~MeshRenderer() override;

    void setMesh(MeshHandle mesh);
    MeshHandle mesh() const;
    void setMaterialOverride(u32 slot, const Material& material);
    void clearMaterialOverrides();
    const Material* materialOverrides() const;
    u32 materialOverrideCount() const;

    // Off keeps a mirror out of the one global probe; shadows still see the object.
    void setVisibleInReflections(bool visible);
    bool visibleInReflections() const;

    bool submeshVisible(u32 submesh) const;
    void setSubmeshVisible(u32 submesh, bool visible);
    u32 submeshCount() const;
    const std::vector<u32>& hiddenSubmeshes() const;
    void setHiddenSubmeshes(std::vector<u32> hidden);
    void applyHiddenSubmeshes();

    // Generated renderers are rebuilt from their component and must not be serialized.
    bool generated() const;

private:
    friend class GameObject;
    friend class ManualMesh;
    friend class TiledTerrain;

    explicit MeshRenderer(MeshHandle mesh = MeshHandle());
    void setGeneratedBy(Component* component);

    MeshHandle mMesh;
    std::vector<Material> mMaterialOverrides;
    std::vector<u32> mHiddenSubmeshes;
    Component* mGeneratedBy = nullptr;
    bool mVisibleInReflections = true;
};

} // namespace Radion

#endif // RADION_MESH_RENDERER_H

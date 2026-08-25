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

    // Whether an environment probe's capture may contain this object. True
    // for everything by default. Turning it off is what keeps a mirror out
    // of the one global probe, which - being a single cubemap shared by the
    // whole scene - cannot leave an object out for one viewer and keep it in
    // for another the way a per-object probe can. Only probe captures honour
    // it; shadows still see the object, since a mirror casts a shadow like
    // anything else.
    void setVisibleInReflections(bool visible);
    bool visibleInReflections() const;

    bool submeshVisible(u32 submesh) const;
    void setSubmeshVisible(u32 submesh, bool visible);
    const std::vector<u32>& hiddenSubmeshes() const;
    void setHiddenSubmeshes(std::vector<u32> hidden);
    void applyHiddenSubmeshes();

private:
    friend class GameObject;

    explicit MeshRenderer(MeshHandle mesh = MeshHandle());

    MeshHandle mMesh;
    std::vector<Material> mMaterialOverrides;
    std::vector<u32> mHiddenSubmeshes;
    bool mVisibleInReflections = true;
};

} // namespace Radion

#endif // RADION_MESH_RENDERER_H

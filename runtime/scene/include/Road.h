#ifndef RADION_ROAD_H
#define RADION_ROAD_H

#include "Component.h"
#include "Material.h"
#include "Mesh.h"

#include <string>
#include <vector>

namespace Radion
{

class Scene;
class Terrain;

class Road final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Road;

    bool addPoint(GameObject* point, f32 width = 6.0f);
    bool insertPoint(usize index, GameObject* point, f32 width = 6.0f);
    bool removePoint(GameObject* point);
    void clearPoints();
    usize pointCount() const;
    GameObject* point(usize index) const;
    void setPointWidth(usize index, f32 width);
    f32 pointWidth(usize index) const;

    void setTerrain(Terrain* terrain);
    Terrain* terrain() const;
    void setSubdivisions(u32 subdivisions);
    void setTextureRepeat(f32 meters);
    void setSurfaceOffset(f32 offset);
    void setConformTerrain(bool enabled);
    void rebuild();

    bool saveSpline(const std::string& path) const;
    bool loadSpline(const std::string& path, Scene& scene);

    bool valid() const;
    MeshHandle mesh() const;
    u32 subdivisions() const;
    f32 textureRepeat() const;
    f32 surfaceOffset() const;
    bool conformTerrain() const;
    Material& material();
    const Material& material() const;

private:
    friend class GameObject;
    friend class Scene;

    struct Point
    {
        GameObject* object = nullptr;
        glm::vec3 previous = glm::vec3(0.0f);
        f32 width = 6.0f;
    };

    struct PathSample
    {
        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 tangent = glm::vec3(0.0f, 0.0f, -1.0f);
        f32 width = 6.0f;
        f32 distance = 0.0f;
    };

    Road();
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;
    PathSample evaluate(usize segment, f32 amount) const;

    std::vector<Point> mPoints;
    Terrain* mTerrain = nullptr;
    MeshHandle mMesh;
    Material mMaterial;
    u32 mSubdivisions = 12;
    f32 mTextureRepeat = 4.0f;
    f32 mSurfaceOffset = 0.06f;
    bool mConformTerrain = true;
    bool mDirty = true;
    u64 mTerrainRevision = 0;
};

} // namespace Radion

#endif // RADION_ROAD_H

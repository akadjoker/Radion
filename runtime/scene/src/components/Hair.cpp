#include "PCH.h"

#include "Hair.h"

#include "Animation.h"
#include "AssetManager.h"
#include "GameObject.h"
#include "MeshRenderer.h"

#include <algorithm>
#include <array>
#include <atomic>

namespace Radion
{

namespace
{
std::atomic<u64> gNextHairKey{1};

f32 vertexMask(const MeshData& mesh, u32 index)
{
    if (index >= mesh.colors.size())
        return 1.0f;
    return static_cast<f32>((mesh.colors[index] >> 24) & 0xFFu) / 255.0f;
}

struct WeightedJoint
{
    u32 joint = 0;
    f32 weight = 0.0f;
};

void rootSkin(const MeshData& mesh, const Math::uvec3& triangle, const Math::vec3& bary,
              Math::uvec4& joints, Math::vec4& weights)
{
    if (mesh.skin.size() != mesh.positions.size())
    {
        joints = Math::uvec4(0u);
        weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    std::vector<WeightedJoint> combined;
    combined.reserve(12);
    for (u32 corner = 0; corner < 3; ++corner)
    {
        const MeshSkinVertex& skin = mesh.skin[triangle[corner]];
        for (u32 influence = 0; influence < 4; ++influence)
        {
            const f32 value = skin.weights[influence] * bary[corner];
            if (value <= 0.0f)
                continue;
            auto found = std::find_if(combined.begin(), combined.end(), [&](const WeightedJoint& x) {
                return x.joint == skin.joints[influence];
            });
            if (found != combined.end())
                found->weight += value;
            else
                combined.push_back({skin.joints[influence], value});
        }
    }
    std::sort(combined.begin(), combined.end(), [](const WeightedJoint& a, const WeightedJoint& b) {
        return a.weight > b.weight;
    });

    joints = Math::uvec4(0u);
    weights = Math::vec4(0.0f);
    const usize count = std::min<usize>(4, combined.size());
    f32 total = 0.0f;
    for (usize i = 0; i < count; ++i)
    {
        joints[static_cast<int>(i)] = combined[i].joint;
        weights[static_cast<int>(i)] = combined[i].weight;
        total += combined[i].weight;
    }
    if (total > 0.000001f)
        weights /= total;
    else
        weights = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
}
} // namespace

Hair::Hair() : Component(Type), mKey(gNextHairKey.fetch_add(1))
{
}

void Hair::onDestroy()
{
    mRoots.clear();
    mColliders.clear();
}

f32 Hair::random()
{
    mRandomState = mRandomState * 1664525u + 1013904223u;
    return static_cast<f32>(mRandomState >> 8) / 16777216.0f;
}

bool Hair::generate()
{
    if (!owner())
        return false;
    MeshRenderer* renderer = owner()->getComponent<MeshRenderer>();
    if (!renderer || !renderer->mesh().valid() || mStrandCount == 0)
        return false;

    const MeshDesc& desc = Assets().meshDesc(renderer->mesh());
    MeshData mesh;
    if (desc.source == MeshSource::None || !Assets().buildMeshData(desc, mesh) ||
        mesh.positions.empty() || mesh.indices.empty() ||
        (!mesh.submeshes.empty() && mSubmesh >= mesh.submeshes.size()))
    {
        Log::error("Hair: scalp mesh must have a rebuildable MeshDesc and submesh %u", mSubmesh);
        return false;
    }
    if (mesh.normals.size() != mesh.positions.size())
        Assets().computeNormals(mesh);

    struct Candidate
    {
        Math::uvec3 triangle = Math::uvec3(0u);
        f32 cumulative = 0.0f;
    };
    std::vector<Candidate> candidates;
    // Procedural MeshData is allowed to omit submeshes; uploadMesh() treats
    // that as one implicit slot covering the complete index buffer, and root
    // generation must follow the same convention.
    const u32 indexOffset = mesh.submeshes.empty() ? 0u : mesh.submeshes[mSubmesh].indexOffset;
    const u32 indexCount = mesh.submeshes.empty() ? static_cast<u32>(mesh.indices.size())
                                                  : mesh.submeshes[mSubmesh].indexCount;
    const u32 end = Math::min(indexOffset + indexCount, static_cast<u32>(mesh.indices.size()));
    f32 total = 0.0f;
    for (u32 i = indexOffset; i + 2 < end; i += 3)
    {
        const Math::uvec3 tri(mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]);
        if (tri.x >= mesh.positions.size() || tri.y >= mesh.positions.size() ||
            tri.z >= mesh.positions.size())
            continue;
        const f32 mask = (vertexMask(mesh, tri.x) + vertexMask(mesh, tri.y) +
                          vertexMask(mesh, tri.z)) / 3.0f;
        if (mask <= 0.0001f)
            continue;
        const f32 averageNormalY = (mesh.normals[tri.x].y + mesh.normals[tri.y].y +
                                    mesh.normals[tri.z].y) / 3.0f;
        if (averageNormalY < mMinimumGrowthNormalY)
            continue;
        // Fade density just above the threshold instead of ending every
        // strand on one perfectly hard latitude. -1 keeps the old unfiltered
        // behaviour for authored scalp submeshes and alpha masks.
        f32 normalMask = 1.0f;
        if (mMinimumGrowthNormalY > -0.999f)
        {
            const f32 fadeEnd = Math::min(1.0f, mMinimumGrowthNormalY + 0.18f);
            normalMask = Math::smoothstep(mMinimumGrowthNormalY, fadeEnd, averageNormalY);
        }
        const Math::vec3 e0 = mesh.positions[tri.y] - mesh.positions[tri.x];
        const Math::vec3 e1 = mesh.positions[tri.z] - mesh.positions[tri.x];
        const f32 weight = Math::length(Math::cross(e0, e1)) * 0.5f * mask * normalMask;
        if (weight <= 0.0000001f)
            continue;
        total += weight;
        candidates.push_back({tri, total});
    }
    if (candidates.empty() || total <= 0.0f)
    {
        Log::error("Hair: scalp submesh %u has no eligible triangles", mSubmesh);
        return false;
    }

    mRandomState = mSeed;
    std::vector<HairRoot> roots;
    roots.reserve(mStrandCount);
    for (u32 strand = 0; strand < mStrandCount; ++strand)
    {
        const f32 pick = random() * total;
        const auto found = std::lower_bound(candidates.begin(), candidates.end(), pick,
                                            [](const Candidate& c, f32 value) {
                                                return c.cumulative < value;
                                            });
        const Math::uvec3 tri = (found != candidates.end() ? found : candidates.end() - 1)->triangle;

        const f32 r0 = std::sqrt(random());
        const f32 r1 = random();
        const Math::vec3 bary(1.0f - r0, r0 * (1.0f - r1), r0 * r1);
        const Math::vec3 position = mesh.positions[tri.x] * bary.x +
                                   mesh.positions[tri.y] * bary.y +
                                   mesh.positions[tri.z] * bary.z;
        Math::vec3 normal = mesh.normals[tri.x] * bary.x + mesh.normals[tri.y] * bary.y +
                           mesh.normals[tri.z] * bary.z;
        normal = Math::dot(normal, normal) > 0.000001f ? Math::normalize(normal)
                                                      : Math::vec3(0.0f, 1.0f, 0.0f);
        const f32 mask = Math::clamp(vertexMask(mesh, tri.x) * bary.x +
                                    vertexMask(mesh, tri.y) * bary.y +
                                    vertexMask(mesh, tri.z) * bary.z, 0.02f, 1.0f);

        HairRoot root;
        const f32 length = Math::mix(mMinimumLength, mMaximumLength, random()) * mask;
        root.positionLength = Math::vec4(position, length);
        root.normalWidth = Math::vec4(normal, mWidth * Math::mix(0.75f, 1.2f, random()));
        rootSkin(mesh, tri, bary, root.joints, root.weights);
        root.params = Math::vec4(random() * Math::two_pi<f32>(), random(), random() - 0.5f, 0.0f);
        roots.push_back(root);
    }

    mRoots.swap(roots);
    ++mRevision;
    mReset = true;
    Log::info("Hair: generated %u roots from scalp submesh %u", rootCount(), mSubmesh);
    return true;
}

void Hair::clear() { mRoots.clear(); ++mRevision; mReset = true; }
u32 Hair::rootCount() const { return static_cast<u32>(mRoots.size()); }

bool Hair::loadTexture(const std::string& filename)
{
    TextureHandle texture = Assets().loadTexture(filename, ColorSpace::sRGB, true, 5);
    if (!texture.valid())
        return false;
    mTexture = texture;
    mTextureFile = filename;
    return true;
}
const std::string& Hair::textureFile() const { return mTextureFile; }
TextureHandle Hair::texture() const { return mTexture; }

void Hair::setStrandCount(u32 v) { mStrandCount = Math::min(v, 250000u); }
void Hair::setSubmesh(u32 v) { mSubmesh = v; }
void Hair::setSeed(u32 v) { mSeed = v; }
void Hair::setMinimumGrowthNormalY(f32 v) { mMinimumGrowthNormalY = Math::clamp(v, -1.0f, 1.0f); }
void Hair::setSegments(u32 v) { mSegments = Math::clamp(v, 1u, kHairMaxSegments); mReset = true; }
void Hair::setFollowers(u32 v) { mFollowers = Math::clamp(v, 1u, kHairMaxFollowers); }
void Hair::setLengthRange(f32 a, f32 b)
{
    mMinimumLength = Math::max(0.001f, Math::min(a, b));
    mMaximumLength = Math::max(mMinimumLength, Math::max(a, b));
}
void Hair::setWidth(f32 v) { mWidth = Math::max(0.0001f, v); }
void Hair::setStiffness(f32 v) { mStiffness = Math::max(0.0f, v); }
void Hair::setDrag(f32 v) { mDrag = Math::clamp(v, 0.0f, 0.999f); }
void Hair::setGravity(f32 v) { mGravity = v; }
void Hair::setWind(f32 v) { mWind = v; }
void Hair::setDrawDistance(f32 v) { mDrawDistance = Math::max(0.0f, v); }
void Hair::setAlphaCut(f32 v) { mAlphaCut = Math::clamp(v, 0.0f, 1.0f); }
void Hair::setRoughness(f32 v) { mRoughness = Math::clamp(v, 0.04f, 1.0f); }
void Hair::setSpecularStrength(f32 v) { mSpecularStrength = Math::clamp(v, 0.0f, 2.0f); }
void Hair::setSpecularTint(f32 v) { mSpecularTint = Math::clamp(v, 0.0f, 1.0f); }
void Hair::setTransmission(f32 v) { mTransmission = Math::clamp(v, 0.0f, 2.0f); }
void Hair::setColor(const Math::vec3& v) { mColor = Math::max(v, Math::vec3(0.0f)); }
void Hair::setSoftFringe(bool v) { mSoftFringe = v; }

u32 Hair::strandCount() const { return mStrandCount; }
u32 Hair::submesh() const { return mSubmesh; }
u32 Hair::seed() const { return mSeed; }
f32 Hair::minimumGrowthNormalY() const { return mMinimumGrowthNormalY; }
u32 Hair::segments() const { return mSegments; }
u32 Hair::followers() const { return mFollowers; }
f32 Hair::minimumLength() const { return mMinimumLength; }
f32 Hair::maximumLength() const { return mMaximumLength; }
f32 Hair::width() const { return mWidth; }
f32 Hair::stiffness() const { return mStiffness; }
f32 Hair::drag() const { return mDrag; }
f32 Hair::gravity() const { return mGravity; }
f32 Hair::wind() const { return mWind; }
f32 Hair::drawDistance() const { return mDrawDistance; }
f32 Hair::alphaCut() const { return mAlphaCut; }
f32 Hair::roughness() const { return mRoughness; }
f32 Hair::specularStrength() const { return mSpecularStrength; }
f32 Hair::specularTint() const { return mSpecularTint; }
f32 Hair::transmission() const { return mTransmission; }
const Math::vec3& Hair::color() const { return mColor; }
bool Hair::softFringe() const { return mSoftFringe; }

void Hair::clearColliders() { mColliders.clear(); }
bool Hair::addSphereCollider(const Math::vec3& centre, f32 radius)
{
    if (mColliders.size() >= kHairMaxColliders || radius <= 0.0f)
        return false;
    HairCollider collider;
    collider.a = Math::vec4(centre, radius);
    collider.type = HairColliderType::Sphere;
    mColliders.push_back(collider);
    return true;
}
bool Hair::addCapsuleCollider(const Math::vec3& a, const Math::vec3& b, f32 radius)
{
    if (mColliders.size() >= kHairMaxColliders || radius <= 0.0f)
        return false;
    HairCollider collider;
    collider.a = Math::vec4(a, radius);
    collider.b = Math::vec4(b, 0.0f);
    collider.type = HairColliderType::Capsule;
    mColliders.push_back(collider);
    return true;
}
u32 Hair::colliderCount() const { return static_cast<u32>(mColliders.size()); }

void Hair::submit(f32 deltaTime)
{
    if (mRoots.empty() || !owner())
        return;
    Animator* animator = owner()->getComponent<Animator>();
    HairDrawCommand command;
    command.key = mKey;
    command.roots = mRoots.data();
    command.rootCount = rootCount();
    command.revision = mRevision;
    command.palette = animator && animator->active() ? &animator->palette() : nullptr;
    command.previousPalette = animator && animator->active() ? &animator->prevPalette() : nullptr;
    command.model = owner()->globalTransform();
    command.previousModel = owner()->previousGlobalTransform();
    command.colliders = mColliders.data();
    command.colliderCount = colliderCount();
    command.texture = mTexture;
    command.color = mColor;
    command.roughness = mRoughness;
    command.specularStrength = mSpecularStrength;
    command.specularTint = mSpecularTint;
    command.transmission = mTransmission;
    command.stiffness = mStiffness;
    command.drag = mDrag;
    command.gravity = mGravity;
    command.wind = mWind;
    command.drawDistance = mDrawDistance;
    command.alphaCut = mAlphaCut;
    command.deltaTime = deltaTime;
    command.segments = mSegments;
    command.followers = mFollowers;
    command.softFringe = mSoftFringe;
    command.reset = mReset;
    HairDraws().submit(command);
    mReset = false;
}

} // namespace Radion

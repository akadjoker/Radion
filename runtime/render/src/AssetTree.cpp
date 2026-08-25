#include "PCH.h"

#include "AssetManager.h"

// Port of proctree.js (Paul Brunt, 2012, BSD 3-clause). Four passes over one
// binary tree of branches: splitBranch() grows it, createForks() puts the
// vertex rings at every fork, createTwigs() adds the leaf cards at the tips,
// buildFaces() joins the rings and walks the bark uv along the branch.
//
// Two details of the original are kept because they are what the parameter
// values mean - changing either reshapes every tree built with the same
// numbers. The random source is |cos(a + a*a)| rather than a PRNG, and the
// split basis crosses the direction with a permutation of its own components
// instead of an orthonormal basis, which is where the irregularity comes from.

namespace Radion
{

namespace
{

// Rodrigues rotation of `vec` about `axis`.
glm::vec3 rotateAxisAngle(const glm::vec3& vec, const glm::vec3& axis, f32 angle)
{
    const f32 cosine = std::cos(angle);
    const f32 sine = std::sin(angle);
    return vec * cosine + glm::cross(axis, vec) * sine +
           axis * (glm::dot(axis, vec) * (1.0f - cosine));
}

// Scales only the component of `vector` that lies along `direction`; a scale of
// zero leaves the perpendicular part alone and drops the rest.
glm::vec3 scaleInDirection(const glm::vec3& vector, const glm::vec3& direction, f32 scale)
{
    const f32 magnitude = glm::dot(vector, direction);
    return vector + direction * (magnitude * scale - magnitude);
}

glm::vec3 normalizeSafe(const glm::vec3& v)
{
    const f32 length = glm::length(v);
    return length > 1e-20f ? v / length : glm::vec3(0.0f);
}

// Positive modulo. Ring indices step backwards past zero, and a negative index
// here would read outside the ring.
s32 wrap(s32 value, s32 count)
{
    const s32 remainder = value % count;
    return remainder < 0 ? remainder + count : remainder;
}

// The generator, not a PRNG - see the file header.
f32 treeRandom(f64 a)
{
    return static_cast<f32>(std::fabs(std::cos(a + a * a)));
}

struct Branch
{
    glm::vec3 head = glm::vec3(0.0f);
    Branch* parent = nullptr;
    Branch* child0 = nullptr;
    Branch* child1 = nullptr;
    f32 length = 1.0f;
    f32 radius = 0.0f;
    bool trunk = false;

    std::vector<u32> rootRing;
    std::vector<u32> ring0;
    std::vector<u32> ring1;
    std::vector<u32> ring2;
    bool ringed = false;
    s32 tip = -1;
};

class TreeBuilder
{
public:
    explicit TreeBuilder(const TreeParams& params) : mParams(params)
    {
    }

    // Fills `out` with the bark in submesh 0 and the twig cards in submesh 1.
    void build(MeshData& out, const AssetManager& assets)
    {
        Branch* root = addBranch(glm::vec3(0.0f, mParams.trunkLength, 0.0f), nullptr);
        root->length = mParams.initialBranchLength;

        splitBranch(root, static_cast<s32>(mParams.levels), static_cast<s32>(mParams.trunkSteps), 1,
                    1);
        createForks(root, mParams.maxRadius);
        createTwigs(root);
        buildFaces(root);
        averageNormals();

        const u32 barkVertices = static_cast<u32>(mPositions.size());
        const u32 barkIndices = static_cast<u32>(mIndices.size());

        out.positions = std::move(mPositions);
        out.normals = std::move(mNormals);
        out.uvs = std::move(mUVs);
        out.indices = std::move(mIndices);

        out.positions.insert(out.positions.end(), mTwigPositions.begin(), mTwigPositions.end());
        out.normals.insert(out.normals.end(), mTwigNormals.begin(), mTwigNormals.end());
        out.uvs.insert(out.uvs.end(), mTwigUVs.begin(), mTwigUVs.end());
        out.indices.reserve(out.indices.size() + mTwigIndices.size());
        for (u32 index : mTwigIndices)
            out.indices.push_back(index + barkVertices);

        SubMesh bark;
        bark.indexOffset = 0;
        bark.indexCount = barkIndices;
        bark.materialSlot = 0;

        SubMesh twigs;
        twigs.indexOffset = barkIndices;
        twigs.indexCount = static_cast<u32>(mTwigIndices.size());
        twigs.materialSlot = 1;

        out.submeshes = {bark, twigs};

        // Bark needs the tangents for its normal map; the twig cards are flat
        // and get theirs from the same pass for free.
        assets.recalculateTangents(out);
        assets.computeBounds(out);
        assets.computeSubMeshBounds(out);
    }

private:
    const TreeParams& mParams;

    // deque: the algorithm holds parent and child pointers while it grows, and
    // a vector would invalidate them on every push.
    std::deque<Branch> mBranches;

    std::vector<glm::vec3> mPositions;
    std::vector<glm::vec3> mNormals;
    std::vector<glm::vec2> mUVs;
    std::vector<u32> mIndices;

    std::vector<glm::vec3> mTwigPositions;
    std::vector<glm::vec3> mTwigNormals;
    std::vector<glm::vec2> mTwigUVs;
    std::vector<u32> mTwigIndices;

    Branch* addBranch(const glm::vec3& head, Branch* parent)
    {
        mBranches.emplace_back();
        Branch* branch = &mBranches.back();
        branch->head = head;
        branch->parent = parent;
        return branch;
    }

    void addTriangle(std::vector<u32>& indices, u32 a, u32 b, u32 c)
    {
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }

    // Reflects `vec` about `norm`, scaled by branchFactor: how the second child
    // is pushed away from the first.
    glm::vec3 mirrorBranch(const glm::vec3& vec, const glm::vec3& norm) const
    {
        const glm::vec3 projected = glm::cross(norm, glm::cross(vec, norm));
        return vec - projected * (mParams.branchFactor * glm::dot(projected, vec));
    }

    // ---------------------------------------------------------- branch tree

    void splitBranch(Branch* branch, s32 level, s32 steps, s32 l1, s32 l2)
    {
        const s32 levels = static_cast<s32>(mParams.levels);
        const s32 reverseLevel = levels - level;

        glm::vec3 origin(0.0f);
        if (branch->parent)
            origin = branch->parent->head;
        else
            branch->trunk = true;

        const glm::vec3 head = branch->head;
        const glm::vec3 direction = normalizeSafe(head - origin);

        const glm::vec3 normal =
            glm::cross(direction, glm::vec3(direction.z, direction.x, direction.y));
        const glm::vec3 tangent = glm::cross(direction, normal);

        const f32 r =
            treeRandom(static_cast<f64>(reverseLevel * 10 + l1 * 5 + l2 + mParams.seed));

        glm::vec3 adjust = normal * r + tangent * (1.0f - r);
        if (r > 0.5f)
            adjust = -adjust;

        const f32 clump = (mParams.clumpMax - mParams.clumpMin) * r + mParams.clumpMin;
        glm::vec3 first = normalizeSafe(adjust * (1.0f - clump) + direction * clump);
        glm::vec3 second = mirrorBranch(first, direction);

        if (r > 0.5f)
            std::swap(first, second);

        // While the trunk is still climbing its side branches spiral around it
        // instead of all leaving from the same side.
        if (steps > 0)
        {
            const f32 angle = static_cast<f32>(steps) / static_cast<f32>(mParams.trunkSteps) *
                              2.0f * glm::pi<f32>() * mParams.twistRate;
            second = normalizeSafe(glm::vec3(std::sin(angle), r, std::cos(angle)));
        }

        const f32 grow = static_cast<f32>(level * level) / static_cast<f32>(levels * levels) *
                         mParams.growAmount;
        const glm::vec3 bend(static_cast<f32>(reverseLevel) * mParams.sweepAmount,
                             static_cast<f32>(reverseLevel) * mParams.dropAmount + grow, 0.0f);
        first = normalizeSafe(first + bend);
        second = normalizeSafe(second + bend);

        Branch* child0 = addBranch(head + first * branch->length, branch);
        Branch* child1 = addBranch(head + second * branch->length, branch);
        branch->child0 = child0;
        branch->child1 = child1;

        const f32 childLength =
            std::pow(branch->length, mParams.lengthFalloffPower) * mParams.lengthFalloffFactor;
        child0->length = childLength;
        child1->length = childLength;

        if (level <= 0)
            return;

        if (steps > 0)
        {
            // The trunk keeps climbing rather than forking for real.
            const f32 kink = (r - 0.5f) * 2.0f * mParams.trunkKink;
            child0->head = branch->head + glm::vec3(kink, mParams.climbRate, kink);
            child0->trunk = true;
            child0->length = branch->length * mParams.taperRate;
            splitBranch(child0, level, steps - 1, l1 + 1, l2);
        }
        else
        {
            splitBranch(child0, level - 1, 0, l1 + 1, l2);
        }
        splitBranch(child1, level - 1, 0, l1, l2 + 1);
    }

    // ---------------------------------------------------------- fork rings

    void createForks(Branch* branch, f32 radius)
    {
        // Stored before the clamp on purpose: buildFaces() scales the bark uv
        // by the stored value, so clamping first would move the texture on
        // every short branch.
        branch->radius = radius;
        if (radius > branch->length)
            radius = branch->length;

        const s32 segments = static_cast<s32>(mParams.segments);
        const f32 segmentAngle = 2.0f * glm::pi<f32>() / static_cast<f32>(segments);

        if (!branch->parent)
        {
            // The base of the trunk: the one ring that is not born of a fork.
            const glm::vec3 axis(0.0f, 1.0f, 0.0f);
            for (s32 i = 0; i < segments; ++i)
            {
                const glm::vec3 vec = rotateAxisAngle(glm::vec3(-1.0f, 0.0f, 0.0f), axis,
                                                      -segmentAngle * static_cast<f32>(i));
                branch->rootRing.push_back(static_cast<u32>(mPositions.size()));
                mPositions.push_back(vec * (radius / mParams.radiusFalloffRate));
            }
        }

        if (!branch->child0)
        {
            // A tip: one vertex for the triangle fan to converge on.
            branch->tip = static_cast<s32>(mPositions.size());
            mPositions.push_back(branch->head);
            return;
        }

        const glm::vec3 axis =
            branch->parent ? normalizeSafe(branch->head - branch->parent->head)
                           : normalizeSafe(branch->head);

        const glm::vec3 axis1 = normalizeSafe(branch->head - branch->child0->head);
        const glm::vec3 axis2 = normalizeSafe(branch->head - branch->child1->head);
        const glm::vec3 tangent = normalizeSafe(glm::cross(axis1, axis2));

        const glm::vec3 axis3 = normalizeSafe(glm::cross(tangent, normalizeSafe(-axis1 - axis2)));
        const glm::vec3 centre =
            branch->head + glm::vec3(axis2.x, 0.0f, axis2.z) * (-mParams.maxRadius * 0.5f);

        branch->ringed = true;

        f32 scale = mParams.radiusFalloffRate;
        if (branch->child0->trunk || branch->trunk)
            scale = 1.0f / mParams.taperRate;

        // linch0 and linch1 belong to all three rings at once. That sharing is
        // what stitches the fork into one surface.
        const u32 linch0 = static_cast<u32>(mPositions.size());
        branch->ring0.push_back(linch0);
        branch->ring2.push_back(linch0);
        mPositions.push_back(centre + tangent * (radius * scale));

        u32 start = static_cast<u32>(mPositions.size()) - 1;
        const glm::vec3 d1 = rotateAxisAngle(tangent, axis2, glm::half_pi<f32>());
        const glm::vec3 d2 = normalizeSafe(glm::cross(tangent, axis));
        const f32 stretch = 1.0f / glm::dot(d1, d2);
        for (s32 i = 1; i < segments / 2; ++i)
        {
            glm::vec3 vec = rotateAxisAngle(tangent, axis2, segmentAngle * static_cast<f32>(i));
            branch->ring0.push_back(start + static_cast<u32>(i));
            branch->ring2.push_back(start + static_cast<u32>(i));
            vec = scaleInDirection(vec, d2, stretch);
            mPositions.push_back(centre + vec * (radius * scale));
        }

        const u32 linch1 = static_cast<u32>(mPositions.size());
        branch->ring0.push_back(linch1);
        branch->ring1.push_back(linch1);
        mPositions.push_back(centre + tangent * (-radius * scale));

        for (s32 i = segments / 2 + 1; i < segments; ++i)
        {
            const glm::vec3 vec =
                rotateAxisAngle(tangent, axis1, segmentAngle * static_cast<f32>(i));
            branch->ring0.push_back(static_cast<u32>(mPositions.size()));
            branch->ring1.push_back(static_cast<u32>(mPositions.size()));
            mPositions.push_back(centre + vec * (radius * scale));
        }

        branch->ring1.push_back(linch0);
        branch->ring2.push_back(linch1);

        start = static_cast<u32>(mPositions.size()) - 1;
        for (s32 i = 1; i < segments / 2; ++i)
        {
            const glm::vec3 vec =
                rotateAxisAngle(tangent, axis3, segmentAngle * static_cast<f32>(i));
            branch->ring1.push_back(start + static_cast<u32>(i));
            branch->ring2.push_back(start + static_cast<u32>(segments / 2 - i));
            mPositions.push_back(centre + vec * (radius * scale));
        }

        const f32 childRadius = radius * mParams.radiusFalloffRate;
        createForks(branch->child0,
                    branch->child0->trunk ? radius * mParams.taperRate : childRadius);
        createForks(branch->child1, childRadius);
    }

    // ---------------------------------------------------------- twig cards

    void createTwigs(Branch* branch)
    {
        if (branch->child0)
        {
            createTwigs(branch->child0);
            createTwigs(branch->child1);
            return;
        }

        const Branch* parent = branch->parent;
        if (!parent || !parent->child0 || !parent->child1)
            return;

        const glm::vec3 tangent = normalizeSafe(glm::cross(parent->child0->head - parent->head,
                                                           parent->child1->head - parent->head));
        const glm::vec3 binormal = normalizeSafe(branch->head - parent->head);

        const f32 scale = mParams.twigScale;
        const f32 length = branch->length;

        const glm::vec3 corners[4] = {
            branch->head + tangent * scale + binormal * (scale * 2.0f - length),
            branch->head - tangent * scale + binormal * (scale * 2.0f - length),
            branch->head - tangent * scale - binormal * length,
            branch->head + tangent * scale - binormal * length,
        };

        // Two coplanar quads with opposite winding. Not waste: back-face
        // culling keeps exactly one of them whichever side the camera is on,
        // so a leaf reads as double-sided without turning culling off.
        const u32 base = static_cast<u32>(mTwigPositions.size());
        for (u32 pass = 0; pass < 2; ++pass)
            for (u32 i = 0; i < 4; ++i)
                mTwigPositions.push_back(corners[i]);

        addTriangle(mTwigIndices, base + 0, base + 1, base + 2);
        addTriangle(mTwigIndices, base + 3, base + 0, base + 2);
        addTriangle(mTwigIndices, base + 6, base + 5, base + 4);
        addTriangle(mTwigIndices, base + 6, base + 4, base + 7);

        const glm::vec3 front = normalizeSafe(
            glm::cross(corners[0] - corners[2], corners[1] - corners[2]));
        for (u32 i = 0; i < 4; ++i)
            mTwigNormals.push_back(front);
        for (u32 i = 0; i < 4; ++i)
            mTwigNormals.push_back(-front);

        const glm::vec2 uvs[4] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};
        for (u32 pass = 0; pass < 2; ++pass)
            for (u32 i = 0; i < 4; ++i)
                mTwigUVs.push_back(uvs[i]);
    }

    // ---------------------------------------------------------- faces and uv

    void buildFaces(Branch* branch)
    {
        const s32 segments = static_cast<s32>(mParams.segments);

        if (!branch->parent)
        {
            mUVs.assign(mPositions.size(), glm::vec2(0.0f));

            // Which segment of the root ring lines up with segment 0 of the
            // first fork. Without it the bark texture twists at the base.
            const glm::vec3 tangent = normalizeSafe(glm::cross(branch->child0->head - branch->head,
                                                               branch->child1->head - branch->head));
            const glm::vec3 reference(-1.0f, 0.0f, 0.0f);
            f32 angle = std::acos(glm::clamp(glm::dot(tangent, reference), -1.0f, 1.0f));
            if (glm::dot(glm::cross(reference, tangent), normalizeSafe(branch->head)) > 0.0f)
                angle = 2.0f * glm::pi<f32>() - angle;

            const s32 offset = static_cast<s32>(
                std::lround(angle / (2.0 * glm::pi<f64>()) * static_cast<f64>(segments)));

            for (s32 i = 0; i < segments; ++i)
            {
                const u32 a = branch->ring0[static_cast<usize>(i)];
                const u32 b = branch->rootRing[static_cast<usize>(wrap(i + offset + 1, segments))];
                const u32 c = branch->rootRing[static_cast<usize>(wrap(i + offset, segments))];
                const u32 d = branch->ring0[static_cast<usize>(wrap(i + 1, segments))];

                addTriangle(mIndices, a, d, c);
                addTriangle(mIndices, d, b, c);

                const f32 u = std::fabs(static_cast<f32>(i) / static_cast<f32>(segments) - 0.5f) * 2.0f;
                mUVs[c] = glm::vec2(u, 0.0f);

                const f32 v = glm::length(mPositions[a] - mPositions[c]) * mParams.vMultiplier;
                mUVs[a] = glm::vec2(u, v);
                mUVs[branch->ring2[static_cast<usize>(i)]] = glm::vec2(u, v);
            }
        }

        if (branch->child0 && branch->child0->ringed)
        {
            const u32 offset0 = matchSegment(branch, branch->ring1, branch->child0);
            const u32 offset1 = matchSegment(branch, branch->ring2, branch->child1);

            // Bark uv density follows the branch's own radius, so a thin twig
            // does not get the trunk's texel size stretched over it.
            const f32 uvScale = mParams.maxRadius / branch->radius;

            for (s32 i = 0; i < segments; ++i)
                stitchChild(branch->child0, branch->ring1, static_cast<s32>(offset0), i, uvScale,
                            false);
            for (s32 i = 0; i < segments; ++i)
                stitchChild(branch->child1, branch->ring2, static_cast<s32>(offset1), i, uvScale,
                            true);

            buildFaces(branch->child0);
            buildFaces(branch->child1);
            return;
        }

        if (!branch->child0 || !branch->child1)
            return;

        // Tips: a triangle fan closing each child ring onto its tip vertex.
        closeTip(branch->child0, branch->ring1, 1.5f);
        closeTip(branch->child1, branch->ring2, 0.5f);
    }

    // Finds the rotation that best aligns the child's ring with the parent's,
    // by killing the component along the child's own direction and comparing
    // what is left - what points sideways.
    u32 matchSegment(const Branch* branch, const std::vector<u32>& ring, const Branch* child) const
    {
        const s32 segments = static_cast<s32>(mParams.segments);

        glm::vec3 side = normalizeSafe(mPositions[ring[0]] - branch->head);
        side = scaleInDirection(side, normalizeSafe(child->head - branch->head), 0.0f);

        s32 best = segments;
        f32 bestMatch = glm::dot(normalizeSafe(mPositions[child->ring0[0]] - child->head), side);
        for (s32 i = 1; i < segments; ++i)
        {
            const glm::vec3 d =
                normalizeSafe(mPositions[child->ring0[static_cast<usize>(i)]] - child->head);
            const f32 match = glm::dot(d, side);
            if (match > bestMatch)
            {
                bestMatch = match;
                best = segments - i;
            }
        }
        return static_cast<u32>(best);
    }

    // One segment of the quad strip joining a child's ring0 to the parent ring
    // it grows out of, and the uv that walks along with it.
    void stitchChild(Branch* child, const std::vector<u32>& ring, s32 offset, s32 i, f32 uvScale,
                     bool flipWinding)
    {
        const s32 segments = static_cast<s32>(mParams.segments);

        const u32 a = child->ring0[static_cast<usize>(i)];
        const u32 b = ring[static_cast<usize>(wrap(i + offset + 1, segments))];
        const u32 c = ring[static_cast<usize>(wrap(i + offset, segments))];
        const u32 d = child->ring0[static_cast<usize>(wrap(i + 1, segments))];

        if (flipWinding)
        {
            addTriangle(mIndices, a, b, c);
            addTriangle(mIndices, a, d, b);
        }
        else
        {
            addTriangle(mIndices, a, d, c);
            addTriangle(mIndices, d, b, c);
        }

        const f32 length = glm::length(mPositions[a] - mPositions[c]) * uvScale;
        const glm::vec2 previous = mUVs[ring[static_cast<usize>(wrap(i + offset - 1, segments))]];
        const glm::vec2 uv(previous.x, previous.y + length * mParams.vMultiplier);
        mUVs[a] = uv;
        mUVs[child->ring2[static_cast<usize>(i)]] = uv;
    }

    void closeTip(const Branch* child, const std::vector<u32>& ring, f32 uvCentre)
    {
        if (child->tip < 0)
            return;

        const s32 segments = static_cast<s32>(mParams.segments);
        const u32 tip = static_cast<u32>(child->tip);

        for (s32 i = 0; i < segments; ++i)
        {
            const u32 a = ring[static_cast<usize>(wrap(i + 1, segments))];
            const u32 b = ring[static_cast<usize>(i)];
            addTriangle(mIndices, tip, a, b);

            const f32 length = glm::length(mPositions[tip] - mPositions[b]);
            mUVs[tip] = glm::vec2(
                std::fabs(static_cast<f32>(i) / static_cast<f32>(segments) - uvCentre) * 2.0f,
                length * mParams.vMultiplier);
        }
    }

    // Smooth normals: the bark is one continuous surface, so every vertex
    // averages the faces around it.
    void averageNormals()
    {
        mNormals.assign(mPositions.size(), glm::vec3(0.0f));

        for (usize i = 0; i + 2 < mIndices.size(); i += 3)
        {
            const u32 a = mIndices[i];
            const u32 b = mIndices[i + 1];
            const u32 c = mIndices[i + 2];
            const glm::vec3 normal = normalizeSafe(
                glm::cross(mPositions[b] - mPositions[c], mPositions[b] - mPositions[a]));
            mNormals[a] += normal;
            mNormals[b] += normal;
            mNormals[c] += normal;
        }

        for (glm::vec3& normal : mNormals)
        {
            const f32 length = glm::length(normal);
            normal = length > 1e-8f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
};

// ------------------------------------------------------------------ presets
 
TreeParams addonDefaults()
{
    TreeParams params;
    params.segments = 6;
    params.levels = 5;
    params.lengthFalloffFactor = 0.85f;
    params.branchFactor = 2.45f;
    params.clumpMax = 0.45f;
    params.clumpMin = 0.40f;
    params.dropAmount = -0.10f;
    params.growAmount = 0.235f;
    params.sweepAmount = 0.01f;
    params.radiusFalloffRate = 0.73f;
    params.climbRate = 0.371f;
    params.trunkKink = 0.093f;
    params.twistRate = 3.02f;
    params.vMultiplier = 0.36f;
    params.seed = 10;
    return params;
}

const std::vector<TreePreset>& presets()
{
    static const std::vector<TreePreset> table = [] {
        std::vector<TreePreset> list;

        TreeParams oak = addonDefaults();
        oak.seed = 696;
        oak.trunkSteps = 9;
        oak.initialBranchLength = 0.79f;
        oak.maxRadius = 0.22f;
        oak.trunkLength = 3.21f;
        oak.twigScale = 0.55f;
        list.push_back({"Oak", oak, 0});

        TreeParams willow = addonDefaults();
        willow.seed = 264;
        willow.levels = 4;
        willow.trunkSteps = 9;
        willow.initialBranchLength = 0.59f;
        willow.lengthFalloffFactor = 0.57f;
        willow.dropAmount = 0.4f;
        willow.maxRadius = 0.05f;
        willow.radiusFalloffRate = 0.7f;
        willow.twistRate = 2.62f;
        willow.trunkLength = 1.79f;
        willow.twigScale = 0.4f;
        list.push_back({"Willow", willow, 1});

        TreeParams shrub = addonDefaults();
        shrub.seed = 264;
        shrub.levels = 3;
        shrub.trunkSteps = 0;
        shrub.initialBranchLength = 0.19f;
        shrub.dropAmount = 0.2f;
        shrub.maxRadius = 0.04f;
        shrub.radiusFalloffRate = 0.58f;
        shrub.twistRate = 5.0f;
        shrub.trunkLength = 0.38f;
        shrub.twigScale = 0.65f;
        list.push_back({"Shrub", shrub, 0});

        TreeParams ash = addonDefaults();
        ash.levels = 7;
        ash.initialBranchLength = 0.59f;
        ash.lengthFalloffFactor = 0.95f;
        ash.dropAmount = -0.3f;
        ash.growAmount = 0.03f;
        ash.maxRadius = 0.18f;
        ash.radiusFalloffRate = 0.61f;
        ash.climbRate = 0.2f;
        ash.trunkKink = -0.37f;
        ash.twistRate = 1.41f;
        ash.trunkLength = 7.05f;
        ash.twigScale = 0.65f;
        list.push_back({"Ash", ash, 1});

        TreeParams poplar = addonDefaults();
        poplar.trunkSteps = 30;
        poplar.dropAmount = 0.0f;
        poplar.maxRadius = 0.2f;
        poplar.climbRate = 0.3f;
        poplar.trunkKink = 0.01f;
        poplar.twistRate = -2.47f;
        poplar.trunkLength = 3.61f;
        poplar.twigScale = 1.0f;
        list.push_back({"Poplar", poplar, 2});

        TreeParams sequoia = addonDefaults();
        sequoia.seed = 264;
        sequoia.levels = 4;
        sequoia.trunkSteps = 12;
        sequoia.initialBranchLength = 2.19f;
        sequoia.dropAmount = 0.0f;
        sequoia.growAmount = -0.38f;
        sequoia.maxRadius = 0.6f;
        sequoia.radiusFalloffRate = 0.57f;
        sequoia.climbRate = 2.02f;
        sequoia.trunkKink = 0.0f;
        sequoia.trunkLength = 11.52f;
        sequoia.twigScale = 2.0f;
        list.push_back({"Sequoia", sequoia, 0});

        TreeParams beech = addonDefaults();
        beech.seed = 325;
        beech.levels = 7;
        beech.trunkSteps = 4;
        beech.initialBranchLength = 0.89f;
        beech.dropAmount = 0.151f;
        beech.growAmount = 0.033f;
        beech.maxRadius = 0.21f;
        beech.climbRate = 0.37f;
        beech.twistRate = 2.21f;
        beech.trunkLength = 4.83f;
        beech.twigScale = 0.7f;
        list.push_back({"Beech", beech, 0});

        return list;
    }();

    return table;
}

 
f32 randomRange(u32& state, f32 low, f32 high)
{
    state = state * 1664525u + 1013904223u;
    return low + static_cast<f32>(state >> 8) / 16777216.0f * (high - low);
}

s32 randomRange(u32& state, s32 low, s32 high)
{
    state = state * 1664525u + 1013904223u;
    return low + static_cast<s32>((state >> 8) % static_cast<u32>(high - low + 1));
}

} // namespace

u32 AssetManager::treePresetCount()
{
    return static_cast<u32>(presets().size());
}

const TreePreset& AssetManager::treePreset(u32 index)
{
    const std::vector<TreePreset>& table = presets();
    return table[index < table.size() ? index : table.size() - 1];
}

TreeParams AssetManager::randomTreeParams(u32& state)
{
    TreeParams params;
    params.seed = randomRange(state, 1, 99999);

    params.levels = static_cast<u32>(randomRange(state, 3, 5));
    params.trunkSteps = static_cast<u32>(randomRange(state, 1, 5));
    params.segments = static_cast<u32>(randomRange(state, 3, 5)) * 2;

    params.trunkLength = randomRange(state, 1.0f, 3.4f);
    params.initialBranchLength = randomRange(state, 0.55f, 1.0f);

 
    params.lengthFalloffFactor = randomRange(state, 0.78f, 0.96f);
    params.lengthFalloffPower = randomRange(state, 0.8f, 1.2f);

    params.branchFactor = randomRange(state, 2.0f, 3.5f);

    // clumpMin has to stay under clumpMax, or the clump goes negative and the
    // branches point backwards.
    params.clumpMax = randomRange(state, 0.55f, 0.95f);
    params.clumpMin = randomRange(state, 0.2f, params.clumpMax - 0.1f);

    params.dropAmount = randomRange(state, -0.25f, 0.40f);
    params.growAmount = randomRange(state, -0.20f, 0.35f);
    params.sweepAmount = randomRange(state, -0.08f, 0.08f);

    params.maxRadius = randomRange(state, 0.07f, 0.24f);
    params.radiusFalloffRate = randomRange(state, 0.5f, 0.75f);
    params.taperRate = randomRange(state, 0.86f, 0.99f);

    params.climbRate = randomRange(state, 0.35f, 1.6f);
    params.trunkKink = randomRange(state, 0.0f, 0.18f);
    params.twistRate = randomRange(state, 0.0f, 20.0f);

    params.vMultiplier = randomRange(state, 0.15f, 1.0f);
    params.twigScale = randomRange(state, 0.22f, 0.6f);
    return params;
}

void AssetManager::buildTree(MeshData& out, const TreeParams& params) const
{
    out.clear();

    TreeParams sane = params;
    // createForks() walks segments/2 and assumes it divides exactly; an odd
    // count leaves holes in the mesh.
    sane.segments = sane.segments < 4 ? 4 : (sane.segments & ~1u);
    sane.levels = sane.levels < 1 ? 1 : sane.levels;

    TreeBuilder builder(sane);
    builder.build(out, *this);
}

MeshHandle AssetManager::createTree(const TreeParams& params)
{
    MeshData data;
    buildTree(data, params);

    // Bark is plain opaque geometry. The twig cards are alpha-tested and lit
    // from both sides - the cards are flat, so a back-facing one would
    // otherwise go black.
    Material bark;
    bark.cull = CullMode::Back;

    Material twigs;
    twigs.flags |= MaterialAlphaTest | MaterialTwoSided;
    twigs.cull = CullMode::Back;
    twigs.params.surface.z = 0.4f;

    data.materials = {bark, twigs};
    return createMesh(data);
}

} // namespace Radion

#include "PCH.h"

#include "VolumeGrid.h"
#include "VolumeMesher.h"

 

namespace Radion::Volume
{

namespace
{
constexpr u32 kGridVersion = 1;
constexpr u32 kGridFlags = 0;
constexpr char kGridMagic[] = "RVOL";
}

GridSource::GridSource(glm::uvec3 dimensions, glm::vec3 origin, f32 cellSize, f32 initialDensity)
    : m_dimensions(dimensions), m_origin(origin), m_cellSize(cellSize)
{
    constexpr u64 maxVoxels = 256ull * 1024ull * 1024ull;
    const u64 count = u64(dimensions.x) * dimensions.y * dimensions.z;
    if (cellSize > 0.0f && std::isfinite(cellSize) && count > 0 && count <= maxVoxels)
        m_values.assign(static_cast<usize>(count), initialDensity);
    else
        m_dimensions = glm::uvec3(0);
}

usize GridSource::index(u32 x, u32 y, u32 z) const
{
    return (static_cast<usize>(z) * m_dimensions.y + y) * m_dimensions.x + x;
}

bool GridSource::contains(u32 x, u32 y, u32 z) const
{
    return x < m_dimensions.x && y < m_dimensions.y && z < m_dimensions.z;
}

AABB GridSource::bounds() const
{
    AABB result;
    if (!valid()) return result;
    result.min = m_origin;
    result.max = m_origin + glm::vec3(m_dimensions - glm::uvec3(1)) * m_cellSize;
    return result;
}

glm::vec3 GridSource::voxelPosition(u32 x, u32 y, u32 z) const
{
    return m_origin + glm::vec3(x, y, z) * m_cellSize;
}

bool GridSource::worldToVoxel(const glm::vec3& position, glm::uvec3& voxel) const
{
    if (!valid() || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) return false;
    const glm::vec3 coordinate = (position - m_origin) / m_cellSize;
    const glm::vec3 last = glm::vec3(m_dimensions - glm::uvec3(1));
    if (glm::any(glm::lessThan(coordinate, glm::vec3(0.0f))) || glm::any(glm::greaterThan(coordinate, last))) return false;
    voxel = glm::uvec3(glm::floor(coordinate));
    return true;
}

f32 GridSource::voxel(u32 x, u32 y, u32 z) const
{
    if (!contains(x, y, z)) return -std::numeric_limits<f32>::infinity();
    return m_values[index(x, y, z)];
}

bool GridSource::setVoxel(u32 x, u32 y, u32 z, f32 density)
{
    if (!contains(x, y, z) || !std::isfinite(density)) return false;
    m_values[index(x, y, z)] = density;
    return true;
}

f32 GridSource::sampleClamped(const glm::ivec3& point) const
{
    const glm::ivec3 last = glm::ivec3(m_dimensions) - glm::ivec3(1);
    const glm::ivec3 clamped = glm::clamp(point, glm::ivec3(0), last);
    return voxel(static_cast<u32>(clamped.x), static_cast<u32>(clamped.y), static_cast<u32>(clamped.z));
}

f32 GridSource::sampleDensity(const glm::vec3& position) const
{
    if (!valid()) return -std::numeric_limits<f32>::infinity();
    const glm::vec3 coordinate = (position - m_origin) / m_cellSize;
    const glm::ivec3 base = glm::floor(coordinate);
    const glm::vec3 fraction = coordinate - glm::vec3(base);
    const f32 c000 = sampleClamped(base + glm::ivec3(0,0,0));
    const f32 c100 = sampleClamped(base + glm::ivec3(1,0,0));
    const f32 c010 = sampleClamped(base + glm::ivec3(0,1,0));
    const f32 c110 = sampleClamped(base + glm::ivec3(1,1,0));
    const f32 c001 = sampleClamped(base + glm::ivec3(0,0,1));
    const f32 c101 = sampleClamped(base + glm::ivec3(1,0,1));
    const f32 c011 = sampleClamped(base + glm::ivec3(0,1,1));
    const f32 c111 = sampleClamped(base + glm::ivec3(1,1,1));
    const f32 x00 = glm::mix(c000, c100, fraction.x), x10 = glm::mix(c010, c110, fraction.x);
    const f32 x01 = glm::mix(c001, c101, fraction.x), x11 = glm::mix(c011, c111, fraction.x);
    return glm::mix(glm::mix(x00, x10, fraction.y), glm::mix(x01, x11, fraction.y), fraction.z);
}

Sample GridSource::sample(const glm::vec3& position) const
{
    const f32 h = m_cellSize * 0.5f;
    const f32 dx = sampleDensity(position + glm::vec3(h,0,0)) - sampleDensity(position - glm::vec3(h,0,0));
    const f32 dy = sampleDensity(position + glm::vec3(0,h,0)) - sampleDensity(position - glm::vec3(0,h,0));
    const f32 dz = sampleDensity(position + glm::vec3(0,0,h)) - sampleDensity(position - glm::vec3(0,0,h));
    return {glm::vec3(dx, dy, dz) / m_cellSize, sampleDensity(position)};
}

void GridSource::fill(const Source& source)
{
    if (!valid()) return;
    for (u32 z = 0; z < m_dimensions.z; ++z) for (u32 y = 0; y < m_dimensions.y; ++y) for (u32 x = 0; x < m_dimensions.x; ++x)
        m_values[index(x,y,z)] = source.sampleDensity(voxelPosition(x, y, z));
}

AABB GridSource::apply(VolumeOperation operation, const Source& brush, const AABB& affected)
{
    AABB changed;
    if (!valid() || affected.empty()) return changed;
    const glm::vec3 minCoord = glm::floor((affected.min - m_origin) / m_cellSize);
    const glm::vec3 maxCoord = glm::ceil((affected.max - m_origin) / m_cellSize);
    for (s32 z = std::max(0, static_cast<s32>(minCoord.z)); z <= std::min(static_cast<s32>(m_dimensions.z) - 1, static_cast<s32>(maxCoord.z)); ++z)
        for (s32 y = std::max(0, static_cast<s32>(minCoord.y)); y <= std::min(static_cast<s32>(m_dimensions.y) - 1, static_cast<s32>(maxCoord.y)); ++y)
            for (s32 x = std::max(0, static_cast<s32>(minCoord.x)); x <= std::min(static_cast<s32>(m_dimensions.x) - 1, static_cast<s32>(maxCoord.x)); ++x)
            {
                const glm::vec3 p = voxelPosition(static_cast<u32>(x), static_cast<u32>(y), static_cast<u32>(z));
                const f32 oldValue = voxel(x,y,z), brushValue = brush.sampleDensity(p);
                f32 value = oldValue;
                if (operation == VolumeOperation::Union) value = std::max(oldValue, brushValue);
                else if (operation == VolumeOperation::Difference) value = std::min(oldValue, -brushValue);
                else value = std::min(oldValue, brushValue);
                if (value != oldValue) { m_values[index(x,y,z)] = value; changed.expand(p); }
            }
    return changed;
}

bool GridSource::buildMesh(MeshData& output, MeshingStats* stats) const
{
    if (!valid()) return false;
    MeshingSettings settings;
    settings.bounds = bounds();
    settings.voxelSize = m_cellSize;
    return Volume::buildMesh(*this, settings, output, stats);
}

bool GridSource::save(ByteArray& output) const
{
    if (!valid() || m_values.size() != static_cast<usize>(m_dimensions.x) * m_dimensions.y * m_dimensions.z)
        return false;
    const usize start = output.tell();
    if (!output.writeBytes(kGridMagic, 4) || !output.writeU32(kGridVersion) ||
        !output.writeU32(m_dimensions.x) || !output.writeU32(m_dimensions.y) || !output.writeU32(m_dimensions.z) ||
        !output.writeF32(m_origin.x) || !output.writeF32(m_origin.y) || !output.writeF32(m_origin.z) ||
        !output.writeF32(m_cellSize) || !output.writeU32(kGridFlags) ||
        !output.writeU64(static_cast<u64>(m_values.size())))
    {
        output.seek(static_cast<long long>(start));
        return false;
    }
    for (f32 value : m_values)
        if (!output.writeF32(value))
        {
            output.seek(static_cast<long long>(start));
            return false;
        }
    return true;
}

bool GridSource::load(ByteArray& input, GridSource& output)
{
    const usize start = input.tell();
    char magic[4]{};
    if (!input.canRead(4)) return false;
    input.readBytes(magic, 4);
    if (std::memcmp(magic, kGridMagic, 4) != 0 || !input.canRead(4 * 4 + 3 * sizeof(f32) + sizeof(u32) + sizeof(u64)))
    {
        input.seek(static_cast<long long>(start));
        return false;
    }
    const u32 version = input.readU32();
    const glm::uvec3 dimensions(input.readU32(), input.readU32(), input.readU32());
    const glm::vec3 origin(input.readF32(), input.readF32(), input.readF32());
    const f32 cellSize = input.readF32();
    const u32 flags = input.readU32();
    const u64 count = input.readU64();
    const u64 expected = u64(dimensions.x) * dimensions.y * dimensions.z;
    constexpr u64 maxVoxels = 256ull * 1024ull * 1024ull;
    if (version != kGridVersion || flags != kGridFlags || dimensions.x == 0 || dimensions.y == 0 || dimensions.z == 0 ||
        expected != count || expected > maxVoxels || !std::isfinite(cellSize) || cellSize <= 0.0f ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
        count > static_cast<u64>(std::numeric_limits<usize>::max()) ||
        !input.canRead(static_cast<usize>(count) * sizeof(f32)))
    {
        input.seek(static_cast<long long>(start));
        return false;
    }
    GridSource loaded(dimensions, origin, cellSize);
    for (u64 i = 0; i < count; ++i)
    {
        const f32 value = input.readF32();
        if (!std::isfinite(value))
        {
            input.seek(static_cast<long long>(start));
            return false;
        }
        loaded.m_values[static_cast<usize>(i)] = value;
    }
    output = std::move(loaded);
    return true;
}

} // namespace Radion::Volume

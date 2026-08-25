#ifndef RADION_FORMAT_H
#define RADION_FORMAT_H

#include "Types.h"

#include <string>

namespace Radion
{
class ByteArray;
}

namespace Radion::AssetFormat
{

constexpr u32 MeshMagic = 0x48534D52;      // RMSH
constexpr u32 StaticMeshMagic = 0x5253544D;  // RSTM
constexpr u32 SkeletonMagic = 0x4C4B5352;  // RSKL
constexpr u32 AnimationMagic = 0x4D4E4152; // RANM
constexpr u32 NavMeshMagic = 0x56414E52;   // RNAV
constexpr u16 VersionMajor = 1;
constexpr u16 VersionMinor = 0;
constexpr u32 Version = (static_cast<u32>(VersionMajor) << 16) | VersionMinor;

enum HeaderFlags : u32
{
    LittleEndian = 1 << 0
};

enum Chunk : u32
{
    MaterialSlots = 0x5354414D, // MATS
    VertexLayout = 0x59414C56,  // VLAY
    VertexBuffer = 0x46554256,  // VBUF
    IndexBuffer = 0x46554249,   // IBUF
    SubMeshes = 0x4D425553,     // SUBM
    Skin = 0x4E494B53,          // SKIN
    LightmapUV = 0x56554D4C,    // LMUV
    SkeletonLink = 0x4C4B5352,  // RSKL
    Bounds = 0x444E4F42,        // BOND
    Bones = 0x454E4F42,         // BONE
    Clip = 0x50494C43,          // CLIP
    Track = 0x4B415254,         // TRAK
    Keys = 0x5359454B,          // KEYS
    NavData = 0x41544144,       // DATA - raw dtNavMesh tile bytes
    NavDebugTriangles = 0x52544144 // DATR - debug-draw triangles
};

enum class VertexSemantic : u8
{
    Position,
    Normal,
    Tangent,
    TexCoord,
    Color,
    Joints,
    Weights
};

enum class VertexType : u8
{
    Float1,
    Float2,
    Float3,
    Float4,
    UByte4,
    UByte4N,
    UShort4
};

enum MeshFlags : u32
{
    MeshSkinned = 1 << 0
};

enum StaticMeshFlags : u32
{
    StaticMeshIndex16 = 1 << 1,
    StaticMeshHasColors = 1 << 2,
    StaticMeshCompressed = 1 << 3
};

// VertexAttribFlags (below) packed into the same header `flags` word the
// static mesh already has, starting past the bits above - one field to
// read, not a new one to add to an already-fixed header shape.
constexpr u32 StaticMeshAttribShift = 4;

enum SubMeshFlags : u32
{
    SubMeshTwoSided = 1 << 0
};

// Which fields a vertex buffer chunk actually carries - position is never
// optional, everything else is. Both VertexBuffer (RMSH) and the static
// mesh payload (RSTM) use this to compute their own per-vertex stride
// instead of assuming a fixed one; a mesh missing an attribute pays
// nothing for it on disk.
enum VertexAttribFlags : u32
{
    HasNormal = 1 << 0,
    HasTangent = 1 << 1,
    HasUV = 1 << 2,
    HasUV2 = 1 << 3,
    HasColor = 1 << 4
};

constexpr u32 vertexAttribStride(u32 attribFlags)
{
    u32 stride = 12; // position: x, y, z
    if (attribFlags & HasNormal)
        stride += 12;
    if (attribFlags & HasTangent)
        stride += 16;
    if (attribFlags & HasUV)
        stride += 8;
    if (attribFlags & HasUV2)
        stride += 8;
    if (attribFlags & HasColor)
        stride += 4;
    return stride;
}

struct Header
{
    u32 magic = 0;
    u32 version = Version;
    u32 flags = LittleEndian;
};

struct ChunkHeader
{
    u32 id = 0;
    u64 size = 0;
};

struct VertexElement
{
    VertexSemantic semantic = VertexSemantic::Position;
    VertexType type = VertexType::Float3;
    u8 stream = 0;
    u8 index = 0;
    u16 offset = 0;
};

constexpr u32 vertexTypeSize(VertexType type)
{
    switch (type)
    {
    case VertexType::Float1:
        return 4;
    case VertexType::Float2:
        return 8;
    case VertexType::Float3:
        return 12;
    case VertexType::Float4:
        return 16;
    case VertexType::UByte4:
    case VertexType::UByte4N:
        return 4;
    case VertexType::UShort4:
        return 8;
    }
    return 0;
}

class Writer
{
public:
    explicit Writer(ByteArray& output);

    void header(u32 magic, u32 flags = LittleEndian);
    u64 beginChunk(u32 id);
    bool endChunk(u64 marker);
    void string(const std::string& value);
    void writeU8(u8 value);
    void writeU16(u16 value);
    void writeU32(u32 value);
    void writeU64(u64 value);
    void writeS32(s32 value);
    void writeF32(f32 value);
    void bytes(const void* data, usize size);

private:
    ByteArray& mOutput;
};

class Reader
{
public:
    explicit Reader(ByteArray& input);

    bool header(u32 expectedMagic, std::string* error = nullptr);
    bool next(ChunkHeader& chunk);
    bool enter(const ChunkHeader& chunk);
    void leave();
    bool string(std::string& value);
    bool skip(u64 bytes);

    bool readU8(u8& value);
    bool readU16(u16& value);
    bool readU32(u32& value);
    bool readU64(u64& value);
    bool readS32(s32& value);
    bool readF32(f32& value);
    bool bytes(void* data, usize size);
    Radion::u64 remaining() const;

private:
    bool available(Radion::u64 size) const;

    ByteArray& mInput;
    Radion::u64 mLimit;
    Radion::u64 mParentLimit = 0;
};

} // namespace Radion::AssetFormat

#endif // RADION_FORMAT_H

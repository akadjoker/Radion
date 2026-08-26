#include "PCH.h"

#include "NavMesh.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "RadionFormat.h"

#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>

#include <cmath>
#include <cstring>

namespace Radion::AI
{

namespace
{

constexpr s32 kMaxPolys = 512;
constexpr s32 kMaxStraightPath = 2048;
// One frame's move crosses very few polygons; the same 16 the Detour path
// corridor uses for the identical call.
constexpr s32 kMaxVisitedPolys = 16;

} // namespace

NavMesh::~NavMesh()
{
    release();
}

void NavMesh::release()
{
    if (mNavQuery)
    {
        dtFreeNavMeshQuery(static_cast<dtNavMeshQuery*>(mNavQuery));
        mNavQuery = nullptr;
    }
    if (mNavMesh)
    {
        dtFreeNavMesh(static_cast<dtNavMesh*>(mNavMesh));
        mNavMesh = nullptr;
    }
    mDebugTriangles.clear();
}

bool NavMesh::valid() const
{
    return mNavMesh != nullptr && mNavQuery != nullptr;
}

namespace
{

// Polygon-adjacency BFS from `seedRef`, walking each poly's own link list
// (dtPoly::firstLink -> dtMeshTile::links[...].next) rather than distance or
// height - so a flat roof with no stairs/ramp actually connecting it to the
// ground comes out unreached even though its slope alone would pass the
// same walkable test the ground did.
void collectReachablePolys(const dtNavMesh& navMesh, dtPolyRef seedRef,
                           std::vector<dtPolyRef>& outReachable)
{
    outReachable.clear();
    std::vector<dtPolyRef> frontier;
    frontier.push_back(seedRef);
    outReachable.push_back(seedRef);

    while (!frontier.empty())
    {
        const dtPolyRef ref = frontier.back();
        frontier.pop_back();

        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (dtStatusFailed(navMesh.getTileAndPolyByRef(ref, &tile, &poly)) || !tile || !poly)
            continue;

        for (u32 linkIndex = poly->firstLink; linkIndex != DT_NULL_LINK;
            linkIndex = tile->links[linkIndex].next)
        {
            const dtPolyRef neighborRef = tile->links[linkIndex].ref;
            if (neighborRef == 0 ||
                std::find(outReachable.begin(), outReachable.end(), neighborRef) !=
                    outReachable.end())
                continue;
            outReachable.push_back(neighborRef);
            frontier.push_back(neighborRef);
        }
    }
}

} // namespace

bool NavMesh::build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
                    const NavMeshConfig& config, const glm::vec3* groundSeed)
{
    release();
    if (!vertices || !indices || vertexCount <= 0 || triangleCount <= 0)
        return false;

    rcContext ctx(false);

    f32 boundsMin[3];
    f32 boundsMax[3];
    rcCalcBounds(vertices, vertexCount, boundsMin, boundsMax);

    rcConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.cs = config.cellSize;
    cfg.ch = config.cellHeight;
    cfg.walkableSlopeAngle = config.agentMaxSlope;
    cfg.walkableHeight = static_cast<s32>(std::ceil(config.agentHeight / cfg.ch));
    cfg.walkableClimb = static_cast<s32>(std::floor(config.agentMaxClimb / cfg.ch));
    cfg.walkableRadius = static_cast<s32>(std::ceil(config.agentRadius / cfg.cs));
    cfg.maxEdgeLen = static_cast<s32>(config.edgeMaxLen / cfg.cs);
    cfg.maxSimplificationError = config.edgeMaxError;
    cfg.minRegionArea = static_cast<s32>(rcSqr(config.regionMinSize));
    cfg.mergeRegionArea = static_cast<s32>(rcSqr(config.regionMergeSize));
    cfg.maxVertsPerPoly = config.vertsPerPoly;
    cfg.detailSampleDist = config.detailSampleDist < 0.9f ? 0.0f : cfg.cs * config.detailSampleDist;
    cfg.detailSampleMaxError = cfg.ch * config.detailSampleMaxError;
    rcCalcGridSize(boundsMin, boundsMax, cfg.cs, &cfg.width, &cfg.height);
    rcVcopy(cfg.bmin, boundsMin);
    rcVcopy(cfg.bmax, boundsMax);

    // 1. Heightfield: rasterise every walkable triangle into voxel spans.
    rcHeightfield* heightfield = rcAllocHeightfield();
    if (!heightfield || !rcCreateHeightfield(&ctx, *heightfield, cfg.width, cfg.height, cfg.bmin,
                                             cfg.bmax, cfg.cs, cfg.ch))
    {
        rcFreeHeightField(heightfield);
        return false;
    }

    std::vector<u8> areas(static_cast<usize>(triangleCount), 0);
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, vertices, vertexCount, indices,
                            triangleCount, areas.data());
    if (!rcRasterizeTriangles(&ctx, vertices, vertexCount, indices, areas.data(), triangleCount,
                              *heightfield, cfg.walkableClimb))
    {
        rcFreeHeightField(heightfield);
        return false;
    }

    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *heightfield);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *heightfield);

    // 2. Compact heightfield, eroded by the agent radius so the surface
    // already excludes what the agent's own body could not fit into.
    rcCompactHeightfield* compact = rcAllocCompactHeightfield();
    if (!compact || !rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb,
                                               *heightfield, *compact))
    {
        rcFreeHeightField(heightfield);
        rcFreeCompactHeightfield(compact);
        return false;
    }
    rcFreeHeightField(heightfield);

    if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *compact) ||
        !rcBuildDistanceField(&ctx, *compact) ||
        !rcBuildRegions(&ctx, *compact, 0, cfg.minRegionArea, cfg.mergeRegionArea))
    {
        rcFreeCompactHeightfield(compact);
        return false;
    }

    // 3. Contours.
    rcContourSet* contours = rcAllocContourSet();
    if (!contours ||
        !rcBuildContours(&ctx, *compact, cfg.maxSimplificationError, cfg.maxEdgeLen, *contours))
    {
        rcFreeCompactHeightfield(compact);
        rcFreeContourSet(contours);
        return false;
    }

    // 4. Polygon mesh.
    rcPolyMesh* polyMesh = rcAllocPolyMesh();
    if (!polyMesh || !rcBuildPolyMesh(&ctx, *contours, cfg.maxVertsPerPoly, *polyMesh))
    {
        rcFreeCompactHeightfield(compact);
        rcFreeContourSet(contours);
        rcFreePolyMesh(polyMesh);
        return false;
    }

    // 5. Detail mesh - the height detail the flat polygons alone lose.
    rcPolyMeshDetail* detailMesh = rcAllocPolyMeshDetail();
    if (!detailMesh || !rcBuildPolyMeshDetail(&ctx, *polyMesh, *compact, cfg.detailSampleDist,
                                              cfg.detailSampleMaxError, *detailMesh))
    {
        rcFreeCompactHeightfield(compact);
        rcFreeContourSet(contours);
        rcFreePolyMesh(polyMesh);
        rcFreePolyMeshDetail(detailMesh);
        return false;
    }
    rcFreeCompactHeightfield(compact);
    rcFreeContourSet(contours);

    for (s32 i = 0; i < polyMesh->npolys; ++i)
        polyMesh->flags[i] = 1;

    // Debug geometry from the detail mesh, before it is freed - world-space
    // triangles of the surface exactly as the query sees it.
    mDebugTriangles.clear();
    for (s32 mesh = 0; mesh < detailMesh->nmeshes; ++mesh)
    {
        const u32* meshDef = &detailMesh->meshes[mesh * 4];
        const u32 baseVerts = meshDef[0];
        const u32 baseTris = meshDef[2];
        const u32 triCount = meshDef[3];
        const f32* verts = &detailMesh->verts[baseVerts * 3];
        const u8* tris = &detailMesh->tris[baseTris * 4];
        for (u32 tri = 0; tri < triCount; ++tri)
            for (u32 corner = 0; corner < 3; ++corner)
            {
                const f32* v = &verts[tris[tri * 4 + corner] * 3];
                mDebugTriangles.push_back(glm::vec3(v[0], v[1], v[2]));
            }
    }

    // 6. Detour data.
    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof(params));
    params.verts = polyMesh->verts;
    params.vertCount = polyMesh->nverts;
    params.polys = polyMesh->polys;
    params.polyAreas = polyMesh->areas;
    params.polyFlags = polyMesh->flags;
    params.polyCount = polyMesh->npolys;
    params.nvp = polyMesh->nvp;
    params.detailMeshes = detailMesh->meshes;
    params.detailVerts = detailMesh->verts;
    params.detailVertsCount = detailMesh->nverts;
    params.detailTris = detailMesh->tris;
    params.detailTriCount = detailMesh->ntris;
    params.walkableHeight = config.agentHeight;
    params.walkableRadius = config.agentRadius;
    params.walkableClimb = config.agentMaxClimb;
    rcVcopy(params.bmin, polyMesh->bmin);
    rcVcopy(params.bmax, polyMesh->bmax);
    params.cs = cfg.cs;
    params.ch = cfg.ch;
    params.buildBvTree = true;

    u8* navData = nullptr;
    s32 navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
    {
        rcFreePolyMesh(polyMesh);
        rcFreePolyMeshDetail(detailMesh);
        mDebugTriangles.clear();
        return false;
    }
    rcFreePolyMesh(polyMesh);
    rcFreePolyMeshDetail(detailMesh);

    // 7. The navmesh owns navData from here (DT_TILE_FREE_DATA).
    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh || dtStatusFailed(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA)))
    {
        dtFree(navData);
        dtFreeNavMesh(navMesh);
        mDebugTriangles.clear();
        return false;
    }

    dtNavMeshQuery* navQuery = dtAllocNavMeshQuery();
    if (!navQuery || dtStatusFailed(navQuery->init(navMesh, 2048)))
    {
        dtFreeNavMeshQuery(navQuery);
        dtFreeNavMesh(navMesh);
        mDebugTriangles.clear();
        return false;
    }

    mNavMesh = navMesh;
    mNavQuery = navQuery;

    if (groundSeed)
    {
        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);
        const f32 seedPosition[3] = {groundSeed->x, groundSeed->y, groundSeed->z};
        // Generous vertical reach - the seed is whatever the caller judged
        // to be ground level, not a point already known to sit on the mesh.
        const f32 seedExtents[3] = {4.0f, 200.0f, 4.0f};
        dtPolyRef seedRef = 0;
        f32 nearest[3];
        if (dtStatusSucceed(
                navQuery->findNearestPoly(seedPosition, seedExtents, &filter, &seedRef, nearest)) &&
            seedRef != 0)
        {
            std::vector<dtPolyRef> reachable;
            collectReachablePolys(*navMesh, seedRef, reachable);

            // Unreachable polys lose their walkable flag rather than being
            // removed from the tile - dtQueryFilter::passFilter() then
            // excludes them from findNearestPoly()/findPath() the same as
            // any other flagged-off polygon, with no need to touch the
            // Recast build that produced them.
            // getTile(int) is overloaded const/non-const and the non-const
            // one is private - calling it through a non-const dtNavMesh*
            // picks that overload regardless of what the result is assigned
            // to, so the cast forces the public const one instead.
            const dtMeshTile* tile = static_cast<const dtNavMesh*>(navMesh)->getTile(0);
            if (tile)
            {
                for (s32 i = 0; i < tile->header->polyCount; ++i)
                {
                    const dtPolyRef polyRef = navMesh->getPolyRefBase(tile) | static_cast<u32>(i);
                    const bool reached =
                        std::find(reachable.begin(), reachable.end(), polyRef) != reachable.end();
                    if (!reached)
                        navMesh->setPolyFlags(polyRef, 0);
                }

                // debugTriangles() mirrors the same prune: a roof that
                // queries now refuse to route onto should not still draw as
                // if it were part of the walkable surface.
                std::vector<glm::vec3> reachableTriangles;
                reachableTriangles.reserve(mDebugTriangles.size());
                for (s32 meshIndex = 0; meshIndex < tile->header->detailMeshCount; ++meshIndex)
                {
                    const dtPolyRef polyRef =
                        navMesh->getPolyRefBase(tile) | static_cast<u32>(meshIndex);
                    if (std::find(reachable.begin(), reachable.end(), polyRef) == reachable.end())
                        continue;
                    const dtPoly& poly = tile->polys[meshIndex];
                    const dtPolyDetail& detail = tile->detailMeshes[meshIndex];
                    for (u32 tri = 0; tri < detail.triCount; ++tri)
                    {
                        const u8* triangle = &tile->detailTris[(detail.triBase + tri) * 4];
                        for (u32 corner = 0; corner < 3; ++corner)
                        {
                            const u8 vertexIndex = triangle[corner];
                            const f32* v =
                                vertexIndex < poly.vertCount
                                    ? &tile->verts[poly.verts[vertexIndex] * 3]
                                    : &tile->detailVerts[(detail.vertBase +
                                                          (vertexIndex - poly.vertCount)) *
                                                         3];
                            reachableTriangles.push_back(glm::vec3(v[0], v[1], v[2]));
                        }
                    }
                }
                mDebugTriangles = std::move(reachableTriangles);
            }
        }
    }

    return true;
}

bool NavMesh::nearestPoint(const glm::vec3& point, glm::vec3& out,
                           const glm::vec3& searchExtents) const
{
    if (!valid())
        return false;

    const dtNavMeshQuery* query = static_cast<const dtNavMeshQuery*>(mNavQuery);
    dtQueryFilter filter;
    filter.setIncludeFlags(0xffff);
    filter.setExcludeFlags(0);

    const f32 position[3] = {point.x, point.y, point.z};
    const f32 extents[3] = {searchExtents.x, searchExtents.y, searchExtents.z};
    dtPolyRef reference = 0;
    f32 nearest[3] = {0.0f, 0.0f, 0.0f};
    if (dtStatusFailed(query->findNearestPoly(position, extents, &filter, &reference, nearest)) ||
        reference == 0)
        return false;

    out = glm::vec3(nearest[0], nearest[1], nearest[2]);
    return true;
}

bool NavMesh::moveAlongSurface(const glm::vec3& from, const glm::vec3& to, glm::vec3& out,
                               const glm::vec3& searchExtents) const
{
    if (!valid())
        return false;

    const dtNavMeshQuery* query = static_cast<const dtNavMeshQuery*>(mNavQuery);
    dtQueryFilter filter;
    filter.setIncludeFlags(0xffff);
    filter.setExcludeFlags(0);

    const f32 fromPosition[3] = {from.x, from.y, from.z};
    const f32 toPosition[3] = {to.x, to.y, to.z};
    const f32 extents[3] = {searchExtents.x, searchExtents.y, searchExtents.z};

    dtPolyRef startRef = 0;
    f32 nearest[3] = {0.0f, 0.0f, 0.0f};
    if (dtStatusFailed(query->findNearestPoly(fromPosition, extents, &filter, &startRef, nearest)) ||
        startRef == 0)
        return false;

    f32 result[3];
    dtPolyRef visited[kMaxVisitedPolys];
    s32 visitedCount = 0;
    if (dtStatusFailed(query->moveAlongSurface(startRef, fromPosition, toPosition, &filter, result,
                                               visited, &visitedCount, kMaxVisitedPolys)))
        return false;

    // Sit on top of the surface: moveAlongSurface only constrains the move in
    // the XZ plane and leaves the height of the input point.
    f32 height = result[1];
    const dtPolyRef endRef = visitedCount > 0 ? visited[visitedCount - 1] : startRef;
    if (dtStatusSucceed(query->getPolyHeight(endRef, result, &height)))
        result[1] = height;

    out = glm::vec3(result[0], result[1], result[2]);
    return true;
}

bool NavMesh::findPath(const glm::vec3& start, const glm::vec3& end,
                       std::vector<glm::vec3>& outPath, const glm::vec3& searchExtents) const
{
    outPath.clear();
    if (!valid())
        return false;

    const dtNavMeshQuery* query = static_cast<const dtNavMeshQuery*>(mNavQuery);
    dtQueryFilter filter;
    filter.setIncludeFlags(0xffff);
    filter.setExcludeFlags(0);

    const f32 startPosition[3] = {start.x, start.y, start.z};
    const f32 endPosition[3] = {end.x, end.y, end.z};
    const f32 extents[3] = {searchExtents.x, searchExtents.y, searchExtents.z};

    dtPolyRef startRef = 0;
    dtPolyRef endRef = 0;
    f32 nearest[3];
    query->findNearestPoly(startPosition, extents, &filter, &startRef, nearest);
    query->findNearestPoly(endPosition, extents, &filter, &endRef, nearest);
    if (!startRef || !endRef)
        return false;

    dtPolyRef polys[kMaxPolys];
    s32 polyCount = 0;
    if (dtStatusFailed(query->findPath(startRef, endRef, startPosition, endPosition, &filter, polys,
                                       &polyCount, kMaxPolys)) ||
        polyCount == 0)
        return false;

    // A route that stops short of the goal is still a route: pull the end
    // onto the last polygon actually reached instead of discarding it.
    f32 endClamped[3];
    rcVcopy(endClamped, endPosition);
    if (polys[polyCount - 1] != endRef)
        query->closestPointOnPoly(polys[polyCount - 1], endPosition, endClamped, nullptr);

    std::vector<f32> straightPath(static_cast<usize>(kMaxStraightPath) * 3);
    std::vector<u8> straightFlags(static_cast<usize>(kMaxStraightPath));
    std::vector<dtPolyRef> straightPolys(static_cast<usize>(kMaxStraightPath));
    s32 straightCount = 0;
    if (dtStatusFailed(query->findStraightPath(startPosition, endClamped, polys, polyCount,
                                               straightPath.data(), straightFlags.data(),
                                               straightPolys.data(), &straightCount,
                                               kMaxStraightPath)) ||
        straightCount == 0)
        return false;

    outPath.reserve(static_cast<usize>(straightCount));
    for (s32 i = 0; i < straightCount; ++i)
        outPath.push_back(glm::vec3(straightPath[static_cast<usize>(i) * 3 + 0],
                                    straightPath[static_cast<usize>(i) * 3 + 1],
                                    straightPath[static_cast<usize>(i) * 3 + 2]));
    return true;
}

const std::vector<glm::vec3>& NavMesh::debugTriangles() const
{
    return mDebugTriangles;
}

bool NavMesh::save(const std::string& filename) const
{
    if (!valid())
        return false;

    // init() below is the single-tile convenience form (DT_TILE_FREE_DATA,
    // no separate dtNavMeshParams/addTile() call), so the whole navmesh is
    // tile 0 - its data/dataSize is exactly the buffer dtCreateNavMeshData()
    // produced in build(), still owned by the dtNavMesh itself.
    const dtNavMesh* navMesh = static_cast<const dtNavMesh*>(mNavMesh);
    const dtMeshTile* tile = navMesh->getTile(0);
    if (!tile || !tile->data || tile->dataSize <= 0)
        return false;

    ByteArray data;
    AssetFormat::Writer writer(data);
    writer.header(AssetFormat::NavMeshMagic);

    const u64 navChunk = writer.beginChunk(AssetFormat::NavData);
    writer.writeU32(static_cast<u32>(tile->dataSize));
    writer.bytes(tile->data, static_cast<usize>(tile->dataSize));
    writer.endChunk(navChunk);

    const u64 debugChunk = writer.beginChunk(AssetFormat::NavDebugTriangles);
    writer.writeU32(static_cast<u32>(mDebugTriangles.size()));
    for (const glm::vec3& vertex : mDebugTriangles)
    {
        writer.writeF32(vertex.x);
        writer.writeF32(vertex.y);
        writer.writeF32(vertex.z);
    }
    writer.endChunk(debugChunk);

    return FileSystem::getSingleton().writeBinary(filename, data);
}

bool NavMesh::load(const std::string& filename)
{
    release();

    ByteArray data = FileSystem::getSingleton().readBinary(filename);
    if (data.empty())
        return false;

    AssetFormat::Reader reader(data);
    if (!reader.header(AssetFormat::NavMeshMagic))
        return false;

    std::vector<u8> navData;
    AssetFormat::ChunkHeader chunk;
    while (reader.remaining() >= 12 && reader.next(chunk))
    {
        if (!reader.enter(chunk))
            return false;

        if (chunk.id == AssetFormat::NavData)
        {
            u32 size = 0;
            if (!reader.readU32(size) || size == 0)
                return false;
            navData.resize(size);
            if (!reader.bytes(navData.data(), size))
                return false;
        }
        else if (chunk.id == AssetFormat::NavDebugTriangles)
        {
            u32 count = 0;
            if (!reader.readU32(count))
                return false;
            mDebugTriangles.resize(count);
            for (glm::vec3& vertex : mDebugTriangles)
                if (!reader.readF32(vertex.x) || !reader.readF32(vertex.y) ||
                    !reader.readF32(vertex.z))
                    return false;
        }
        reader.leave();
    }

    if (navData.empty())
    {
        mDebugTriangles.clear();
        return false;
    }

    // dtNavMesh wants its own copy it can free later - Detour's own
    // convention is dtAlloc'd memory paired with DT_TILE_FREE_DATA, not
    // whatever container the caller happened to read the file into.
    u8* ownedData = static_cast<u8*>(dtAlloc(navData.size(), DT_ALLOC_PERM));
    if (!ownedData)
    {
        mDebugTriangles.clear();
        return false;
    }
    std::memcpy(ownedData, navData.data(), navData.size());

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh ||
        dtStatusFailed(navMesh->init(ownedData, static_cast<s32>(navData.size()), DT_TILE_FREE_DATA)))
    {
        dtFree(ownedData);
        dtFreeNavMesh(navMesh);
        mDebugTriangles.clear();
        return false;
    }

    dtNavMeshQuery* navQuery = dtAllocNavMeshQuery();
    if (!navQuery || dtStatusFailed(navQuery->init(navMesh, 2048)))
    {
        dtFreeNavMeshQuery(navQuery);
        dtFreeNavMesh(navMesh);
        mDebugTriangles.clear();
        return false;
    }

    mNavMesh = navMesh;
    mNavQuery = navQuery;
    return true;
}

} // namespace Radion::AI

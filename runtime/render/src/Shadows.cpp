#include "PCH.h"

#include "Shadows.h"

namespace Radion
{

DirectionalShadowRegion directionalShadowRegion(u32 atlasSize, u32 cascadeCount, u32 cascade)
{
    const u32 size = glm::max(atlasSize, 1u);
    const u32 count = glm::clamp(cascadeCount, 1u, MaxShadowCascades);
    cascade = glm::min(cascade, count - 1u);

    if (count == 1)
        return {0, 0, size, size};
    if (count == 2)
    {
        const u32 half = glm::max(size / 2u, 1u);
        return {0, cascade * half, size, half};
    }

    const u32 half = glm::max(size / 2u, 1u);
    return {(cascade & 1u) * half, (cascade >> 1u) * half, half, half};
}

namespace
{
struct ShelfPacker
{
    u32 width = 0;
    u32 height = 0;
    u32 x = 0;
    u32 y = 0;
    u32 rowHeight = 0;

    bool add(u32 itemWidth, u32 itemHeight, u32& outputX, u32& outputY)
    {
        if (itemWidth > width || itemHeight > height)
            return false;
        if (x + itemWidth > width)
        {
            y += rowHeight;
            x = 0;
            rowHeight = 0;
        }
        if (y + itemHeight > height)
            return false;
        outputX = x;
        outputY = y;
        x += itemWidth;
        rowHeight = glm::max(rowHeight, itemHeight);
        return true;
    }
};

f32 snapped(f32 value, f32 step)
{
    return step != 0.0f ? std::floor(value / step + 0.5f) * step : value;
}

u32 floorPowerOfTwo(u32 value)
{
    if (value == 0)
        return 0;
    u32 result = 1;
    while (result <= value / 2)
        result *= 2;
    return result;
}

f32 cascadeZeroTexelsPerUnit(CascadeShadowSettings settings, const ShadowCamera& camera,
                             const glm::vec3& lightDirection)
{
    CascadeShadowCalculator calculator;
    calculator.settings = settings;
    CascadeShadowData output;
    if (!calculator.update(camera, lightDirection, output))
        return 0.0f;
    const f32 span = output.halfExtents[0] * 2.0f;
    if (span <= 0.0f)
        return 0.0f;
    return static_cast<f32>(settings.resolution) / span;
}

static const unsigned char kSilhouetteRingSize[64] = {
    0, 4, 4, 0, 4, 6, 6, 8, 4, 6, 6, 8, 6, 6, 6, 6,
    4, 6, 6, 8, 0, 8, 8, 0, 6, 6, 6, 6, 8, 6, 6, 4,
    4, 6, 6, 8, 6, 6, 6, 6, 0, 8, 8, 0, 8, 6, 6, 4,
    6, 6, 6, 6, 8, 6, 6, 4, 8, 6, 6, 4, 0, 4, 4, 0,
};

static const unsigned char kSilhouetteRing[64][8] = {
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 7, 6, 4, 5, 0, 0, 0, 0 },
    { 1, 0, 2, 3, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1, 5, 4, 0, 0, 0, 0, 0 },
    { 1, 5, 7, 6, 4, 0, 0, 0 },
    { 4, 0, 2, 3, 1, 5, 0, 0 },
    { 5, 7, 6, 4, 0, 2, 3, 1 },
    { 0, 4, 6, 2, 0, 0, 0, 0 },
    { 0, 4, 5, 7, 6, 2, 0, 0 },
    { 6, 2, 3, 1, 0, 4, 0, 0 },
    { 2, 3, 1, 0, 4, 5, 7, 6 },
    { 0, 1, 5, 4, 6, 2, 0, 0 },
    { 0, 1, 5, 7, 6, 2, 0, 0 },
    { 6, 2, 3, 1, 5, 4, 0, 0 },
    { 2, 3, 1, 5, 7, 6, 0, 0 },
    { 2, 6, 7, 3, 0, 0, 0, 0 },
    { 2, 6, 4, 5, 7, 3, 0, 0 },
    { 7, 3, 1, 0, 2, 6, 0, 0 },
    { 3, 1, 0, 2, 6, 4, 5, 7 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 2, 6, 4, 0, 1, 5, 7, 3 },
    { 7, 3, 1, 5, 4, 0, 2, 6 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 2, 0, 4, 6, 7, 3, 0, 0 },
    { 2, 0, 4, 5, 7, 3, 0, 0 },
    { 7, 3, 1, 0, 4, 6, 0, 0 },
    { 3, 1, 0, 4, 5, 7, 0, 0 },
    { 2, 0, 1, 5, 4, 6, 7, 3 },
    { 2, 0, 1, 5, 7, 3, 0, 0 },
    { 7, 3, 1, 5, 4, 6, 0, 0 },
    { 3, 1, 5, 7, 0, 0, 0, 0 },
    { 3, 7, 5, 1, 0, 0, 0, 0 },
    { 3, 7, 6, 4, 5, 1, 0, 0 },
    { 5, 1, 0, 2, 3, 7, 0, 0 },
    { 7, 6, 4, 5, 1, 0, 2, 3 },
    { 3, 7, 5, 4, 0, 1, 0, 0 },
    { 3, 7, 6, 4, 0, 1, 0, 0 },
    { 5, 4, 0, 2, 3, 7, 0, 0 },
    { 7, 6, 4, 0, 2, 3, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 3, 7, 6, 2, 0, 4, 5, 1 },
    { 5, 1, 0, 4, 6, 2, 3, 7 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 3, 7, 5, 4, 6, 2, 0, 1 },
    { 3, 7, 6, 2, 0, 1, 0, 0 },
    { 5, 4, 6, 2, 3, 7, 0, 0 },
    { 7, 6, 2, 3, 0, 0, 0, 0 },
    { 3, 2, 6, 7, 5, 1, 0, 0 },
    { 3, 2, 6, 4, 5, 1, 0, 0 },
    { 5, 1, 0, 2, 6, 7, 0, 0 },
    { 1, 0, 2, 6, 4, 5, 0, 0 },
    { 3, 2, 6, 7, 5, 4, 0, 1 },
    { 3, 2, 6, 4, 0, 1, 0, 0 },
    { 5, 4, 0, 2, 6, 7, 0, 0 },
    { 6, 4, 0, 2, 0, 0, 0, 0 },
    { 3, 2, 0, 4, 6, 7, 5, 1 },
    { 3, 2, 0, 4, 5, 1, 0, 0 },
    { 5, 1, 0, 4, 6, 7, 0, 0 },
    { 1, 0, 4, 5, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 3, 2, 0, 1, 0, 0, 0, 0 },
    { 5, 4, 6, 7, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0 },
};

glm::vec4 outwardPlane(const glm::vec4& row)
{
    const f32 length = glm::length(glm::vec3(row));
    if (length < 0.000000000001f)
        return glm::vec4(0.0f);
    return glm::vec4(-glm::vec3(row) / length, row.w / length);
}

glm::vec4 planeFromTriangle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3)
{
    glm::vec3 normal = glm::cross(p1 - p3, p1 - p2);
    const f32 length = glm::length(normal);
    if (length < 0.000000000001f)
        return glm::vec4(0.0f);
    normal /= length;
    return glm::vec4(normal, glm::dot(normal, p1));
}

void appendCullPlane(std::vector<Plane>& output, const glm::vec4& plane)
{
    if (glm::dot(glm::vec3(plane), glm::vec3(plane)) < 0.5f)
        return;
    output.push_back({-glm::vec3(plane), plane.w});
}

void buildCasterCullPlanes(const glm::mat4& sliceViewProjection,
                           const glm::vec3& lightDirection, std::vector<Plane>& output)
{
    output.clear();

    const glm::mat4& M = sliceViewProjection;
    const glm::vec4 row0(M[0][0], M[1][0], M[2][0], M[3][0]);
    const glm::vec4 row1(M[0][1], M[1][1], M[2][1], M[3][1]);
    const glm::vec4 row2(M[0][2], M[1][2], M[2][2], M[3][2]);
    const glm::vec4 row3(M[0][3], M[1][3], M[2][3], M[3][3]);
    const glm::vec4 planes[6] = {
        outwardPlane(row3 + row2), outwardPlane(row3 - row2), outwardPlane(row3 + row0),
        outwardPlane(row3 - row1), outwardPlane(row3 - row0), outwardPlane(row3 + row1)};

    static const glm::vec3 kCornersNdc[8] = {
        {-1.0f, 1.0f, 1.0f},  {-1.0f, -1.0f, 1.0f},  {1.0f, 1.0f, 1.0f},  {1.0f, -1.0f, 1.0f},
        {-1.0f, 1.0f, -1.0f}, {-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}};
    const glm::mat4 inverse = glm::inverse(M);
    glm::vec3 points[8];
    for (u32 i = 0; i < 8; ++i)
    {
        const glm::vec4 p = inverse * glm::vec4(kCornersNdc[i], 1.0f);
        points[i] = glm::vec3(p) / p.w;
    }

    u32 lookup = 0;
    for (u32 n = 0; n < 6; ++n)
        if (glm::dot(glm::vec3(planes[n]), lightDirection) > 0.0f)
        {
            lookup |= 1u << n;
            appendCullPlane(output, planes[n]);
        }

    if (lookup == 63u)
    {
        output.clear();
        for (u32 n = 0; n < 6; ++n)
            appendCullPlane(output, planes[n]);
        return;
    }

    const u32 ringSize = kSilhouetteRingSize[lookup];
    if (ringSize == 0)
    {
        output.clear();
        return;
    }

    for (u32 e = 0; e < ringSize; ++e)
    {
        const glm::vec3& pt0 = points[kSilhouetteRing[lookup][e]];
        const glm::vec3& pt1 = points[kSilhouetteRing[lookup][(e + 1) % ringSize]];
        const glm::vec3 pt2 = pt0 - lightDirection;
        appendCullPlane(output, planeFromTriangle(pt0, pt1, pt2));
    }
}
} // namespace

CascadeShadowSettings CascadeShadowSettings::sizedForScene(f32 sceneRadius)
{
    CascadeShadowSettings settings;
    const f32 radius = glm::max(sceneRadius, 1.0f);

    settings.distance = glm::clamp(radius * 2.0f, settings.distance, 500.0f);
    // Casters well outside the view still throw shadows into it - a column
    // behind the camera, sunlight low enough to rake in from the side - so
    // this reaches further than distance on purpose.
    settings.casterExtrusion = settings.distance * 1.5f;
    // A moderate logarithmic bias keeps useful near density without starving
    // the middle of the view. The previous .9 split placed ordinary demo
    // subjects almost immediately in the final cascade.
    settings.lambda = 0.85f;
    settings.filterRadiusWorld =
        settings.distance / static_cast<f32>(glm::max(settings.resolution, 1u)) * 6.0f;
    // Count and resolution stay at the struct's own defaults (4, 1024): both
    // are a straight cost multiplier on the shadow pass - one more cascade is
    // another whole pass over the caster list, double the resolution is four
    // times the fill rate - and scaling them off the scene automatically was
    // the fps hit the "Auto" button caused the first time this shipped. Going
    // higher resolution is a choice to make from the panel, priced against
    // the frame budget, not a default handed out for free. Four splits are a
    // modest geometry cost, but avoid making one final split cover almost the
    // entire useful view as the old 3/lambda=.9 combination did.
    return settings;
}

CascadeShadowSettings CascadeShadowSettings::sizedForCamera(const CascadeShadowSettings& base,
                                                            f32 sceneRadius,
                                                            const ShadowCamera& camera,
                                                            const glm::vec3& lightDirection,
                                                            f32 targetTexelsPerUnit)
{
    CascadeShadowSettings settings = base;

    f32 low = 1.0f;
    f32 high = sizedForScene(sceneRadius).distance;
    f32 best = low;
    for (u32 i = 0; i < 16; ++i)
    {
        const f32 mid = (low + high) * 0.5f;
        CascadeShadowSettings candidate = settings;
        candidate.distance = mid;
        const f32 density = cascadeZeroTexelsPerUnit(candidate, camera, lightDirection);
        if (density >= targetTexelsPerUnit)
        {
            best = mid;
            low = mid;
        }
        else
        {
            high = mid;
        }
    }

    settings.distance = glm::clamp(best, 10.0f, 500.0f);
    settings.casterExtrusion = settings.distance * 1.5f;
    return settings;
}

bool CascadeShadowCalculator::update(const ShadowCamera& camera, const glm::vec3& lightDirection,
                                     CascadeShadowData& output) const
{
    if (settings.count == 0 || settings.resolution == 0 || camera.nearPlane <= 0.0f ||
        camera.aspect <= 0.0f || camera.fieldOfView <= 0.0f)
        return false;

    output.count = glm::clamp(settings.count, 1u, MaxShadowCascades);
    const u32 splits = output.count;
    const glm::vec3 direction = glm::dot(lightDirection, lightDirection) > 0.000000000001f
                                    ? glm::normalize(lightDirection)
                                    : glm::vec3(0.0f, -1.0f, 0.0f);

    const f32 maxDistance = glm::max(settings.distance, camera.nearPlane + 0.001f);
    const f32 minDistance = glm::min(camera.nearPlane, maxDistance);
    const f32 range = maxDistance - minDistance;

    f32 distances[MaxShadowCascades + 1];
    distances[0] = minDistance;
    for (u32 i = 0; i < splits; ++i)
        distances[i + 1] =
            minDistance + settings.splitOffset[glm::min(i, 2u)] * range;
    distances[splits] = maxDistance;

    const u32 atlasResolution = splits >= 3 ? settings.resolution * 2u : settings.resolution;

    static constexpr f32 kQualityRadius[6] = {1.0f, 1.5f, 2.0f, 2.0f, 3.0f, 4.0f};
    const f32 qualityRadius = kQualityRadius[glm::min(settings.quality, 5u)];
    const f32 tanAngle = settings.angularDiameter > 0.0f
                             ? std::tan(glm::radians(settings.angularDiameter))
                             : 0.0f;
    output.softShadowScale = settings.blur * (tanAngle > 0.0f ? 1.0f : qualityRadius);

    const glm::vec3 zVec = -direction;
    const glm::vec3 upHint =
        glm::abs(zVec.y) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 xVec = glm::normalize(glm::cross(upHint, zVec));
    const glm::vec3 yVec = glm::cross(zVec, xVec);

    const glm::mat4 inverseView = glm::inverse(camera.view);
    const glm::vec3 cameraPosition = glm::vec3(inverseView[3]);

    glm::mat4 biasMatrix(1.0f);
    biasMatrix[0][0] = 0.5f;
    biasMatrix[1][1] = 0.5f;
    biasMatrix[2][2] = 0.5f;
    biasMatrix[3][0] = 0.5f;
    biasMatrix[3][1] = 0.5f;
    biasMatrix[3][2] = 0.5f;

    const glm::vec2 corners[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};

    for (u32 cascade = 0; cascade < splits; ++cascade)
    {
        const f32 sliceNear =
            distances[(cascade == 0 || !settings.blend) ? cascade : cascade - 1];
        const f32 sliceFar = distances[cascade + 1];

        const glm::mat4 sliceProjection = glm::perspective(glm::radians(camera.fieldOfView),
                                                           camera.aspect, sliceNear, sliceFar);
        const glm::mat4 inverseVP = glm::inverse(sliceProjection * camera.view);
        glm::vec3 points[8];
        for (u32 i = 0; i < 4; ++i)
        {
            glm::vec4 a = inverseVP * glm::vec4(corners[i].x, corners[i].y, -1.0f, 1.0f);
            glm::vec4 b = inverseVP * glm::vec4(corners[i].x, corners[i].y, 1.0f, 1.0f);
            points[i] = glm::vec3(a) / a.w;
            points[i + 4] = glm::vec3(b) / b.w;
        }
        buildCasterCullPlanes(sliceProjection * camera.view, direction,
                              output.casterPlanes[cascade]);

        const DirectionalShadowRegion region =
            directionalShadowRegion(atlasResolution, splits, cascade);
        const f32 textureSize = static_cast<f32>(glm::max(region.height, 1u));

        glm::vec3 center(0.0f);
        for (const glm::vec3& point : points)
            center += point;
        center /= 8.0f;

        f32 radius = 0.0f;
        for (const glm::vec3& point : points)
            radius = glm::max(radius, glm::distance(center, point));
        radius *= textureSize / glm::max(textureSize - 2.0f, 1.0f);

        const f32 zDotCenter = glm::dot(zVec, center);
        const f32 zMinCam = zDotCenter - radius;

        f32 softShadowExpand = 0.0f;
        if (tanAngle > 0.0f)
        {
            const f32 zRange = (zDotCenter + radius + settings.pancakeSize) - zMinCam;
            softShadowExpand = tanAngle * zRange;
        }

        f32 xMaxCam = glm::dot(xVec, center) + radius + softShadowExpand;
        f32 xMinCam = glm::dot(xVec, center) - radius - softShadowExpand;
        f32 yMaxCam = glm::dot(yVec, center) + radius + softShadowExpand;
        f32 yMinCam = glm::dot(yVec, center) - radius - softShadowExpand;
        if (settings.stabilize)
        {
            const f32 unit = (radius + softShadowExpand) * 4.0f / textureSize;
            xMaxCam = snapped(xMaxCam, unit);
            xMinCam = snapped(xMinCam, unit);
            yMaxCam = snapped(yMaxCam, unit);
            yMinCam = snapped(yMinCam, unit);
        }

        const f32 zMaxPancake = zDotCenter + radius + settings.pancakeSize;
        const f32 halfX = (xMaxCam - xMinCam) * 0.5f;
        const f32 halfY = (yMaxCam - yMinCam) * 0.5f;
        const f32 zFar = zMaxPancake - zMinCam;

        const glm::mat4 projection = glm::ortho(-halfX, halfX, -halfY, halfY, 0.0f, zFar);
        const glm::mat4 cullProjection =
            glm::ortho(-halfX, halfX, -halfY, halfY, -10000000.0f, zFar);

        const glm::vec3 origin =
            xVec * (xMinCam + halfX) + yVec * (yMinCam + halfY) + zVec * zMaxPancake;
        glm::mat4 lightTransform(1.0f);
        lightTransform[0] = glm::vec4(xVec, 0.0f);
        lightTransform[1] = glm::vec4(yVec, 0.0f);
        lightTransform[2] = glm::vec4(zVec, 0.0f);
        lightTransform[3] = glm::vec4(origin, 1.0f);
        const glm::mat4 view = glm::inverse(lightTransform);

        const f32 atlasSize = static_cast<f32>(atlasResolution);
        const glm::vec4 rect(static_cast<f32>(region.x) / atlasSize,
                             static_cast<f32>(region.y) / atlasSize,
                             static_cast<f32>(region.width) / atlasSize,
                             static_cast<f32>(region.height) / atlasSize);
        glm::mat4 rectMatrix(1.0f);
        rectMatrix[0][0] = rect.z;
        rectMatrix[1][1] = rect.w;
        rectMatrix[3][0] = rect.x;
        rectMatrix[3][1] = rect.y;

        output.viewProjection[cascade] = projection * view;
        output.cullViewProjection[cascade] = cullProjection * view;
        output.shadowMatrix[cascade] = rectMatrix * biasMatrix * projection * view;
        output.splits[cascade] = distances[cascade + 1];
        output.halfExtents[cascade] = glm::max(halfX, halfY);
        output.texelSize[cascade] = radius * 2.0f / textureSize;
        output.shadowBias[cascade] =
            settings.bias / 100.0f * zFar * output.softShadowScale;
        output.shadowNormalBias[cascade] = settings.normalBias * output.texelSize[cascade];
        output.rangeBegin[cascade] = zMaxPancake - glm::dot(zVec, cameraPosition);
        output.uvScale[cascade] =
            glm::vec2(1.0f / glm::max(xMaxCam - xMinCam, 0.000001f),
                      1.0f / glm::max(yMaxCam - yMinCam, 0.000001f)) *
            glm::vec2(rect.z, rect.w);
    }

    for (u32 i = splits; i < MaxShadowCascades; ++i)
    {
        output.viewProjection[i] = output.viewProjection[splits - 1];
        output.cullViewProjection[i] = output.cullViewProjection[splits - 1];
        output.shadowMatrix[i] = output.shadowMatrix[splits - 1];
        output.splits[i] = 0.0f;
        output.halfExtents[i] = output.halfExtents[splits - 1];
        output.shadowBias[i] = output.shadowBias[splits - 1];
        output.shadowNormalBias[i] = output.shadowNormalBias[splits - 1];
        output.rangeBegin[i] = output.rangeBegin[splits - 1];
        output.uvScale[i] = output.uvScale[splits - 1];
        output.texelSize[i] = output.texelSize[splits - 1];
    }

    const f32 lastSplit = output.splits[splits - 1];
    output.fadeFrom = lastSplit * glm::min(settings.fadeStart, 0.999f);
    output.fadeTo = lastSplit;
    return true;
}

void ShadowAtlasLayout::update(const glm::vec3& cameraPosition,
                               const std::vector<RenderLight>& lights)
{
    mTiles.clear();
    mScale = 1.0f;
    if (settings.size == 0 || settings.maximumTileSize == 0)
        return;

    while (mScale > 0.03f)
    {
        mTiles.clear();
        for (usize i = 0; i < lights.size(); ++i)
        {
            const RenderLight& light = lights[i];
            if ((light.flags & RenderLightCastShadow) == 0 ||
                light.type == RenderLightType::Directional || light.range <= 0.0f)
                continue;
            if (light.type == RenderLightType::Point && !settings.point)
                continue;
            if ((light.type == RenderLightType::Spot || light.type == RenderLightType::Rectangle) &&
                !settings.spot)
                continue;
            const f32 distance = glm::max(glm::distance(cameraPosition, light.position), 0.001f);
            f32 importance = glm::min(1.0f, light.range / distance);
            if ((light.flags & RenderLightVolumetric) != 0)
                importance = glm::min(1.0f, importance * settings.volumetricPriority);
            const u32 faceCount = light.type == RenderLightType::Point ? 6 : 1;
            if (faceCount == 6)
                importance = glm::min(1.0f, importance * settings.pointPriority);
            const u32 tile =
                floorPowerOfTwo(static_cast<u32>(settings.maximumTileSize * importance * mScale));
            if (tile < settings.minimumTileSize)
                continue;
            ShadowTile output;
            output.lightIndex = static_cast<u32>(i);
            output.size = tile;
            output.faceCount = faceCount;
            output.importance = importance;
            mTiles.push_back(output);
        }

        std::sort(mTiles.begin(), mTiles.end(),
                  [](const ShadowTile& a, const ShadowTile& b)
                  {
                      return a.importance > b.importance;
                  });
        ShelfPacker packer;
        packer.width = settings.size;
        packer.height = settings.size;
        bool packed = true;
        for (ShadowTile& tile : mTiles)
        {
            if (!packer.add(tile.size * tile.faceCount, tile.size, tile.x, tile.y))
            {
                packed = false;
                break;
            }
        }
        if (packed)
            return;
        mScale *= 0.5f;
    }
    mTiles.clear();
}

} // namespace Radion

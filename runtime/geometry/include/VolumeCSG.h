#ifndef RADION_VOLUME_CSG_H
#define RADION_VOLUME_CSG_H

#include "Noise.h"
#include "VolumeSource.h"

#include <vector>

namespace Radion::Volume
{

class SphereSource final : public Source
{
public:
    SphereSource(Math::Vec3 center, f32 radius);
    f32 sampleDensity(const Math::Vec3& position) const override;
    Sample sample(const Math::Vec3& position) const override;

private:
    Math::Vec3 m_center;
    f32 m_radius;
};

class PlaneSource final : public Source
{
public:
    // Plane equation: dot(normal, position) + offset == 0.
    PlaneSource(Math::Vec3 normal, f32 offset = 0.0f);
    f32 sampleDensity(const Math::Vec3& position) const override;
    Sample sample(const Math::Vec3& position) const override;

private:
    Math::Vec3 m_normal;
    f32 m_offset;
};

class BoxSource final : public Source
{
public:
    BoxSource(Math::Vec3 center, Math::Vec3 halfExtents);
    f32 sampleDensity(const Math::Vec3& position) const override;
    Sample sample(const Math::Vec3& position) const override;

private:
    Math::Vec3 m_center;
    Math::Vec3 m_halfExtents;
};

class BinarySource : public Source
{
protected:
    BinarySource(const Source& left, const Source& right);
    const Source& m_left;
    const Source& m_right;
};

class UnionSource final : public BinarySource
{
public:
    UnionSource(const Source& left, const Source& right);
    f32 sampleDensity(const Math::Vec3&) const override;
    Sample sample(const Math::Vec3&) const override;
};

class IntersectionSource final : public BinarySource
{
public:
    IntersectionSource(const Source& left, const Source& right);
    f32 sampleDensity(const Math::Vec3&) const override;
    Sample sample(const Math::Vec3&) const override;
};

class DifferenceSource final : public BinarySource
{
public:
    DifferenceSource(const Source& left, const Source& right);
    f32 sampleDensity(const Math::Vec3&) const override;
    Sample sample(const Math::Vec3&) const override;
};

class NegateSource final : public Source
{
public:
    explicit NegateSource(const Source& source);
    f32 sampleDensity(const Math::Vec3&) const override;
    Sample sample(const Math::Vec3&) const override;
private:
    const Source& m_source;
};

class ScaleSource final : public Source
{
public:
    ScaleSource(const Source& source, f32 scale);
    f32 sampleDensity(const Math::Vec3&) const override;
    Sample sample(const Math::Vec3&) const override;
private:
    const Source& m_source;
    f32 m_scale;
};

struct NoiseOctave { f32 frequency = 1.0f; f32 amplitude = 1.0f; };

class NoiseSource final : public Source
{
public:
    NoiseSource(u32 seed, f32 frequency = 1.0f, f32 amplitude = 1.0f);
    NoiseSource(const Source& source, u32 seed, f32 frequency = 1.0f, f32 amplitude = 1.0f);
    void setOctaves(std::vector<NoiseOctave> octaves);
    f32 sampleDensity(const Math::Vec3& position) const override;
private:
    f32 noiseAt(const Math::Vec3& position) const;

    const Source* m_source;
    Noise::Perlin m_noise;
    f32 m_frequency;
    f32 m_amplitude;
    std::vector<NoiseOctave> m_octaves;
};

} // namespace Radion::Volume

#endif // RADION_VOLUME_CSG_H

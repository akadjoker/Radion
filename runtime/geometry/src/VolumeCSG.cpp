#include "PCH.h"

#include "VolumeCSG.h"

 

namespace Radion::Volume
{

SphereSource::SphereSource(Math::vec3 center, f32 radius) : m_center(center), m_radius(std::max(radius, 0.0f)) {}
f32 SphereSource::sampleDensity(const Math::vec3& p) const { return m_radius - Math::length(p - m_center); }
Sample SphereSource::sample(const Math::vec3& p) const
{
    const Math::vec3 delta = p - m_center;
    const f32 length = Math::length(delta);
    return {length > std::numeric_limits<f32>::epsilon() ? delta / length : Math::vec3(0.0f), m_radius - length};
}

PlaneSource::PlaneSource(Math::vec3 normal, f32 offset) : m_normal(Math::normalize(normal)), m_offset(offset) {}
f32 PlaneSource::sampleDensity(const Math::vec3& p) const { return Math::dot(m_normal, p) + m_offset; }
Sample PlaneSource::sample(const Math::vec3& p) const { return {m_normal, sampleDensity(p)}; }

BoxSource::BoxSource(Math::vec3 center, Math::vec3 halfExtents) : m_center(center), m_halfExtents(Math::max(halfExtents, Math::vec3(0.0f))) {}
f32 BoxSource::sampleDensity(const Math::vec3& p) const
{
    const Math::vec3 q = Math::abs(p - m_center) - m_halfExtents;
    const Math::vec3 outside = Math::max(q, Math::vec3(0.0f));
    const f32 inside = std::min(std::max(std::max(q.x, q.y), q.z), 0.0f);
    return -Math::length(outside) - inside;
}
Sample BoxSource::sample(const Math::vec3& p) const { return Source::sample(p); }

BinarySource::BinarySource(const Source& left, const Source& right) : m_left(left), m_right(right) {}

UnionSource::UnionSource(const Source& left, const Source& right) : BinarySource(left, right) {}
IntersectionSource::IntersectionSource(const Source& left, const Source& right) : BinarySource(left, right) {}
DifferenceSource::DifferenceSource(const Source& left, const Source& right) : BinarySource(left, right) {}

f32 UnionSource::sampleDensity(const Math::vec3& p) const { return std::max(m_left.sampleDensity(p), m_right.sampleDensity(p)); }
Sample UnionSource::sample(const Math::vec3& p) const { return m_left.sampleDensity(p) >= m_right.sampleDensity(p) ? m_left.sample(p) : m_right.sample(p); }
f32 IntersectionSource::sampleDensity(const Math::vec3& p) const { return std::min(m_left.sampleDensity(p), m_right.sampleDensity(p)); }
Sample IntersectionSource::sample(const Math::vec3& p) const { return m_left.sampleDensity(p) <= m_right.sampleDensity(p) ? m_left.sample(p) : m_right.sample(p); }
f32 DifferenceSource::sampleDensity(const Math::vec3& p) const { return std::min(m_left.sampleDensity(p), -m_right.sampleDensity(p)); }
Sample DifferenceSource::sample(const Math::vec3& p) const
{
    const Sample a = m_left.sample(p); const Sample b = m_right.sample(p);
    return a.density <= -b.density ? a : Sample{-b.gradient, -b.density};
}

NegateSource::NegateSource(const Source& source) : m_source(source) {}
f32 NegateSource::sampleDensity(const Math::vec3& p) const { return -m_source.sampleDensity(p); }
Sample NegateSource::sample(const Math::vec3& p) const { const Sample s = m_source.sample(p); return {-s.gradient, -s.density}; }

ScaleSource::ScaleSource(const Source& source, f32 scale) : m_source(source), m_scale(scale) {}
f32 ScaleSource::sampleDensity(const Math::vec3& p) const { return m_scale == 0.0f ? -std::numeric_limits<f32>::infinity() : m_source.sampleDensity(p / m_scale) * std::abs(m_scale); }
Sample ScaleSource::sample(const Math::vec3& p) const
{
    if (m_scale == 0.0f) return {{0.0f, 0.0f, 0.0f}, -std::numeric_limits<f32>::infinity()};
    const Sample s = m_source.sample(p / m_scale);
    return {s.gradient, sampleDensity(p)};
}

NoiseSource::NoiseSource(u32 seed, f32 frequency, f32 amplitude)
    : m_source(nullptr), m_frequency(frequency), m_amplitude(amplitude) { m_noise.initialise(seed); }
NoiseSource::NoiseSource(const Source& source, u32 seed, f32 frequency, f32 amplitude)
    : m_source(&source), m_frequency(frequency), m_amplitude(amplitude) { m_noise.initialise(seed); }
void NoiseSource::setOctaves(std::vector<NoiseOctave> octaves) { m_octaves = std::move(octaves); }
f32 NoiseSource::noiseAt(const Math::vec3& p) const
{
    if (m_octaves.empty()) return m_noise.compute(p.x * m_frequency, p.y * m_frequency, p.z * m_frequency) * m_amplitude;
    f32 value = 0.0f; for (const NoiseOctave& octave : m_octaves) value += m_noise.compute(p.x * octave.frequency, p.y * octave.frequency, p.z * octave.frequency) * octave.amplitude;
    return value * m_amplitude;
}
f32 NoiseSource::sampleDensity(const Math::vec3& p) const
{
    return m_source ? m_source->sampleDensity(p) + noiseAt(p) : noiseAt(p);
}

} // namespace Radion::Volume

#include "PCH.h"

#include "VolumeCSG.h"

 

namespace Radion::Volume
{

SphereSource::SphereSource(glm::vec3 center, f32 radius) : m_center(center), m_radius(std::max(radius, 0.0f)) {}
f32 SphereSource::sampleDensity(const glm::vec3& p) const { return m_radius - glm::length(p - m_center); }
Sample SphereSource::sample(const glm::vec3& p) const
{
    const glm::vec3 delta = p - m_center;
    const f32 length = glm::length(delta);
    return {length > std::numeric_limits<f32>::epsilon() ? delta / length : glm::vec3(0.0f), m_radius - length};
}

PlaneSource::PlaneSource(glm::vec3 normal, f32 offset) : m_normal(glm::normalize(normal)), m_offset(offset) {}
f32 PlaneSource::sampleDensity(const glm::vec3& p) const { return glm::dot(m_normal, p) + m_offset; }
Sample PlaneSource::sample(const glm::vec3& p) const { return {m_normal, sampleDensity(p)}; }

BoxSource::BoxSource(glm::vec3 center, glm::vec3 halfExtents) : m_center(center), m_halfExtents(glm::max(halfExtents, glm::vec3(0.0f))) {}
f32 BoxSource::sampleDensity(const glm::vec3& p) const
{
    const glm::vec3 q = glm::abs(p - m_center) - m_halfExtents;
    const glm::vec3 outside = glm::max(q, glm::vec3(0.0f));
    const f32 inside = std::min(std::max(std::max(q.x, q.y), q.z), 0.0f);
    return -glm::length(outside) - inside;
}
Sample BoxSource::sample(const glm::vec3& p) const { return Source::sample(p); }

BinarySource::BinarySource(const Source& left, const Source& right) : m_left(left), m_right(right) {}

UnionSource::UnionSource(const Source& left, const Source& right) : BinarySource(left, right) {}
IntersectionSource::IntersectionSource(const Source& left, const Source& right) : BinarySource(left, right) {}
DifferenceSource::DifferenceSource(const Source& left, const Source& right) : BinarySource(left, right) {}

f32 UnionSource::sampleDensity(const glm::vec3& p) const { return std::max(m_left.sampleDensity(p), m_right.sampleDensity(p)); }
Sample UnionSource::sample(const glm::vec3& p) const { return m_left.sampleDensity(p) >= m_right.sampleDensity(p) ? m_left.sample(p) : m_right.sample(p); }
f32 IntersectionSource::sampleDensity(const glm::vec3& p) const { return std::min(m_left.sampleDensity(p), m_right.sampleDensity(p)); }
Sample IntersectionSource::sample(const glm::vec3& p) const { return m_left.sampleDensity(p) <= m_right.sampleDensity(p) ? m_left.sample(p) : m_right.sample(p); }
f32 DifferenceSource::sampleDensity(const glm::vec3& p) const { return std::min(m_left.sampleDensity(p), -m_right.sampleDensity(p)); }
Sample DifferenceSource::sample(const glm::vec3& p) const
{
    const Sample a = m_left.sample(p); const Sample b = m_right.sample(p);
    return a.density <= -b.density ? a : Sample{-b.gradient, -b.density};
}

NegateSource::NegateSource(const Source& source) : m_source(source) {}
f32 NegateSource::sampleDensity(const glm::vec3& p) const { return -m_source.sampleDensity(p); }
Sample NegateSource::sample(const glm::vec3& p) const { const Sample s = m_source.sample(p); return {-s.gradient, -s.density}; }

ScaleSource::ScaleSource(const Source& source, f32 scale) : m_source(source), m_scale(scale) {}
f32 ScaleSource::sampleDensity(const glm::vec3& p) const { return m_scale == 0.0f ? -std::numeric_limits<f32>::infinity() : m_source.sampleDensity(p / m_scale) * std::abs(m_scale); }
Sample ScaleSource::sample(const glm::vec3& p) const
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
f32 NoiseSource::noiseAt(const glm::vec3& p) const
{
    if (m_octaves.empty()) return m_noise.compute(p.x * m_frequency, p.y * m_frequency, p.z * m_frequency) * m_amplitude;
    f32 value = 0.0f; for (const NoiseOctave& octave : m_octaves) value += m_noise.compute(p.x * octave.frequency, p.y * octave.frequency, p.z * octave.frequency) * octave.amplitude;
    return value * m_amplitude;
}
f32 NoiseSource::sampleDensity(const glm::vec3& p) const
{
    return m_source ? m_source->sampleDensity(p) + noiseAt(p) : noiseAt(p);
}

} // namespace Radion::Volume

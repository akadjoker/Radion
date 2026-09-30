#ifndef RADION_AUDIO_ENGINE_H
#define RADION_AUDIO_ENGINE_H

#include "Types.h"

#include "Math.h"
#include <string>

namespace Radion
{

// SoundId names a loaded file; a VoiceId one playback of it. Sounds are held as encoded bytes.
class AudioEngine
{
public:
    struct Impl;
    using SoundId = s32;
    using VoiceId = s32;

    static AudioEngine& getSingleton();

    AudioEngine();
    ~AudioEngine();

    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    bool initialize();
    void shutdown();
    bool ready() const;

    SoundId loadSound(const std::string& path);
    SoundId loadMusic(const std::string& path);
    // Bytes are copied; the caller may release its buffer on return.
    SoundId loadSoundMemory(const void* data, usize size);
    SoundId loadMusicMemory(const void* data, usize size);
    bool unload(SoundId sound);
    void clear();

    VoiceId play(SoundId sound, f32 volume = 1.0f, f32 pitch = 1.0f, f32 pan = 0.0f);
    VoiceId playMusic(SoundId sound, bool loop = true, f32 volume = 1.0f);
    bool stop(VoiceId voice);
    bool pause(VoiceId voice);
    bool resume(VoiceId voice);
    bool isPlaying(VoiceId voice) const;
    bool setVoiceVolume(VoiceId voice, f32 volume);
    bool setVoicePitch(VoiceId voice, f32 pitch);
    bool setVoicePan(VoiceId voice, f32 pan);
    bool fadeIn(VoiceId voice, f32 seconds);
    bool fadeOut(VoiceId voice, f32 seconds, bool stopWhenDone = false);
    VoiceId crossfadeMusic(SoundId sound, bool loop = true, f32 volume = 1.0f,
                           f32 seconds = 1.0f);

    // Set both position and orientation: a stale orientation pans every voice wrongly as the camera turns.
    bool setListenerPosition(const Math::vec3& position);
    bool setListenerOrientation(const Math::vec3& forward, const Math::vec3& up);
    bool setListenerVelocity(const Math::vec3& velocity);
    bool setVoicePosition(VoiceId voice, const Math::vec3& position);
    bool setVoiceVelocity(VoiceId voice, const Math::vec3& velocity);
    // World units; silent past maxDistance, rolloff shapes the inverse curve.
    bool setVoiceSpatial(VoiceId voice, bool enabled, f32 minDistance = 1.0f,
                         f32 maxDistance = 100.0f, f32 rolloff = 1.0f);
    VoiceId playAt(SoundId sound, const Math::vec3& position, f32 volume = 1.0f,
                   f32 pitch = 1.0f, f32 minDistance = 1.0f, f32 maxDistance = 100.0f,
                   f32 rolloff = 1.0f);
    void stopAll();
    void stopMusic();

    void setMasterVolume(f32 volume);
    void setSfxVolume(f32 volume);
    void setMusicVolume(f32 volume);
    f32 masterVolume() const;
    f32 sfxVolume() const;
    f32 musicVolume() const;
    void setMasterMuted(bool muted);
    void setSfxMuted(bool muted);
    void setMusicMuted(bool muted);
    bool masterMuted() const;
    bool sfxMuted() const;
    bool musicMuted() const;

    u32 voiceCount() const;
    u32 soundCount() const;

    // Reclaims finished non-looping voices. Call once per frame.
    void update();

private:
    Impl* mImpl;
};

AudioEngine& Audio();

} // namespace Radion

#endif // RADION_AUDIO_ENGINE_H

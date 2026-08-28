#ifndef RADION_AUDIO_PLAYER_H
#define RADION_AUDIO_PLAYER_H

#include "AudioEngine.h"
#include "Component.h"

#include <string>

namespace Radion
{

// One sound attached to a GameObject. Non-spatial by default: a spatial
// voice follows its owner's world position every frame and attenuates
// against the listener the Scene sets from the active camera.
class AudioPlayer final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::AudioPlayer;

    void setSource(const std::string& path);
    const std::string& source() const;
    // Music streams from one voice at a time and ignores pan; sound effects
    // decode up front and overlap freely.
    void setMusic(bool music);
    bool music() const;
    void setAutoplay(bool autoplay);
    bool autoplay() const;
    void setLoop(bool loop);
    bool loop() const;
    void setVolume(f32 volume);
    f32 volume() const;
    void setPitch(f32 pitch);
    f32 pitch() const;
    void setPan(f32 pan);
    f32 pan() const;
    void setSpatial(bool spatial);
    bool spatial() const;
    void setMinDistance(f32 distance);
    f32 minDistance() const;
    void setMaxDistance(f32 distance);
    f32 maxDistance() const;
    void setRolloff(f32 rolloff);
    f32 rolloff() const;

    AudioEngine::VoiceId play();
    bool stop();
    bool pause();
    bool resume();
    bool playing() const;

private:
    friend class GameObject;

    AudioPlayer();

    void onStart() override;
    void onUpdate(f32 deltaTime) override;
    void onDisable() override;
    void onDestroy() override;

    void releaseSound();
    AudioEngine::SoundId load();

    std::string mSource;
    AudioEngine::SoundId mSound = 0;
    AudioEngine::VoiceId mVoice = 0;
    f32 mVolume = 1.0f;
    f32 mPitch = 1.0f;
    f32 mPan = 0.0f;
    f32 mMinDistance = 1.0f;
    f32 mMaxDistance = 100.0f;
    f32 mRolloff = 1.0f;
    bool mMusic = false;
    bool mAutoplay = false;
    bool mLoop = false;
    bool mSpatial = false;
};

} // namespace Radion

#endif // RADION_AUDIO_PLAYER_H

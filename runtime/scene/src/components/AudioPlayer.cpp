#include "AudioPlayer.h"

#include "GameObject.h"

namespace Radion
{
namespace
{
f32 clampValue(f32 value, f32 minimum, f32 maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}
} // namespace

AudioPlayer::AudioPlayer() : Component(Type, ComponentEventUpdate)
{
}

void AudioPlayer::setSource(const std::string& path)
{
    if (mSource == path)
        return;
    releaseSound();
    mSource = path;
}

const std::string& AudioPlayer::source() const
{
    return mSource;
}

void AudioPlayer::setMusic(bool music)
{
    if (mMusic == music)
        return;
    releaseSound();
    mMusic = music;
}

bool AudioPlayer::music() const
{
    return mMusic;
}

void AudioPlayer::setAutoplay(bool autoplay)
{
    mAutoplay = autoplay;
}

bool AudioPlayer::autoplay() const
{
    return mAutoplay;
}

void AudioPlayer::setLoop(bool loop)
{
    mLoop = loop;
}

bool AudioPlayer::loop() const
{
    return mLoop;
}

void AudioPlayer::setVolume(f32 volume)
{
    mVolume = clampValue(volume, 0.0f, 4.0f);
    if (mVoice)
        Audio().setVoiceVolume(mVoice, mVolume);
}

f32 AudioPlayer::volume() const
{
    return mVolume;
}

void AudioPlayer::setPitch(f32 pitch)
{
    mPitch = clampValue(pitch, 0.01f, 4.0f);
    if (mVoice)
        Audio().setVoicePitch(mVoice, mPitch);
}

f32 AudioPlayer::pitch() const
{
    return mPitch;
}

void AudioPlayer::setPan(f32 pan)
{
    mPan = clampValue(pan, -1.0f, 1.0f);
    if (mVoice && !mMusic)
        Audio().setVoicePan(mVoice, mPan);
}

f32 AudioPlayer::pan() const
{
    return mPan;
}

void AudioPlayer::setSpatial(bool spatial)
{
    mSpatial = spatial;
    if (mVoice)
        Audio().setVoiceSpatial(mVoice, mSpatial, mMinDistance, mMaxDistance, mRolloff);
}

bool AudioPlayer::spatial() const
{
    return mSpatial;
}

void AudioPlayer::setMinDistance(f32 distance)
{
    mMinDistance = clampValue(distance, 0.0f, 1000000.0f);
    if (mMaxDistance < mMinDistance)
        mMaxDistance = mMinDistance;
    if (mVoice && mSpatial)
        Audio().setVoiceSpatial(mVoice, true, mMinDistance, mMaxDistance, mRolloff);
}

f32 AudioPlayer::minDistance() const
{
    return mMinDistance;
}

void AudioPlayer::setMaxDistance(f32 distance)
{
    mMaxDistance = clampValue(distance, mMinDistance, 1000000.0f);
    if (mVoice && mSpatial)
        Audio().setVoiceSpatial(mVoice, true, mMinDistance, mMaxDistance, mRolloff);
}

f32 AudioPlayer::maxDistance() const
{
    return mMaxDistance;
}

void AudioPlayer::setRolloff(f32 rolloff)
{
    mRolloff = clampValue(rolloff, 0.0f, 100.0f);
    if (mVoice && mSpatial)
        Audio().setVoiceSpatial(mVoice, true, mMinDistance, mMaxDistance, mRolloff);
}

f32 AudioPlayer::rolloff() const
{
    return mRolloff;
}

AudioEngine::SoundId AudioPlayer::load()
{
    if (mSound || mSource.empty())
        return mSound;
    mSound = mMusic ? Audio().loadMusic(mSource) : Audio().loadSound(mSource);
    return mSound;
}

AudioEngine::VoiceId AudioPlayer::play()
{
    const AudioEngine::SoundId sound = load();
    if (!sound)
        return 0;
    mVoice = mMusic ? Audio().playMusic(sound, mLoop, mVolume)
                    : (mSpatial && owner()
                           ? Audio().playAt(sound, owner()->globalPosition(), mVolume, mPitch,
                                            mMinDistance, mMaxDistance, mRolloff)
                           : Audio().play(sound, mVolume, mPitch, mPan));
    return mVoice;
}

bool AudioPlayer::stop()
{
    const bool stopped = mVoice && Audio().stop(mVoice);
    mVoice = 0;
    return stopped;
}

bool AudioPlayer::pause()
{
    return mVoice && Audio().pause(mVoice);
}

bool AudioPlayer::resume()
{
    return mVoice && Audio().resume(mVoice);
}

bool AudioPlayer::playing() const
{
    return mVoice && Audio().isPlaying(mVoice);
}

void AudioPlayer::releaseSound()
{
    stop();
    if (mSound)
        Audio().unload(mSound);
    mSound = 0;
}

void AudioPlayer::onStart()
{
    if (mAutoplay)
        play();
}

void AudioPlayer::onUpdate(f32)
{
    if (mSpatial && mVoice && owner())
        Audio().setVoicePosition(mVoice, owner()->globalPosition());
}

void AudioPlayer::onDisable()
{
    stop();
}

void AudioPlayer::onDestroy()
{
    releaseSound();
}

} // namespace Radion

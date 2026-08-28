#include "AudioEngine.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "Log.h"

#include "miniaudio/miniaudio.h"

#include <cstring>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Radion
{
namespace
{
f32 clampValue(f32 value, f32 minimum, f32 maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}
} // namespace

struct AudioEngine::Impl
{
    struct Sound
    {
        SoundId id = 0;
        std::string path;
        std::vector<u8> bytes;
        bool music = false;
    };

    struct Voice
    {
        VoiceId id = 0;
        SoundId sound = 0;
        bool loop = false;
        bool music = false;
        bool hasDecoder = false;
        bool paused = false;
        bool removeWhenStopped = false;
        f32 volume = 1.0f;
        ma_decoder decoder{};
        ma_sound player{};
    };

    ma_engine engine{};
    ma_sound_group sfxGroup{};
    ma_sound_group musicGroup{};
    bool ready = false;
    bool groupsReady = false;
    SoundId nextSound = 1;
    VoiceId nextVoice = 1;
    VoiceId musicVoice = 0;
    f32 masterVolume = 1.0f;
    f32 sfxVolume = 1.0f;
    f32 musicVolume = 1.0f;
    bool masterMuted = false;
    bool sfxMuted = false;
    bool musicMuted = false;
    std::unordered_map<SoundId, Sound> sounds;
    std::vector<Voice*> voices;
};

namespace
{
u64 milliseconds(f32 seconds)
{
    return seconds > 0.0f ? static_cast<u64>(seconds * 1000.0f + 0.5f) : 0;
}

void applyMixer(AudioEngine::Impl* impl)
{
    if (!impl || !impl->ready)
        return;
    ma_engine_set_volume(&impl->engine, impl->masterMuted ? 0.0f : impl->masterVolume);
    if (impl->groupsReady)
    {
        ma_sound_group_set_volume(&impl->sfxGroup, impl->sfxMuted ? 0.0f : impl->sfxVolume);
        ma_sound_group_set_volume(&impl->musicGroup,
                                  impl->musicMuted ? 0.0f : impl->musicVolume);
    }
}

AudioEngine::Impl::Voice* findVoice(AudioEngine::Impl* impl, AudioEngine::VoiceId id)
{
    if (!impl || id <= 0)
        return nullptr;
    for (AudioEngine::Impl::Voice* voice : impl->voices)
        if (voice && voice->id == id)
            return voice;
    return nullptr;
}

const AudioEngine::Impl::Voice* findVoice(const AudioEngine::Impl* impl,
                                          AudioEngine::VoiceId id)
{
    if (!impl || id <= 0)
        return nullptr;
    for (const AudioEngine::Impl::Voice* voice : impl->voices)
        if (voice && voice->id == id)
            return voice;
    return nullptr;
}

void destroyVoice(AudioEngine::Impl* impl, AudioEngine::Impl::Voice* voice)
{
    if (!voice)
        return;
    if (impl && impl->musicVoice == voice->id)
        impl->musicVoice = 0;
    ma_sound_stop(&voice->player);
    ma_sound_uninit(&voice->player);
    if (voice->hasDecoder)
        ma_decoder_uninit(&voice->decoder);
    delete voice;
}

AudioEngine::Impl::Voice* createMusicVoice(AudioEngine::Impl* impl,
                                           AudioEngine::SoundId soundId, bool loop, f32 volume)
{
    if (!impl || !impl->ready)
        return nullptr;
    const auto found = impl->sounds.find(soundId);
    if (found == impl->sounds.end() || !found->second.music)
        return nullptr;

    AudioEngine::Impl::Voice* voice = new AudioEngine::Impl::Voice();
    voice->id = impl->nextVoice++;
    voice->sound = soundId;
    voice->loop = loop;
    voice->music = true;
    voice->volume = clampValue(volume, 0.0f, 4.0f);
    ma_result result = MA_ERROR;
    if (!found->second.bytes.empty())
    {
        const ma_decoder_config config = ma_decoder_config_init_default();
        result = ma_decoder_init_memory(found->second.bytes.data(), found->second.bytes.size(),
                                        &config, &voice->decoder);
        if (result == MA_SUCCESS)
        {
            voice->hasDecoder = true;
            result = ma_sound_init_from_data_source(&impl->engine, &voice->decoder, 0,
                                                    &impl->musicGroup, &voice->player);
        }
    }
    else
    {
        result = ma_sound_init_from_file(&impl->engine, found->second.path.c_str(),
                                         MA_SOUND_FLAG_STREAM, &impl->musicGroup, nullptr,
                                         &voice->player);
    }
    if (result != MA_SUCCESS)
    {
        if (voice->hasDecoder)
            ma_decoder_uninit(&voice->decoder);
        delete voice;
        return nullptr;
    }
    ma_sound_set_looping(&voice->player, loop ? MA_TRUE : MA_FALSE);
    ma_sound_set_spatialization_enabled(&voice->player, MA_FALSE);
    ma_sound_set_volume(&voice->player, voice->volume);
    if (ma_sound_start(&voice->player) != MA_SUCCESS)
    {
        ma_sound_uninit(&voice->player);
        if (voice->hasDecoder)
            ma_decoder_uninit(&voice->decoder);
        delete voice;
        return nullptr;
    }
    impl->voices.push_back(voice);
    return voice;
}
} // namespace

AudioEngine& AudioEngine::getSingleton()
{
    static AudioEngine audio;
    return audio;
}

AudioEngine& Audio()
{
    return AudioEngine::getSingleton();
}

AudioEngine::AudioEngine() : mImpl(new Impl())
{
}

AudioEngine::~AudioEngine()
{
    shutdown();
    delete mImpl;
    mImpl = nullptr;
}

bool AudioEngine::initialize()
{
    if (!mImpl)
        return false;
    if (mImpl->ready)
        return true;

    ma_engine_config config = ma_engine_config_init();
    if (ma_engine_init(&config, &mImpl->engine) != MA_SUCCESS)
    {
        Log::error("Audio: no output device, playback is silent this run");
        return false;
    }
    if (ma_sound_group_init(&mImpl->engine, 0, nullptr, &mImpl->sfxGroup) != MA_SUCCESS)
    {
        ma_engine_uninit(&mImpl->engine);
        return false;
    }
    if (ma_sound_group_init(&mImpl->engine, 0, nullptr, &mImpl->musicGroup) != MA_SUCCESS)
    {
        ma_sound_group_uninit(&mImpl->sfxGroup);
        ma_engine_uninit(&mImpl->engine);
        return false;
    }

    mImpl->groupsReady = true;
    mImpl->ready = true;
    applyMixer(mImpl);
    return true;
}

void AudioEngine::shutdown()
{
    if (!mImpl)
        return;
    stopAll();
    clear();
    if (!mImpl->ready)
        return;
    if (mImpl->groupsReady)
    {
        ma_sound_group_uninit(&mImpl->musicGroup);
        ma_sound_group_uninit(&mImpl->sfxGroup);
        mImpl->groupsReady = false;
    }
    ma_engine_uninit(&mImpl->engine);
    mImpl->ready = false;
}

bool AudioEngine::ready() const
{
    return mImpl && mImpl->ready;
}

AudioEngine::SoundId AudioEngine::loadSound(const std::string& path)
{
    if (!mImpl || path.empty())
        return 0;
    const ByteArray file = FileSystem::getSingleton().readBinary(path);
    if (file.size() == 0)
    {
        Log::warning("Audio: '%s' is missing or empty", path.c_str());
        return 0;
    }
    const SoundId id = mImpl->nextSound++;
    Impl::Sound sound;
    sound.id = id;
    sound.path = path;
    sound.bytes.resize(file.size());
    std::memcpy(sound.bytes.data(), file.data(), file.size());
    mImpl->sounds[id] = std::move(sound);
    return id;
}

AudioEngine::SoundId AudioEngine::loadMusic(const std::string& path)
{
    const SoundId id = loadSound(path);
    if (id)
        mImpl->sounds[id].music = true;
    return id;
}

AudioEngine::SoundId AudioEngine::loadSoundMemory(const void* data, usize size)
{
    if (!mImpl || !data || size == 0)
        return 0;
    const SoundId id = mImpl->nextSound++;
    Impl::Sound sound;
    sound.id = id;
    sound.bytes.resize(size);
    std::memcpy(sound.bytes.data(), data, size);
    mImpl->sounds[id] = std::move(sound);
    return id;
}

AudioEngine::SoundId AudioEngine::loadMusicMemory(const void* data, usize size)
{
    const SoundId id = loadSoundMemory(data, size);
    if (id)
        mImpl->sounds[id].music = true;
    return id;
}

bool AudioEngine::unload(SoundId sound)
{
    if (!mImpl || sound <= 0)
        return false;
    for (auto it = mImpl->voices.begin(); it != mImpl->voices.end();)
    {
        if (*it && (*it)->sound == sound)
        {
            destroyVoice(mImpl, *it);
            it = mImpl->voices.erase(it);
        }
        else
            ++it;
    }
    return mImpl->sounds.erase(sound) != 0;
}

void AudioEngine::clear()
{
    if (!mImpl)
        return;
    stopAll();
    mImpl->sounds.clear();
    mImpl->nextSound = 1;
}

AudioEngine::VoiceId AudioEngine::play(SoundId sound, f32 volume, f32 pitch, f32 pan)
{
    if (!mImpl || !mImpl->ready)
        return 0;
    const auto found = mImpl->sounds.find(sound);
    if (found == mImpl->sounds.end() || found->second.music)
        return 0;

    Impl::Voice* voice = new Impl::Voice();
    voice->id = mImpl->nextVoice++;
    voice->sound = sound;
    ma_result result = MA_ERROR;
    if (!found->second.bytes.empty())
    {
        const ma_decoder_config config = ma_decoder_config_init_default();
        result = ma_decoder_init_memory(found->second.bytes.data(), found->second.bytes.size(),
                                        &config, &voice->decoder);
        if (result == MA_SUCCESS)
        {
            voice->hasDecoder = true;
            result = ma_sound_init_from_data_source(&mImpl->engine, &voice->decoder, 0,
                                                    &mImpl->sfxGroup, &voice->player);
        }
    }
    else
    {
        result = ma_sound_init_from_file(&mImpl->engine, found->second.path.c_str(),
                                         MA_SOUND_FLAG_DECODE, &mImpl->sfxGroup, nullptr,
                                         &voice->player);
    }
    if (result != MA_SUCCESS)
    {
        if (voice->hasDecoder)
            ma_decoder_uninit(&voice->decoder);
        delete voice;
        return 0;
    }
    voice->volume = clampValue(volume, 0.0f, 4.0f);
    ma_sound_set_volume(&voice->player, voice->volume);
    ma_sound_set_pitch(&voice->player, clampValue(pitch, 0.01f, 4.0f));
    ma_sound_set_spatialization_enabled(&voice->player, MA_FALSE);
    ma_sound_set_pan(&voice->player, clampValue(pan, -1.0f, 1.0f));
    if (ma_sound_start(&voice->player) != MA_SUCCESS)
    {
        ma_sound_uninit(&voice->player);
        if (voice->hasDecoder)
            ma_decoder_uninit(&voice->decoder);
        delete voice;
        return 0;
    }
    mImpl->voices.push_back(voice);
    return voice->id;
}

AudioEngine::VoiceId AudioEngine::playMusic(SoundId sound, bool loop, f32 volume)
{
    Impl::Voice* voice = createMusicVoice(mImpl, sound, loop, volume);
    if (!voice)
        return 0;
    stopMusic();
    mImpl->musicVoice = voice->id;
    return voice->id;
}

bool AudioEngine::stop(VoiceId voice)
{
    if (!mImpl)
        return false;
    for (auto it = mImpl->voices.begin(); it != mImpl->voices.end(); ++it)
    {
        if (*it && (*it)->id == voice)
        {
            destroyVoice(mImpl, *it);
            mImpl->voices.erase(it);
            return true;
        }
    }
    return false;
}

bool AudioEngine::pause(VoiceId voice)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found || ma_sound_stop(&found->player) != MA_SUCCESS)
        return false;
    found->paused = true;
    return true;
}

bool AudioEngine::resume(VoiceId voice)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found || ma_sound_start(&found->player) != MA_SUCCESS)
        return false;
    found->paused = false;
    return true;
}

bool AudioEngine::isPlaying(VoiceId voice) const
{
    const Impl::Voice* found = findVoice(mImpl, voice);
    return found && ma_sound_is_playing(&found->player) == MA_TRUE;
}

bool AudioEngine::setVoiceVolume(VoiceId voice, f32 volume)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    found->volume = clampValue(volume, 0.0f, 4.0f);
    ma_sound_set_volume(&found->player, found->volume);
    // Cancels a fade still in flight: without this the fade keeps writing
    // its own gain over the volume just set.
    ma_sound_set_fade_in_milliseconds(&found->player, 1.0f, 1.0f, 0);
    return true;
}

bool AudioEngine::setVoicePitch(VoiceId voice, f32 pitch)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    ma_sound_set_pitch(&found->player, clampValue(pitch, 0.01f, 4.0f));
    return true;
}

bool AudioEngine::setVoicePan(VoiceId voice, f32 pan)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    ma_sound_set_pan(&found->player, clampValue(pan, -1.0f, 1.0f));
    return true;
}

bool AudioEngine::fadeIn(VoiceId voice, f32 seconds)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    ma_sound_set_fade_in_milliseconds(&found->player, 0.0f, 1.0f, milliseconds(seconds));
    return true;
}

bool AudioEngine::fadeOut(VoiceId voice, f32 seconds, bool stopWhenDone)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    const u64 duration = milliseconds(seconds);
    if (stopWhenDone)
    {
        if (ma_sound_stop_with_fade_in_milliseconds(&found->player, duration) != MA_SUCCESS)
            return false;
        found->removeWhenStopped = true;
    }
    else
    {
        ma_sound_set_fade_in_milliseconds(&found->player, -1.0f, 0.0f, duration);
    }
    return true;
}

AudioEngine::VoiceId AudioEngine::crossfadeMusic(SoundId sound, bool loop, f32 volume,
                                                 f32 seconds)
{
    Impl::Voice* next = createMusicVoice(mImpl, sound, loop, volume);
    if (!next)
        return 0;
    const VoiceId previous = mImpl->musicVoice;
    mImpl->musicVoice = next->id;
    fadeIn(next->id, seconds);
    if (previous)
        fadeOut(previous, seconds, true);
    return next->id;
}

bool AudioEngine::setListenerPosition(const glm::vec3& position)
{
    if (!mImpl || !mImpl->ready)
        return false;
    ma_engine_listener_set_position(&mImpl->engine, 0, position.x, position.y, position.z);
    return true;
}

bool AudioEngine::setListenerOrientation(const glm::vec3& forward, const glm::vec3& up)
{
    if (!mImpl || !mImpl->ready)
        return false;
    ma_engine_listener_set_direction(&mImpl->engine, 0, forward.x, forward.y, forward.z);
    ma_engine_listener_set_world_up(&mImpl->engine, 0, up.x, up.y, up.z);
    return true;
}

bool AudioEngine::setListenerVelocity(const glm::vec3& velocity)
{
    if (!mImpl || !mImpl->ready)
        return false;
    ma_engine_listener_set_velocity(&mImpl->engine, 0, velocity.x, velocity.y, velocity.z);
    return true;
}

bool AudioEngine::setVoicePosition(VoiceId voice, const glm::vec3& position)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    ma_sound_set_position(&found->player, position.x, position.y, position.z);
    return true;
}

bool AudioEngine::setVoiceVelocity(VoiceId voice, const glm::vec3& velocity)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    ma_sound_set_velocity(&found->player, velocity.x, velocity.y, velocity.z);
    return true;
}

bool AudioEngine::setVoiceSpatial(VoiceId voice, bool enabled, f32 minDistance, f32 maxDistance,
                                  f32 rolloff)
{
    Impl::Voice* found = findVoice(mImpl, voice);
    if (!found)
        return false;
    minDistance = clampValue(minDistance, 0.0f, 1000000.0f);
    maxDistance = clampValue(maxDistance, minDistance, 1000000.0f);
    ma_sound_set_spatialization_enabled(&found->player, enabled ? MA_TRUE : MA_FALSE);
    ma_sound_set_positioning(&found->player, ma_positioning_absolute);
    ma_sound_set_attenuation_model(&found->player, ma_attenuation_model_inverse);
    ma_sound_set_min_distance(&found->player, minDistance);
    ma_sound_set_max_distance(&found->player, maxDistance);
    ma_sound_set_rolloff(&found->player, clampValue(rolloff, 0.0f, 100.0f));
    return true;
}

AudioEngine::VoiceId AudioEngine::playAt(SoundId sound, const glm::vec3& position, f32 volume,
                                         f32 pitch, f32 minDistance, f32 maxDistance,
                                         f32 rolloff)
{
    const VoiceId voice = play(sound, volume, pitch);
    if (!voice)
        return 0;
    setVoicePosition(voice, position);
    setVoiceSpatial(voice, true, minDistance, maxDistance, rolloff);
    return voice;
}

void AudioEngine::stopAll()
{
    if (!mImpl)
        return;
    for (Impl::Voice* voice : mImpl->voices)
        destroyVoice(mImpl, voice);
    mImpl->voices.clear();
    mImpl->musicVoice = 0;
}

void AudioEngine::stopMusic()
{
    if (mImpl && mImpl->musicVoice)
        stop(mImpl->musicVoice);
}

void AudioEngine::setMasterVolume(f32 volume)
{
    if (!mImpl)
        return;
    mImpl->masterVolume = clampValue(volume, 0.0f, 4.0f);
    applyMixer(mImpl);
}

void AudioEngine::setSfxVolume(f32 volume)
{
    if (!mImpl)
        return;
    mImpl->sfxVolume = clampValue(volume, 0.0f, 4.0f);
    applyMixer(mImpl);
}

void AudioEngine::setMusicVolume(f32 volume)
{
    if (!mImpl)
        return;
    mImpl->musicVolume = clampValue(volume, 0.0f, 4.0f);
    applyMixer(mImpl);
}

f32 AudioEngine::masterVolume() const
{
    return mImpl ? mImpl->masterVolume : 0.0f;
}

f32 AudioEngine::sfxVolume() const
{
    return mImpl ? mImpl->sfxVolume : 0.0f;
}

f32 AudioEngine::musicVolume() const
{
    return mImpl ? mImpl->musicVolume : 0.0f;
}

void AudioEngine::setMasterMuted(bool muted)
{
    if (!mImpl)
        return;
    mImpl->masterMuted = muted;
    applyMixer(mImpl);
}

void AudioEngine::setSfxMuted(bool muted)
{
    if (!mImpl)
        return;
    mImpl->sfxMuted = muted;
    applyMixer(mImpl);
}

void AudioEngine::setMusicMuted(bool muted)
{
    if (!mImpl)
        return;
    mImpl->musicMuted = muted;
    applyMixer(mImpl);
}

bool AudioEngine::masterMuted() const
{
    return mImpl && mImpl->masterMuted;
}

bool AudioEngine::sfxMuted() const
{
    return mImpl && mImpl->sfxMuted;
}

bool AudioEngine::musicMuted() const
{
    return mImpl && mImpl->musicMuted;
}

u32 AudioEngine::voiceCount() const
{
    return mImpl ? static_cast<u32>(mImpl->voices.size()) : 0;
}

u32 AudioEngine::soundCount() const
{
    return mImpl ? static_cast<u32>(mImpl->sounds.size()) : 0;
}

void AudioEngine::update()
{
    if (!mImpl)
        return;
    for (auto it = mImpl->voices.begin(); it != mImpl->voices.end();)
    {
        Impl::Voice* voice = *it;
        if (!voice || (!voice->loop && ma_sound_at_end(&voice->player) == MA_TRUE) ||
            (voice->removeWhenStopped && !voice->paused &&
             ma_sound_is_playing(&voice->player) != MA_TRUE))
        {
            destroyVoice(mImpl, voice);
            it = mImpl->voices.erase(it);
        }
        else
            ++it;
    }
}

} // namespace Radion

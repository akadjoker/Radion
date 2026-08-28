/* miniaudio + stb_vorbis implementation, compiled as C.
 *
 * Kept out of AudioEngine.cpp on purpose: building the implementation as C
 * gives every miniaudio function C linkage. AudioEngine.cpp includes
 * miniaudio.h for its declarations only.
 */
#include "stb_vorbis.c"

/* stb_vorbis uses short, generic macro names for its internal channel map.
 * They have done their job once the implementation above has been included,
 * and some of them also occur in platform headers miniaudio includes below.
 */
#undef PLAYBACK_MONO
#undef PLAYBACK_LEFT
#undef PLAYBACK_RIGHT
#undef L
#undef C
#undef R

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

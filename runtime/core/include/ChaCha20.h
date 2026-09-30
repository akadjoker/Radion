#ifndef RADION_CHACHA20_H
#define RADION_CHACHA20_H

#include "Types.h"

#include <string>

namespace Radion
{

// ChaCha20 per RFC 8439; encrypting and decrypting are the same call.
// Obfuscation only: a shipped build carries its key, so this stops casual pack opening, not an attacker.
class ChaCha20
{
public:
    static constexpr usize KeySize = 32;
    static constexpr usize NonceSize = 12;
    static constexpr usize BlockSize = 64;
    static constexpr usize SaltSize = 16;

    ChaCha20();

    void setKey(const u8 key[KeySize]);
    void setNonce(const u8 nonce[NonceSize], u32 counter);

    void process(u8* data, usize size);

    // Iterated permutation over the salt, not a memory-hard KDF.
    static void deriveKey(const std::string& passphrase, const u8 salt[SaltSize],
                          u8 key[KeySize]);

private:
    static void block(const u32 state[16], u8 out[BlockSize]);

    u32 mState[16];
};

} // namespace Radion

#endif // RADION_CHACHA20_H

#ifndef RADION_CHACHA20_H
#define RADION_CHACHA20_H

#include "Types.h"

#include <string>

namespace Radion
{

// ChaCha20 as specified in RFC 8439: a 32-byte key and a 12-byte nonce select
// a keystream, which process() XORs into the caller's bytes. Encrypting and
// decrypting are the same call.
//
// What this buys, and what it does not: a shipped build carries the key it
// uses, so anyone willing to read the binary can recover it. This raises the
// cost of opening a pack from "rename it to .zip" to "reverse the loader",
// and nothing beyond that. It is not protection against an attacker.
class ChaCha20
{
public:
    static constexpr usize KeySize = 32;
    static constexpr usize NonceSize = 12;
    static constexpr usize BlockSize = 64;
    static constexpr usize SaltSize = 16;

    ChaCha20();

    void setKey(const u8 key[KeySize]);
    // `counter` is the block index the next process() call starts from.
    void setNonce(const u8 nonce[NonceSize], u32 counter);

    // XORs the keystream into `data` in place, advancing the block counter.
    void process(u8* data, usize size);

    // Stretches a passphrase of any length into a key, iterating the ChaCha20
    // permutation over the salt. An iterated permutation, not a memory-hard
    // KDF: it makes a short passphrase cost something to guess offline, and
    // makes two packs built from the same passphrase carry different keys.
    static void deriveKey(const std::string& passphrase, const u8 salt[SaltSize],
                          u8 key[KeySize]);

private:
    static void block(const u32 state[16], u8 out[BlockSize]);

    u32 mState[16];
};

} // namespace Radion

#endif // RADION_CHACHA20_H

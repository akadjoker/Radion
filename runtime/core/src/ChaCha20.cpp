#include "PCH.h"

#include "ChaCha20.h"

#include <cstring>

namespace Radion
{

namespace
{

constexpr u32 kDeriveRounds = 4096;

u32 rotateLeft(u32 value, int bits)
{
    return (value << bits) | (value >> (32 - bits));
}

void quarterRound(u32& a, u32& b, u32& c, u32& d)
{
    a += b;
    d = rotateLeft(d ^ a, 16);
    c += d;
    b = rotateLeft(b ^ c, 12);
    a += b;
    d = rotateLeft(d ^ a, 8);
    c += d;
    b = rotateLeft(b ^ c, 7);
}

u32 readLE32(const u8* bytes)
{
    return static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) |
           (static_cast<u32>(bytes[2]) << 16) | (static_cast<u32>(bytes[3]) << 24);
}

void writeLE32(u8* bytes, u32 value)
{
    bytes[0] = static_cast<u8>(value);
    bytes[1] = static_cast<u8>(value >> 8);
    bytes[2] = static_cast<u8>(value >> 16);
    bytes[3] = static_cast<u8>(value >> 24);
}

void writeLE64(u8* bytes, u64 value)
{
    writeLE32(bytes, static_cast<u32>(value));
    writeLE32(bytes + 4, static_cast<u32>(value >> 32));
}

u64 foldPassphrase(const std::string& passphrase, u64 seed)
{
    u64 hash = 14695981039346656037ull ^ seed;
    for (usize i = 0; i < passphrase.size(); ++i)
    {
        hash ^= static_cast<u8>(passphrase[i]);
        hash *= 1099511628211ull;
    }
    // One more pass over the length, so trailing zero bytes in a passphrase
    // cannot produce the same lane value as a shorter one.
    hash ^= passphrase.size();
    hash *= 1099511628211ull;
    return hash;
}

} // namespace

ChaCha20::ChaCha20()
{
    std::memset(mState, 0, sizeof(mState));
}

void ChaCha20::setKey(const u8 key[KeySize])
{
    mState[0] = 0x61707865u;
    mState[1] = 0x3320646eu;
    mState[2] = 0x79622d32u;
    mState[3] = 0x6b206574u;
    for (u32 i = 0; i < 8; ++i)
        mState[4 + i] = readLE32(key + i * 4);
}

void ChaCha20::setNonce(const u8 nonce[NonceSize], u32 counter)
{
    mState[12] = counter;
    mState[13] = readLE32(nonce);
    mState[14] = readLE32(nonce + 4);
    mState[15] = readLE32(nonce + 8);
}

void ChaCha20::block(const u32 state[16], u8 out[BlockSize])
{
    u32 working[16];
    for (u32 i = 0; i < 16; ++i)
        working[i] = state[i];

    for (u32 i = 0; i < 10; ++i)
    {
        quarterRound(working[0], working[4], working[8], working[12]);
        quarterRound(working[1], working[5], working[9], working[13]);
        quarterRound(working[2], working[6], working[10], working[14]);
        quarterRound(working[3], working[7], working[11], working[15]);
        quarterRound(working[0], working[5], working[10], working[15]);
        quarterRound(working[1], working[6], working[11], working[12]);
        quarterRound(working[2], working[7], working[8], working[13]);
        quarterRound(working[3], working[4], working[9], working[14]);
    }

    for (u32 i = 0; i < 16; ++i)
        writeLE32(out + i * 4, working[i] + state[i]);
}

void ChaCha20::process(u8* data, usize size)
{
    u8 keystream[BlockSize];
    usize done = 0;
    while (done < size)
    {
        block(mState, keystream);
        ++mState[12];

        const usize chunk = size - done < BlockSize ? size - done : BlockSize;
        for (usize i = 0; i < chunk; ++i)
            data[done + i] ^= keystream[i];
        done += chunk;
    }
}

void ChaCha20::deriveKey(const std::string& passphrase, const u8 salt[SaltSize], u8 key[KeySize])
{
    u8 seed[KeySize];
    for (u32 lane = 0; lane < 4; ++lane)
        writeLE64(seed + lane * 8, foldPassphrase(passphrase, lane * 0x9e3779b97f4a7c15ull));

    u8 nonce[NonceSize];
    std::memcpy(nonce, salt, NonceSize);

    ChaCha20 stage;
    u8 keystream[BlockSize];
    for (u32 round = 0; round < kDeriveRounds; ++round)
    {
        stage.setKey(seed);
        stage.setNonce(nonce, round);
        std::memset(keystream, 0, sizeof(keystream));
        stage.process(keystream, sizeof(keystream));
        std::memcpy(seed, keystream, KeySize);
        // The nonce only carries 12 of the salt's 16 bytes; folding the whole
        // salt back in every round is what makes the last four matter.
        for (usize i = 0; i < SaltSize; ++i)
            seed[i] ^= salt[i];
    }

    std::memcpy(key, seed, KeySize);
}

} // namespace Radion

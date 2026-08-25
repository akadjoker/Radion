#include "PCH.h"

#include "ChaCha20.h"
#include "FilePack.h"
#include "FileSystem.h"

#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "FilePackTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

const char* kPackPath = "filepack_tests.rpak";
const char* kKey = "a passphrase nobody would guess";

std::vector<u8> textBytes(const std::string& text)
{
    return std::vector<u8>(text.begin(), text.end());
}

// Compressible on purpose: an entry that deflates and one that does not take
// different paths through both the writer and the reader.
std::vector<u8> repeatingBytes(usize size)
{
    std::vector<u8> out(size);
    for (usize i = 0; i < size; ++i)
        out[i] = static_cast<u8>('a' + (i % 7));
    return out;
}

std::vector<u8> randomBytes(usize size, u32 seed)
{
    std::vector<u8> out(size);
    std::mt19937 generator(seed);
    for (usize i = 0; i < size; ++i)
        out[i] = static_cast<u8>(generator() & 0xffu);
    return out;
}

bool sameBytes(const ByteArray& got, const std::vector<u8>& expected)
{
    if (got.size() != expected.size())
        return false;
    return expected.empty() || std::memcmp(got.data(), expected.data(), expected.size()) == 0;
}

// ------------------------------------------------------------------ ChaCha20

// RFC 8439 section 2.4.2: the one published vector this implementation has to
// reproduce byte for byte, or nothing written with it can be read back.
void testChaChaMatchesRfc8439()
{
    u8 key[ChaCha20::KeySize];
    for (u32 i = 0; i < ChaCha20::KeySize; ++i)
        key[i] = static_cast<u8>(i);

    const u8 nonce[ChaCha20::NonceSize] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x4a, 0x00, 0x00, 0x00, 0x00};

    const char* plainText = "Ladies and Gentlemen of the class of '99: If I could offer you only "
                            "one tip for the future, sunscreen would be it.";
    const usize length = std::strlen(plainText);

    std::vector<u8> data(plainText, plainText + length);
    ChaCha20 stream;
    stream.setKey(key);
    stream.setNonce(nonce, 1);
    stream.process(data.data(), data.size());

    const u8 expectedHead[16] = {0x6e, 0x2e, 0x35, 0x9a, 0x25, 0x68, 0xf9, 0x80,
                                 0x41, 0xba, 0x07, 0x28, 0xdd, 0x0d, 0x69, 0x81};
    CHECK(std::memcmp(data.data(), expectedHead, sizeof(expectedHead)) == 0);

    stream.setKey(key);
    stream.setNonce(nonce, 1);
    stream.process(data.data(), data.size());
    CHECK(std::memcmp(data.data(), plainText, length) == 0);
}

// The vector above only reaches into the second block; this pins down how the
// counter gets there. Running one buffer straight through has to equal running
// it in block-sized pieces with the counter set by hand - if process() failed
// to advance, or advanced by anything other than one block, the two diverge
// from byte 64 on while still round-tripping perfectly against itself.
void testCounterAdvancesOneBlockAtATime()
{
    u8 key[ChaCha20::KeySize];
    for (u32 i = 0; i < ChaCha20::KeySize; ++i)
        key[i] = static_cast<u8>(i * 7 + 1);
    u8 nonce[ChaCha20::NonceSize];
    for (u32 i = 0; i < ChaCha20::NonceSize; ++i)
        nonce[i] = static_cast<u8>(i * 3);

    const usize length = ChaCha20::BlockSize * 3 + 17;
    std::vector<u8> whole(length, 0);
    std::vector<u8> pieces(length, 0);

    ChaCha20 oneShot;
    oneShot.setKey(key);
    oneShot.setNonce(nonce, 5);
    oneShot.process(whole.data(), whole.size());

    usize done = 0;
    u32 counter = 5;
    while (done < length)
    {
        const usize chunk =
            length - done < ChaCha20::BlockSize ? length - done : ChaCha20::BlockSize;
        ChaCha20 step;
        step.setKey(key);
        step.setNonce(nonce, counter);
        step.process(pieces.data() + done, chunk);
        done += chunk;
        ++counter;
    }

    CHECK(std::memcmp(whole.data(), pieces.data(), length) == 0);
    // And the keystream is not simply repeating every block.
    CHECK(std::memcmp(whole.data(), whole.data() + ChaCha20::BlockSize, ChaCha20::BlockSize) != 0);
}

void testDeriveKeyDependsOnSaltAndPassphrase()
{
    u8 saltA[ChaCha20::SaltSize];
    u8 saltB[ChaCha20::SaltSize];
    for (u32 i = 0; i < ChaCha20::SaltSize; ++i)
    {
        saltA[i] = static_cast<u8>(i);
        saltB[i] = static_cast<u8>(i);
    }
    // Only the last byte differs, and it is one of the four the nonce cannot
    // carry - the round-by-round fold is what has to make it count.
    saltB[ChaCha20::SaltSize - 1] ^= 0x01;

    u8 first[ChaCha20::KeySize];
    u8 second[ChaCha20::KeySize];
    u8 third[ChaCha20::KeySize];
    ChaCha20::deriveKey("hunter2", saltA, first);
    ChaCha20::deriveKey("hunter2", saltA, second);
    ChaCha20::deriveKey("hunter2", saltB, third);

    CHECK(std::memcmp(first, second, ChaCha20::KeySize) == 0);
    CHECK(std::memcmp(first, third, ChaCha20::KeySize) != 0);

    u8 other[ChaCha20::KeySize];
    ChaCha20::deriveKey("hunter3", saltA, other);
    CHECK(std::memcmp(first, other, ChaCha20::KeySize) != 0);
}

// ------------------------------------------------------------------ FilePack

void writeTestPack(const std::string& key)
{
    FilePackWriter writer;
    writer.setKey(key);

    const std::vector<u8> shader = textBytes("#version 330 core\nvoid main() {}\n");
    const std::vector<u8> compressible = repeatingBytes(64 * 1024);
    const std::vector<u8> incompressible = randomBytes(4096, 1234);

    CHECK(writer.addData("lit.frag", shader.data(), shader.size()));
    CHECK(writer.addData("lensflare/flare_0_halo.png", compressible.data(), compressible.size()));
    CHECK(writer.addData("noise.bin", incompressible.data(), incompressible.size()));
    CHECK(writer.addData("empty.txt", nullptr, 0));

    // A name added twice could never be read back, so the writer refuses it.
    CHECK(!writer.addData("lit.frag", shader.data(), shader.size()));

    CHECK(writer.entryCount() == 4);
    CHECK(writer.write(kPackPath));
}

void testRoundTrip(const std::string& key)
{
    writeTestPack(key);

    FilePack pack;
    CHECK(pack.open(kPackPath, key));
    CHECK(pack.entryCount() == 4);

    CHECK(pack.exists("lit.frag"));
    CHECK(pack.exists("lensflare/flare_0_halo.png"));
    // Backslashes are normalized on both sides, the way ZipArchive does it.
    CHECK(pack.exists("lensflare\\flare_0_halo.png"));
    CHECK(!pack.exists("missing.frag"));

    CHECK(sameBytes(pack.readBinary("lit.frag"),
                    textBytes("#version 330 core\nvoid main() {}\n")));
    CHECK(sameBytes(pack.readBinary("lensflare/flare_0_halo.png"), repeatingBytes(64 * 1024)));
    CHECK(sameBytes(pack.readBinary("noise.bin"), randomBytes(4096, 1234)));
    CHECK(pack.readBinary("empty.txt").size() == 0);
    CHECK(pack.readBinary("missing.frag").size() == 0);

    // Reading out of order, and twice, has to give the same bytes: one file
    // cursor is shared and every read seeks it.
    CHECK(sameBytes(pack.readBinary("noise.bin"), randomBytes(4096, 1234)));
    CHECK(sameBytes(pack.readBinary("lit.frag"),
                    textBytes("#version 330 core\nvoid main() {}\n")));
}

void testCompressionActuallyRuns()
{
    writeTestPack(std::string());

    FilePack pack;
    CHECK(pack.open(kPackPath, std::string()));

    for (u32 i = 0; i < pack.entryCount(); ++i)
    {
        if (pack.entryName(i) == "lensflare/flare_0_halo.png")
        {
            CHECK(pack.entrySizeRaw(i) == 64 * 1024);
            CHECK(pack.entrySizeStored(i) < pack.entrySizeRaw(i) / 4);
        }
        // Random bytes do not deflate; the writer must store them raw rather
        // than pay deflate's overhead for a larger result.
        if (pack.entryName(i) == "noise.bin")
            CHECK(pack.entrySizeStored(i) == pack.entrySizeRaw(i));
    }
}

void testWrongKeyIsRejected()
{
    writeTestPack(kKey);

    FilePack wrong;
    CHECK(!wrong.open(kPackPath, "not the key"));
    CHECK(!wrong.isOpen());

    FilePack none;
    CHECK(!none.open(kPackPath, std::string()));

    FilePack right;
    CHECK(right.open(kPackPath, kKey));
}

// The point of the key is that the bytes on disk do not read as themselves.
void testEncryptedPackHidesItsContents()
{
    writeTestPack(kKey);

    std::FILE* file = std::fopen(kPackPath, "rb");
    CHECK(file != nullptr);
    if (!file)
        return;

    std::fseek(file, 0, SEEK_END);
    const long length = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    std::vector<u8> bytes(static_cast<usize>(length));
    CHECK(std::fread(bytes.data(), 1, bytes.size(), file) == bytes.size());
    std::fclose(file);

    const std::string haystack(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    CHECK(haystack.find("lit.frag") == std::string::npos);
    CHECK(haystack.find("#version 330") == std::string::npos);
    // Only the header stays readable, and it must, to find the directory.
    CHECK(haystack.compare(0, 4, "RPAK") == 0);
}

void testCorruptEntryIsCaught()
{
    writeTestPack(std::string());

    std::FILE* file = std::fopen(kPackPath, "r+b");
    CHECK(file != nullptr);
    if (!file)
        return;
    // Straight past the header, into the first entry's stored bytes.
    std::fseek(file, static_cast<long>(FilePack::HeaderSize), SEEK_SET);
    const u8 garbage = 0x00;
    std::fwrite(&garbage, 1, 1, file);
    std::fclose(file);

    FilePack pack;
    CHECK(pack.open(kPackPath, std::string()));
    CHECK(pack.readBinary("lit.frag").size() == 0);
    // Every other entry still reads: one bad entry is not a bad pack.
    CHECK(sameBytes(pack.readBinary("noise.bin"), randomBytes(4096, 1234)));
}

void testNotAPack()
{
    std::FILE* file = std::fopen(kPackPath, "wb");
    CHECK(file != nullptr);
    if (!file)
        return;
    const char* junk = "this is not a pack, not even close";
    std::fwrite(junk, 1, std::strlen(junk), file);
    std::fclose(file);

    FilePack pack;
    CHECK(!pack.open(kPackPath, std::string()));
    CHECK(!pack.open("no_such_file_at_all.rpak", std::string()));
}

// What the engine actually does with a pack: mount it and ask FileSystem for
// a bare name, the same call every shader and texture load already makes.
void testMountedThroughFileSystem()
{
    writeTestPack(kKey);

    FileSystem files;
    CHECK(!files.mountPack(kPackPath, "wrong"));
    CHECK(files.mountPack(kPackPath, kKey));

    CHECK(files.exists("lit.frag"));
    CHECK(!files.exists("nothing_here.frag"));
    CHECK(files.readText("lit.frag") == "#version 330 core\nvoid main() {}\n");
    CHECK(sameBytes(files.readBinary("lensflare/flare_0_halo.png"), repeatingBytes(64 * 1024)));

    // A name in no archive still falls through to the search paths.
    CHECK(!files.exists("still_not_here.frag"));

    files.unmountAll();
    CHECK(!files.exists("lit.frag"));
}

// A pack beats a loose file of the same name - the decision that lets a
// shipped build ignore whatever someone leaves in the folder next to it.
void testPackWinsOverDisk()
{
    const char* looseName = "filepack_tests_loose.txt";

    FileSystem files;
    files.addSearchPath(".");
    ByteArray onDisk(9);
    std::memcpy(onDisk.data(), "from disk", 9);
    CHECK(files.writeBinary(looseName, onDisk));
    CHECK(files.readText(looseName) == "from disk");

    FilePackWriter writer;
    const std::vector<u8> packed = textBytes("from pack");
    CHECK(writer.addData(looseName, packed.data(), packed.size()));
    CHECK(writer.write(kPackPath));

    CHECK(files.mountPack(kPackPath, std::string()));
    CHECK(files.readText(looseName) == "from pack");

    files.unmountAll();
    CHECK(files.readText(looseName) == "from disk");
    files.removeFile(looseName);
}

std::vector<u8> readWholeFile(const char* path)
{
    std::vector<u8> bytes;
    std::FILE* file = std::fopen(path, "rb");
    if (!file)
        return bytes;
    std::fseek(file, 0, SEEK_END);
    const long length = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    bytes.resize(static_cast<usize>(length));
    if (!bytes.empty() && std::fread(bytes.data(), 1, bytes.size(), file) != bytes.size())
        bytes.clear();
    std::fclose(file);
    return bytes;
}

// The embedded case: the same pack, read straight out of a byte array with no
// file behind it at all.
void testOpenFromMemory()
{
    writeTestPack(kKey);
    const std::vector<u8> bytes = readWholeFile(kPackPath);
    CHECK(!bytes.empty());

    FilePack pack;
    CHECK(pack.openFromMemory(bytes.data(), bytes.size(), kKey));
    CHECK(pack.isOpen());
    CHECK(pack.entryCount() == 4);
    CHECK(sameBytes(pack.readBinary("lit.frag"),
                    textBytes("#version 330 core\nvoid main() {}\n")));
    CHECK(sameBytes(pack.readBinary("lensflare/flare_0_halo.png"), repeatingBytes(64 * 1024)));
    CHECK(pack.readBinary("missing.frag").size() == 0);

    FilePack wrong;
    CHECK(!wrong.openFromMemory(bytes.data(), bytes.size(), "not the key"));

    FilePack truncated;
    CHECK(!truncated.openFromMemory(bytes.data(), 8, kKey));
    CHECK(!truncated.openFromMemory(nullptr, 0, kKey));
}

// The whole point of the embedded pack being a fallback rather than an
// override: a file on disk still beats it, so editing a shader in the assets
// folder keeps working on a build that carries its own copy.
void testDiskWinsOverFallback()
{
    const char* looseName = "filepack_tests_fallback.txt";

    FilePackWriter writer;
    const std::vector<u8> packed = textBytes("from fallback");
    CHECK(writer.addData(looseName, packed.data(), packed.size()));
    CHECK(writer.addData("only_in_fallback.txt", packed.data(), packed.size()));
    CHECK(writer.write(kPackPath));
    const std::vector<u8> bytes = readWholeFile(kPackPath);

    FileSystem files;
    files.addSearchPath(".");
    CHECK(files.mountFallbackPack(bytes.data(), bytes.size(), std::string()));

    // Nothing on disk yet, so the fallback answers.
    CHECK(files.readText(looseName) == "from fallback");
    CHECK(files.exists(looseName));

    ByteArray onDisk(9);
    std::memcpy(onDisk.data(), "from disk", 9);
    CHECK(files.writeBinary(looseName, onDisk));
    CHECK(files.readText(looseName) == "from disk");

    // A name the disk does not have still comes from the fallback.
    CHECK(files.readText("only_in_fallback.txt") == "from fallback");
    CHECK(!files.exists("in_neither.txt"));

    // And a mounted pack outranks both, the other direction entirely.
    FilePackWriter override;
    const std::vector<u8> fromPack = textBytes("from pack");
    CHECK(override.addData(looseName, fromPack.data(), fromPack.size()));
    CHECK(override.write("filepack_tests_override.rpak"));
    CHECK(files.mountPack("filepack_tests_override.rpak", std::string()));
    CHECK(files.readText(looseName) == "from pack");

    files.removeFile(looseName);
    files.unmountAll();
    std::remove("filepack_tests_override.rpak");
}

} // namespace

int main()
{
    testChaChaMatchesRfc8439();
    testCounterAdvancesOneBlockAtATime();
    testDeriveKeyDependsOnSaltAndPassphrase();
    testRoundTrip(std::string());
    testRoundTrip(kKey);
    testCompressionActuallyRuns();
    testWrongKeyIsRejected();
    testEncryptedPackHidesItsContents();
    testCorruptEntryIsCaught();
    testNotAPack();
    testMountedThroughFileSystem();
    testPackWinsOverDisk();
    testOpenFromMemory();
    testDiskWinsOverFallback();

    std::remove(kPackPath);

    if (gFailures)
        std::fprintf(stderr, "%d file pack test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}

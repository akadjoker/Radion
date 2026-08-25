#include "FilePack.h"
#include "FileSystem.h"
#include "Log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace Radion;

namespace
{

struct Root
{
    std::string path;
    std::string prefix;
};

struct Options
{
    std::string output;
    std::string source;
    std::string key;
    std::vector<Root> roots;
    int level;
    bool list;
    bool verbose;
};

void usage()
{
    std::printf("radion_pack - builds and inspects .rpak archives\n"
                "\n"
                "  radion_pack -o <out.rpak> [options] <path>...\n"
                "  radion_pack -t <pack.rpak> [-k <key>]\n"
                "\n"
                "Each <path> is a root: files under it are named relative to it, so\n"
                "  radion_pack -o shaders.rpak assets/shaders\n"
                "stores lit.frag - the same name the engine asks FileSystem for once\n"
                "that folder is a search path. -p puts a folder back in front of the\n"
                "names that follow, for a root packed from deeper than it is asked for:\n"
                "  radion_pack -o d.rpak assets/shaders -p lensflare assets/textures/lensflare\n"
                "\n"
                "  -o <file>   pack to write\n"
                "  -t <file>   list an existing pack instead of writing one\n"
                "  -c <file>   also write the pack as a C++ source to compile in\n"
                "  -k <key>    encrypt with this key (omit for a plain pack)\n"
                "  -p <name>   prefix the names of every root after this one\n"
                "  -l <0-9>    deflate level, default 9; 0 stores everything raw\n"
                "  -v          print every entry as it is added\n");
}

bool parseArguments(int argc, char** argv, Options& options)
{
    options.level = 9;
    options.list = false;
    options.verbose = false;

    std::string prefix;
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "-o" && i + 1 < argc)
            options.output = argv[++i];
        else if (argument == "-t" && i + 1 < argc)
        {
            options.output = argv[++i];
            options.list = true;
        }
        else if (argument == "-c" && i + 1 < argc)
            options.source = argv[++i];
        else if (argument == "-k" && i + 1 < argc)
            options.key = argv[++i];
        else if (argument == "-p" && i + 1 < argc)
            prefix = argv[++i];
        else if (argument == "-l" && i + 1 < argc)
            options.level = std::atoi(argv[++i]);
        else if (argument == "-v")
            options.verbose = true;
        else if (argument == "-h" || argument == "--help")
            return false;
        else if (!argument.empty() && argument[0] == '-')
        {
            std::printf("radion_pack: unknown option %s\n", argument.c_str());
            return false;
        }
        else
        {
            Root root;
            root.path = argument;
            root.prefix = prefix;
            options.roots.push_back(root);
        }
    }

    if (options.output.empty())
        return false;
    if (!options.list && options.roots.empty())
        return false;
    return true;
}

std::string joinRelative(const std::string& prefix, const std::string& name)
{
    if (prefix.empty())
        return name;
    return prefix + "/" + name;
}

// `relative` is where the walk has reached inside `root`; `prefix` is what
// goes in front of the stored name and never touches the path on disk.
bool addDirectory(FilePackWriter& writer, const FileSystem& files, const std::string& root,
                  const std::string& prefix, const std::string& relative, bool verbose, u32& added)
{
    const std::string directory = relative.empty() ? root : root + "/" + relative;
    std::vector<FileSystem::DirEntry> entries = files.listDirectory(directory);

    for (usize i = 0; i < entries.size(); ++i)
    {
        const FileSystem::DirEntry& entry = entries[i];
        if (entry.name == "." || entry.name == "..")
            continue;

        const std::string child = joinRelative(relative, entry.name);
        if (entry.isDirectory)
        {
            if (!addDirectory(writer, files, root, prefix, child, verbose, added))
                return false;
            continue;
        }

        const std::string name = joinRelative(prefix, child);
        if (!writer.addFile(name, root + "/" + child))
            return false;
        ++added;
        if (verbose)
            std::printf("  %s (%llu bytes)\n", name.c_str(),
                        static_cast<unsigned long long>(entry.size));
    }

    return true;
}

// The pack, byte for byte, as something the compiler will put in .rodata.
// FilePack::openFromMemory() reads it in place - it is never copied, never
// decompressed as a whole, and costs nothing at startup beyond its directory.
bool writeSource(const std::string& packPath, const std::string& sourcePath,
                 const std::string& key)
{
    std::FILE* input = std::fopen(packPath.c_str(), "rb");
    if (!input)
    {
        std::printf("radion_pack: could not reopen %s\n", packPath.c_str());
        return false;
    }

    std::fseek(input, 0, SEEK_END);
    const long length = std::ftell(input);
    std::fseek(input, 0, SEEK_SET);

    std::vector<unsigned char> bytes(static_cast<usize>(length > 0 ? length : 0));
    if (!bytes.empty() && std::fread(bytes.data(), 1, bytes.size(), input) != bytes.size())
    {
        std::printf("radion_pack: could not read back %s\n", packPath.c_str());
        std::fclose(input);
        return false;
    }
    std::fclose(input);

    std::FILE* output = std::fopen(sourcePath.c_str(), "wb");
    if (!output)
    {
        std::printf("radion_pack: could not create %s\n", sourcePath.c_str());
        return false;
    }

    std::fprintf(output, "// Generated by radion_pack from %s. Do not edit.\n\n",
                 packPath.c_str());
    std::fprintf(output, "namespace Radion\n{\nnamespace Generated\n{\n\n");
    std::fprintf(output, "extern const unsigned char kDefaultPackData[];\n");
    std::fprintf(output, "extern const unsigned long long kDefaultPackSize;\n");
    std::fprintf(output, "extern const char* const kDefaultPackKey;\n\n");

    std::fprintf(output, "const unsigned char kDefaultPackData[] = {");
    for (usize i = 0; i < bytes.size(); ++i)
    {
        if (i % 16 == 0)
            std::fprintf(output, "\n    ");
        std::fprintf(output, "0x%02x,", bytes[i]);
    }
    std::fprintf(output, "\n};\n\n");

    std::fprintf(output, "const unsigned long long kDefaultPackSize = %lluull;\n",
                 static_cast<unsigned long long>(bytes.size()));

    std::fprintf(output, "const char* const kDefaultPackKey = \"");
    for (usize i = 0; i < key.size(); ++i)
        std::fprintf(output, "\\x%02x", static_cast<unsigned char>(key[i]));
    std::fprintf(output, "\";\n\n");

    std::fprintf(output, "} // namespace Generated\n} // namespace Radion\n");
    std::fclose(output);

    std::printf("%s: %llu bytes embedded\n", sourcePath.c_str(),
                static_cast<unsigned long long>(bytes.size()));
    return true;
}

int listPack(const Options& options)
{
    FilePack pack;
    if (!pack.open(options.output, options.key))
        return 1;

    u64 raw = 0;
    u64 stored = 0;
    for (u32 i = 0; i < pack.entryCount(); ++i)
    {
        raw += pack.entrySizeRaw(i);
        stored += pack.entrySizeStored(i);
        std::printf("%10u  %10u  %s\n", pack.entrySizeRaw(i), pack.entrySizeStored(i),
                    pack.entryName(i).c_str());
    }

    std::printf("%u entries, %llu bytes stored from %llu\n", pack.entryCount(),
                static_cast<unsigned long long>(stored), static_cast<unsigned long long>(raw));
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parseArguments(argc, argv, options))
    {
        usage();
        return 1;
    }

    if (options.list)
        return listPack(options);

    FileSystem files;
    FilePackWriter writer;
    writer.setKey(options.key);
    writer.setCompressionLevel(options.level);

    u32 added = 0;
    for (usize i = 0; i < options.roots.size(); ++i)
    {
        const Root& root = options.roots[i];
        if (files.isDirectory(root.path))
        {
            if (!addDirectory(writer, files, root.path, root.prefix, std::string(),
                              options.verbose, added))
                return 1;
        }
        else
        {
            const std::string name = joinRelative(root.prefix, FileSystem::fileName(root.path));
            if (!writer.addFile(name, root.path))
                return 1;
            ++added;
            if (options.verbose)
                std::printf("  %s\n", name.c_str());
        }
    }

    if (!writer.write(options.output))
        return 1;

    if (!options.source.empty() && !writeSource(options.output, options.source, options.key))
        return 1;

    std::printf("%s: %u entries, %llu bytes from %llu (%.1f%%)%s\n", options.output.c_str(), added,
                static_cast<unsigned long long>(writer.storedBytes()),
                static_cast<unsigned long long>(writer.rawBytes()),
                writer.rawBytes() ? 100.0 * static_cast<double>(writer.storedBytes()) /
                                        static_cast<double>(writer.rawBytes())
                                  : 0.0,
                options.key.empty() ? "" : ", encrypted");
    return 0;
}

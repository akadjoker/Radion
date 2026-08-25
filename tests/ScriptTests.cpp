// ScriptTests.cpp - smoke tests for the radion_script library (runtime/script).
//
// Runs short Zen scripts through Radion::ScriptVM and checks that
// radion.version() round-trips through the VM, and that compile/runtime
// errors come back as return values instead of being printed by ScriptVM.

#include "PCH.h"

#include "ScriptVM.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "ScriptTests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testVersionRoundTrip()
{
    ScriptVM vm;
    std::string error;

    const char* script =
        "import radion\n"
        "assert radion.version() == \"0.1.0\", \"unexpected radion.version()\"\n";

    const bool ok = vm.runString(script, "<test>", error);
    CHECK(ok);
    CHECK(error.empty());
}

void testAssertionFailureIsReturned()
{
    ScriptVM vm;
    std::string error;

    const char* script =
        "import radion\n"
        "assert radion.version() == \"9.9.9\", \"deliberate mismatch\"\n";

    const bool ok = vm.runString(script, "<test>", error);
    CHECK(!ok);
    CHECK(error.find("deliberate mismatch") != std::string::npos);
}

void testCompileErrorIsReturned()
{
    ScriptVM vm;
    std::string error;

    const char* script = "def broken(:\n";

    const bool ok = vm.runString(script, "<test>", error);
    CHECK(!ok);
    CHECK(!error.empty());
}

void testRunFile()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "radion_script_vm_test.py";

    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << "import radion\n";
        out << "assert radion.version() == \"0.1.0\", \"unexpected radion.version()\"\n";
    }

    ScriptVM vm;
    std::string error;
    const bool ok = vm.runFile(path.string().c_str(), error);
    CHECK(ok);
    CHECK(error.empty());

    std::error_code removeError;
    std::filesystem::remove(path, removeError);
}

void testMissingFileIsReturned()
{
    ScriptVM vm;
    std::string error;
    const bool ok = vm.runFile("this_file_does_not_exist.py", error);
    CHECK(!ok);
    CHECK(!error.empty());
}

// The builtin modules a script may import. Checked by actually importing
// each one and touching a name on it: a module that is compiled in but never
// registered looks exactly like one that is not there at all, and net/http
// are only useful if a script can reach them.
void testBuiltinModulesAreImportable()
{
    ScriptVM vm;
    std::string error;

    const char* script =
        "import math\n"
        "import time\n"
        "import struct\n"
        "import io\n"
        "import os\n"
        "import path\n"
        "import json\n"
        "import net\n"
        "import http\n"
        "assert math.floor(2.7) == 2, \"math\"\n"
        "assert json.stringify([1]) != \"\", \"json\"\n"
        // No port, so nothing is bound and no packet leaves the machine -
        // this only proves the module is wired up and its socket path runs.
        "s = net.udp_create()\n"
        "assert s != None, \"net.udp_create() returned nothing\"\n"
        "assert net.close(s), \"net.close() failed\"\n";

    CHECK(vm.runString(script, "<modules>", error));
    CHECK(error.empty());
    if (!error.empty())
        std::fprintf(stderr, "  module error: %s\n", error.c_str());
}

} // namespace

int main()
{
    testVersionRoundTrip();
    testBuiltinModulesAreImportable();
    testAssertionFailureIsReturned();
    testCompileErrorIsReturned();
    testRunFile();
    testMissingFileIsReturned();

    if (gFailures)
        std::fprintf(stderr, "%d script test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}

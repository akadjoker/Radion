#include "PCH.h"

/** ****************************************************************************
  ZipArchive implementation - thin wrapper around miniz's mz_zip_reader API.
**************************************************************************** */
#include "Log.h"
#include "ZipArchive.h"

#include <cstring>

namespace Radion
{

ZipArchive::ZipArchive() : mOpen(false)
{
    std::memset(&mZip, 0, sizeof(mZip));
}

ZipArchive::~ZipArchive()
{
    if (mOpen)
        mz_zip_reader_end(&mZip);
}

std::string ZipArchive::normalize(const std::string& name)
{
    // Zip entries always use forward slashes; miniz's locate_file already
    // does a case-insensitive match, so only the separator needs fixing up.
    std::string out = name;
    for (char& c : out)
        if (c == '\\')
            c = '/';
    return out;
}

bool ZipArchive::openFromMemory(ByteArray&& zipData)
{
    mData = std::move(zipData);

    std::memset(&mZip, 0, sizeof(mZip));
    if (!mz_zip_reader_init_mem(&mZip, mData.data(), mData.size(), 0))
    {
        Log::error("ZipArchive: mz_zip_reader_init_mem failed (%s)",
                   mz_zip_get_error_string(mz_zip_get_last_error(&mZip)));
        return false;
    }

    mOpen = true;
    return true;
}

bool ZipArchive::exists(const std::string& name) const
{
    if (!mOpen)
        return false;
    std::string n = normalize(name);
    return mz_zip_reader_locate_file(const_cast<mz_zip_archive*>(&mZip), n.c_str(), nullptr, 0) >=
           0;
}

ByteArray ZipArchive::readBinary(const std::string& name) const
{
    if (!mOpen)
        return ByteArray();

    std::string n = normalize(name);
    int index =
        mz_zip_reader_locate_file(const_cast<mz_zip_archive*>(&mZip), n.c_str(), nullptr, 0);
    if (index < 0)
        return ByteArray();

    usize size = 0;
    void* buf = mz_zip_reader_extract_to_heap(const_cast<mz_zip_archive*>(&mZip),
                                              static_cast<mz_uint>(index), &size, 0);
    if (!buf)
    {
        Log::error(
            "ZipArchive: failed to extract %s (%s)", name.c_str(),
            mz_zip_get_error_string(mz_zip_get_last_error(const_cast<mz_zip_archive*>(&mZip))));
        return ByteArray();
    }

    // miniz allocates via its (default) malloc callback, matching the
    // std::malloc/std::free ownership contract ByteArray expects.
    return ByteArray(static_cast<uint8*>(buf), size, true);
}

} // namespace Radion

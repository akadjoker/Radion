
#ifndef RADION_ZIP_ARCHIVE_H
#define RADION_ZIP_ARCHIVE_H

#include "Archive.h"
#include "ByteArray.h"
#include "miniz.h"

#include <string>

namespace Radion
{

class ZipArchive : public Archive
{
public:
    ZipArchive();
    ~ZipArchive() override;

    ZipArchive(const ZipArchive&) = delete;
    ZipArchive& operator=(const ZipArchive&) = delete;

    // Parses the central directory of a zip file already loaded into memory.
    // Takes ownership of the raw zip bytes (kept around for on-demand reads).
    bool openFromMemory(ByteArray&& zipData);

    bool exists(const std::string& name) const override;
    ByteArray readBinary(const std::string& name) const override;

private:
    static std::string normalize(const std::string& name);

    ByteArray mData;
    mz_zip_archive mZip;
    bool mOpen;
};

} // namespace Radion

#endif // RADION_ZIP_ARCHIVE_H

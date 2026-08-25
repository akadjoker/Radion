#ifndef RADION_ARCHIVE_H
#define RADION_ARCHIVE_H

#include "ByteArray.h"

#include <string>

namespace Radion
{

// A read-only source of named files, searched by FileSystem before the disk.
class Archive
{
public:
    virtual ~Archive();

    virtual bool exists(const std::string& name) const = 0;
    virtual ByteArray readBinary(const std::string& name) const = 0;
};

} // namespace Radion

#endif // RADION_ARCHIVE_H

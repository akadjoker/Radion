#ifndef RADION_ASSET_PATHS_H
#define RADION_ASSET_PATHS_H

#include <SDL.h>
#include <filesystem>
#include <string>

namespace Radion
{

// Installed applications live in <release>/bin while the loose engine assets
// live in <release>/assets.  SDL gives us the executable directory even when
// an application was launched from a file manager or another working folder.
// A development build keeps its compile-time source-tree path as a fallback.
inline std::string resolveAssetDirectory(const char* developmentAssetDirectory)
{
    char* basePath = SDL_GetBasePath();
    if (basePath)
    {
        const std::filesystem::path installedAssets =
            std::filesystem::path(basePath).parent_path() / "assets";
        SDL_free(basePath);

        std::error_code error;
        if (std::filesystem::is_directory(installedAssets, error))
            return installedAssets.string();
    }
    return developmentAssetDirectory;
}

} // namespace Radion

#endif // RADION_ASSET_PATHS_H

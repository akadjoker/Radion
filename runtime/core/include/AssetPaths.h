#ifndef RADION_ASSET_PATHS_H
#define RADION_ASSET_PATHS_H

#include <SDL.h>
#include <filesystem>
#include <string>

namespace Radion
{

// Installed apps: <release>/bin, loose assets: <release>/assets. Dev builds fall back to the source-tree path.
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

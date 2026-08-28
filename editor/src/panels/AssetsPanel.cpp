#include "PCH.h"

#include "panels/AssetsPanel.h"

#include "AssetManager.h"
#include "EditorApplication.h"
#include "Engine.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Hash.h"
#include "Material.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"
#include "Pixmap.h"
#include "Prefab.h"
#include "Scene.h"
#include "SceneSerializer.h"
#include "Skeleton.h"

#include <IconsMaterialDesignIcons.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <imgui.h>
#include <imgui_internal.h>

namespace Radion
{

namespace
{
std::string extensionOf(const std::string& name)
{
    const usize dot = name.find_last_of('.');
    if (dot == std::string::npos || dot + 1 >= name.size())
        return std::string();

    std::string extension = name.substr(dot + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char character)
                   {
                       return static_cast<char>(std::tolower(character));
                   });
    return extension;
}

bool isImageAsset(const std::string& extension)
{
    return extension == "png" || extension == "jpg" || extension == "jpeg" ||
          extension == "tga" || extension == "bmp" || extension == "dds" || extension == "hdr" ||
          extension == "webp";
    // .exr left out on purpose: AssetTexture's loader does not read it, so
    // loadTexture() would just fail every frame this tried to thumbnail one.
}

bool isScriptAsset(const std::string& extension)
{
    return extension == "py";
}

// A create dialog takes a single entry name, never a path: allowing a slash
// here would make a harmless-looking "new script" action write outside the
// directory the user right-clicked.
bool isValidEntryName(const char* name)
{
    return name && name[0] != '\0' && std::strcmp(name, ".") != 0 && std::strcmp(name, "..") != 0 &&
           std::strpbrk(name, "/\\") == nullptr;
}

std::string scriptFileName(const char* name)
{
    const std::string value = name ? name : "";
    return isScriptAsset(extensionOf(value)) ? value : value + ".py";
}

std::string scriptClassName(const char* name)
{
    std::string source = name ? name : "";
    const usize dot = source.find_last_of('.');
    if (dot != std::string::npos)
        source.resize(dot);

    std::string result;
    bool capitalize = true;
    for (const char character : source)
    {
        const unsigned char value = static_cast<unsigned char>(character);
        if (std::isalnum(value) || character == '_')
        {
            if (result.empty() && std::isdigit(value))
                result = "Script";
            result.push_back(capitalize ? static_cast<char>(std::toupper(value)) : character);
            capitalize = false;
        }
        else
        {
            capitalize = true;
        }
    }
    return result.empty() ? "NewScript" : result;
}

std::string makeScriptTemplate(const char* name)
{
    return "class " + scriptClassName(name) + "(ScriptComponent):\n"
           "    def on_start(self):\n"
           "        pass\n"
           "\n"
           "    def on_update(self, dt):\n"
           "        pass\n";
}

// Only what MeshLoader actually has an importer registered for
// (AssetManager.cpp's importer list) - .dae has no importer at all, so it
// stays a plain file icon rather than offering an Import that would just
// fail.
bool isMeshAsset(const std::string& extension)
{
    return extension == "obj" || extension == "fbx" || extension == "gltf" || extension == "glb" ||
          extension == "rmesh" || extension == "rstm" || extension == "mesh" ||
          extension == "b3d" || extension == "3ds" || extension == "ms3d";
}

// Radion's own format - already the engine's native representation, so
// bringing one into the scene is a plain Load. Everything else in
// isMeshAsset() goes through one of the foreign-format importers instead,
// which is what "Import" means here.
bool isNativeMeshAsset(const std::string& extension)
{
    return extension == "rmesh" || extension == "rstm";
}

// A saved GameObject subtree (Prefab::saveToFile()'s own format) - reading
// one is a plain SceneSerializer::subtreeFromJson(), not an importer.
bool isPrefabAsset(const std::string& extension)
{
    return extension == "rprefab";
}

std::string objMaterialName(const std::string& source, usize index)
{
    std::string name;
    name.reserve(source.size() + 16);
    for (const char character : source)
    {
        const unsigned char value = static_cast<unsigned char>(character);
        if (std::isalnum(value) || character == '_' || character == '-' || character == '.')
            name.push_back(character);
        else
            name.push_back('_');
    }
    while (!name.empty() && name.front() == '_')
        name.erase(name.begin());
    while (!name.empty() && name.back() == '_')
        name.pop_back();
    if (name.empty())
        name = "Material";
    return "mat_" + std::to_string(index) + "_" + name;
}

std::string relativeTexturePath(const std::string& file, const std::filesystem::path& sourceDirectory,
                                const std::filesystem::path& outputDirectory)
{
    if (file.empty())
        return std::string();

    namespace fs = std::filesystem;
    fs::path texture = fs::path(file);
    const std::string resolved = FileSystem::getSingleton().resolve(file);
    if (!resolved.empty())
        texture = fs::path(resolved);
    else if (texture.is_relative())
        texture = sourceDirectory / texture;

    std::error_code error;
    const fs::path relative = fs::relative(texture, outputDirectory, error);
    if (!error && !relative.empty())
        return relative.generic_string();
    return fs::path(file).generic_string();
}

bool exportRmeshToObj(const std::string& sourceFile, const std::string& objFile)
{
    MeshData mesh;
    if (!Assets().importMeshFileData(sourceFile, mesh))
        return false;

    namespace fs = std::filesystem;
    fs::path objPath(objFile);
    const fs::path sourcePath(sourceFile);
    const fs::path outputDirectory = objPath.parent_path();
    const fs::path sourceDirectory = outputDirectory;
    fs::path mtlPath = objPath;
    mtlPath.replace_extension(".mtl");

    std::vector<std::string> materialNames;
    usize materialCount = mesh.materials.size();
    for (const SubMesh& submesh : mesh.submeshes)
        materialCount = std::max(materialCount, static_cast<usize>(submesh.materialSlot) + 1);
    materialNames.reserve(materialCount);
    for (usize index = 0; index < materialCount; ++index)
    {
        const std::string material = index < mesh.materials.size() ? mesh.materials[index].name : "";
        materialNames.push_back(objMaterialName(material, index));
    }

    std::ofstream mtl(mtlPath);
    if (!mtl)
        return false;
    for (usize index = 0; index < materialCount; ++index)
    {
        const Material* material = index < mesh.materials.size() ? &mesh.materials[index] : nullptr;
        const glm::vec4 base = material ? material->params.baseColor : glm::vec4(0.75f, 0.78f, 0.82f, 1.0f);
        const glm::vec4 emissive = material ? material->params.emissive : glm::vec4(0.0f);
        mtl << "newmtl " << materialNames[index] << "\n"
             << "Ka 0.000000 0.000000 0.000000\n"
             << "Kd " << base.r << ' ' << base.g << ' ' << base.b << "\n"
             << "Ks 0.000000 0.000000 0.000000\n"
             << "Ke " << emissive.r << ' ' << emissive.g << ' ' << emissive.b << "\n"
             << "d " << base.a << "\n";

        const auto textureFile = [&](MaterialSlot slot, usize fallbackIndex) -> std::string
        {
            if (material && !material->textures[slot].file.empty())
                return material->textures[slot].file;
            switch (slot)
            {
            case SlotAlbedo: return fallbackIndex < mesh.materialTextureFiles.size()
                                       ? mesh.materialTextureFiles[fallbackIndex] : std::string();
            case SlotNormal: return fallbackIndex < mesh.materialNormalFiles.size()
                                       ? mesh.materialNormalFiles[fallbackIndex] : std::string();
            case SlotSurface: return fallbackIndex < mesh.materialSurfaceFiles.size()
                                        ? mesh.materialSurfaceFiles[fallbackIndex] : std::string();
            case SlotEmissive: return fallbackIndex < mesh.materialEmissiveFiles.size()
                                         ? mesh.materialEmissiveFiles[fallbackIndex] : std::string();
            default: return std::string();
            }
        };
        const std::string albedo = textureFile(SlotAlbedo, index);
        const std::string normal = textureFile(SlotNormal, index);
        const std::string surface = textureFile(SlotSurface, index);
        const std::string emissiveMap = textureFile(SlotEmissive, index);
        if (!albedo.empty())
            mtl << "map_Kd " << relativeTexturePath(albedo, sourceDirectory, outputDirectory) << "\n";
        if (!normal.empty())
            mtl << "map_Bump " << relativeTexturePath(normal, sourceDirectory, outputDirectory) << "\n";
        if (!surface.empty())
            mtl << "map_Ks " << relativeTexturePath(surface, sourceDirectory, outputDirectory) << "\n";
        if (!emissiveMap.empty())
            mtl << "map_Ke " << relativeTexturePath(emissiveMap, sourceDirectory, outputDirectory) << "\n";
        mtl << "\n";
    }
    mtl.close();

    std::ofstream obj(objPath);
    if (!obj)
        return false;
    obj << "# Exported from " << sourcePath.filename().generic_string() << "\n"
        << "mtllib " << mtlPath.filename().generic_string() << "\n"
        << "o " << objPath.stem().generic_string() << "\n";
    for (const glm::vec3& position : mesh.positions)
        obj << "v " << position.x << ' ' << position.y << ' ' << position.z << "\n";
    for (const glm::vec2& uv : mesh.uvs)
        obj << "vt " << uv.x << ' ' << (1.0f - uv.y) << "\n";
    for (const glm::vec3& normal : mesh.normals)
        obj << "vn " << normal.x << ' ' << normal.y << ' ' << normal.z << "\n";

    const bool hasUV = mesh.uvs.size() == mesh.positions.size();
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    const auto faceVertex = [hasUV, hasNormals](u32 index)
    {
        const u32 value = index + 1;
        if (hasUV && hasNormals)
            return std::to_string(value) + "/" + std::to_string(value) + "/" + std::to_string(value);
        if (hasUV)
            return std::to_string(value) + "/" + std::to_string(value);
        if (hasNormals)
            return std::to_string(value) + "//" + std::to_string(value);
        return std::to_string(value);
    };

    for (usize submeshIndex = 0; submeshIndex < mesh.submeshes.size(); ++submeshIndex)
    {
        const SubMesh& submesh = mesh.submeshes[submeshIndex];
        obj << "g submesh_" << submeshIndex << "\n";
        if (submesh.materialSlot < materialNames.size())
            obj << "usemtl " << materialNames[submesh.materialSlot] << "\n";
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        if (submesh.indexCount % 3 != 0 || end > mesh.indices.size())
            return false;
        for (u32 offset = 0; offset < submesh.indexCount; offset += 3)
        {
            const u32 a = mesh.indices[submesh.indexOffset + offset];
            const u32 b = mesh.indices[submesh.indexOffset + offset + 1];
            const u32 c = mesh.indices[submesh.indexOffset + offset + 2];
            if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size())
                return false;
            obj << "f " << faceVertex(a) << ' ' << faceVertex(b) << ' ' << faceVertex(c) << "\n";
        }
    }
    return obj.good();
}

const char* iconForAsset(const FileSystem::DirEntry& entry)
{
    if (entry.isDirectory)
        return ICON_MDI_FOLDER;

    const std::string extension = extensionOf(entry.name);
    if (isImageAsset(extension) || extension == "exr")
        return ICON_MDI_FILE_IMAGE;
    if (isMeshAsset(extension))
        return ICON_MDI_CUBE_OUTLINE;
    if (isPrefabAsset(extension))
        return ICON_MDI_PACKAGE_VARIANT_CLOSED;
    if (extension == "cpp" || extension == "c" || extension == "h" || extension == "hpp" ||
        extension == "glsl" || extension == "vert" || extension == "frag" || extension == "lua" ||
        isScriptAsset(extension))
        return ICON_MDI_CODE_BRACES;
    if (extension == "txt" || extension == "md" || extension == "json" || extension == "xml" ||
        extension == "yaml" || extension == "yml" || extension == "ini" || extension == "cfg")
        return ICON_MDI_FILE_DOCUMENT;
    if (extension == "wav" || extension == "mp3" || extension == "ogg" || extension == "flac")
        return ICON_MDI_FILE_MUSIC;
    return ICON_MDI_FILE;
}

// One colour per category so the icon-only grid reads at a glance instead of
// every entry being the same white glyph - a folder should not look like a
// mesh should not look like a script.
ImVec4 iconColorForAsset(const FileSystem::DirEntry& entry)
{
    if (entry.isDirectory)
        return ImVec4(0.95f, 0.78f, 0.35f, 1.0f); // folder tan
    const std::string extension = extensionOf(entry.name);
    if (isImageAsset(extension) || extension == "exr")
        return ImVec4(0.55f, 0.85f, 0.55f, 1.0f); // image green
    if (isMeshAsset(extension))
        return ImVec4(0.4f, 0.75f, 0.9f, 1.0f); // mesh cyan
    if (isPrefabAsset(extension))
        return ImVec4(0.75f, 0.55f, 0.95f, 1.0f); // prefab violet
    if (extension == "cpp" || extension == "c" || extension == "h" || extension == "hpp" ||
        extension == "glsl" || extension == "vert" || extension == "frag" || extension == "lua" ||
        extension == "py")
        return ImVec4(0.6f, 0.65f, 0.95f, 1.0f); // code blue
    if (extension == "txt" || extension == "md" || extension == "json" || extension == "xml" ||
        extension == "yaml" || extension == "yml" || extension == "ini" || extension == "cfg")
        return ImVec4(0.75f, 0.75f, 0.75f, 1.0f); // document grey
    if (extension == "wav" || extension == "mp3" || extension == "ogg" || extension == "flac")
        return ImVec4(0.9f, 0.55f, 0.65f, 1.0f); // audio pink
    return ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // default text colour, unknown type
}
} // namespace

AssetsPanel::AssetsPanel(EditorApplication& app) : EditorPanel("Assets", app)
{
    const EditorSettings& settings = app.settings();
    mCurrentDirectory = settings.assetsDirectory;
    mViewMode = static_cast<ViewMode>(glm::clamp(settings.assetsViewMode, 0, 2));
    mThumbnailSize = glm::clamp(settings.assetsThumbnailSize, 48.0f, 256.0f);
    mHistory.clear();
    mHistory.push_back(mCurrentDirectory);
    mHistoryPosition = 0;
    mLastBrowserRoot = app.assetBrowserRoot();
}

void AssetsPanel::navigateTo(const std::filesystem::path& directory)
{
    const std::filesystem::path normalized = directory.lexically_normal();
    if (normalized == mCurrentDirectory)
        return;
    // Drop anything ahead of the current spot first - the same rule a
    // browser's own history follows: going Back and then somewhere new
    // abandons the branch that used to be reachable by Forward.
    mHistory.erase(mHistory.begin() + static_cast<std::ptrdiff_t>(mHistoryPosition) + 1,
                   mHistory.end());
    mHistory.push_back(normalized);
    mHistoryPosition = mHistory.size() - 1;
    mCurrentDirectory = normalized;
    mEntriesDirty = true;
}

void AssetsPanel::refreshEntries(const std::filesystem::path& directory)
{
    if (!mEntriesDirty && mCachedDirectory == directory)
        return;

    mTreeCache.clear();
    mRelativePathCache.clear();
    mEntries = FileSystem::getSingleton().listDirectory(directory.string());
    std::sort(mEntries.begin(), mEntries.end(),
              [](const FileSystem::DirEntry& a, const FileSystem::DirEntry& b)
              {
                  if (a.isDirectory != b.isDirectory)
                      return a.isDirectory > b.isDirectory;
                  return a.name < b.name;
              });
    mCachedDirectory = directory;
    mEntriesDirty = false;
}

namespace
{
// TreeNodeEx's own arrow is a few pixels of triangle at the row's own font
// size - easy to miss, and OpenOnArrow means missing it just re-navigates
// instead of expanding. ArrowButton is a full, dedicated widget (its own
// hit-box, drawn with ImGui's native RenderArrow rather than a glyph from
// whatever icon font may or may not have loaded), so every row in the
// browser tree gets one explicitly instead of relying on it.
bool drawExpandArrow(ImGuiStorage* storage, bool hasChildren)
{
    if (!hasChildren)
    {
        ImGui::Dummy(ImVec2(ImGui::GetFrameHeight(), 1.0f));
        ImGui::SameLine(0.0f, 4.0f);
        return false;
    }
    const ImGuiID openId = ImGui::GetID("##open");
    bool open = storage->GetBool(openId, false);
    if (ImGui::ArrowButton("##expand", open ? ImGuiDir_Down : ImGuiDir_Right))
    {
        open = !open;
        storage->SetBool(openId, open);
    }
    ImGui::SameLine(0.0f, 4.0f);
    return open;
}

bool directoryHasSubdirectory(const std::filesystem::path& directory)
{
    for (const FileSystem::DirEntry& entry : FileSystem::getSingleton().listDirectory(directory.string()))
        if (entry.isDirectory)
            return true;
    return false;
}
} // namespace

// One top-level tree row for a fixed location (project Assets, engine
// Assets, an extra search path, or the filesystem root) - clicking the label
// jumps straight there, the arrow drills into it with drawDirectoryTree().
void AssetsPanel::drawBookmark(const char* label, const std::filesystem::path& root)
{
    std::error_code error;
    if (root.empty() || !std::filesystem::is_directory(root, error))
        return;

    const std::filesystem::path normalizedRoot = root.lexically_normal();

    // By the path, not the label: two search paths with the same basename
    // (models/soldier/textures and models/castel/textures both showing as
    // "textures", say) are different bookmarks and need different ids, or
    // ImGui reports them as one conflicting widget and renders neither right.
    ImGui::PushID(normalizedRoot.string().c_str());
    const bool open = drawExpandArrow(ImGui::GetStateStorage(), true);
    const std::string rowLabel = std::string(ICON_MDI_FOLDER) + " " + label;
    if (ImGui::Selectable(rowLabel.c_str(), normalizedRoot == mCurrentDirectory))
        navigateTo(normalizedRoot);
    if (open)
    {
        ImGui::Indent();
        drawDirectoryTree(normalizedRoot);
        ImGui::Unindent();
    }
    ImGui::PopID();
}

void AssetsPanel::drawDirectoryTree(const std::filesystem::path& directory)
{
    const std::string key = directory.string();
    auto cached = mTreeCache.find(key);
    if (cached == mTreeCache.end())
    {
        std::vector<FileSystem::DirEntry> children = FileSystem::getSingleton().listDirectory(key);
        std::sort(children.begin(), children.end(),
                  [](const FileSystem::DirEntry& a, const FileSystem::DirEntry& b)
                  {
                      return a.name < b.name;
                  });
        TreeDirectory listing;
        for (const FileSystem::DirEntry& entry : children)
        {
            if (!entry.isDirectory)
                continue;
            // The has-children peek behind the arrow is a listDirectory() of
            // its own - done here, once per cache fill, never per frame.
            listing.names.push_back(entry.name);
            listing.hasChildren.push_back(directoryHasSubdirectory(directory / entry.name) ? 1
                                                                                           : 0);
        }
        cached = mTreeCache.emplace(key, std::move(listing)).first;
    }

    const TreeDirectory& listing = cached->second;
    for (usize i = 0; i < listing.names.size(); ++i)
    {
        const std::string& name = listing.names[i];
        const std::filesystem::path childPath = directory / name;

        ImGui::PushID(name.c_str());
        const bool open = drawExpandArrow(ImGui::GetStateStorage(), listing.hasChildren[i] != 0);
        const std::string rowLabel = std::string(ICON_MDI_FOLDER) + " " + name;
        if (ImGui::Selectable(rowLabel.c_str(), childPath == mCurrentDirectory))
            navigateTo(childPath);
        if (open)
        {
            ImGui::Indent();
            drawDirectoryTree(childPath);
            ImGui::Unindent();
        }
        ImGui::PopID();
    }
}

bool AssetsPanel::assetRelativePath(const std::filesystem::path& absolute, std::string& outRelative)
{
    const std::string key = absolute.string();
    const auto cached = mRelativePathCache.find(key);
    if (cached != mRelativePathCache.end())
    {
        if (cached->second.resolved)
            outRelative = cached->second.relative;
        return cached->second.resolved;
    }

    RelativePathResult result;
    std::vector<std::filesystem::path> roots;
    roots.emplace_back(app().assetBrowserRoot());
    roots.emplace_back(RADION_ASSET_DIR);
    for (const std::string& path : app().projectSearchPaths())
        roots.emplace_back(path);

    const std::filesystem::path normalized = absolute.lexically_normal();
    for (const std::filesystem::path& root : roots)
    {
        std::error_code error;
        if (root.empty() || !std::filesystem::is_directory(root, error))
            continue;

        const std::filesystem::path relative =
            std::filesystem::relative(normalized, root.lexically_normal(), error);
        if (error || relative.empty())
            continue;
        // std::filesystem::relative() happily returns a path starting with
        // ".." when `absolute` is not actually under `root` - that is not a
        // match, it is "how far away", so reject it the same way a failed
        // resolve would be. "." is the opposite case - `absolute` IS `root`
        // itself, exactly - and is a real match, not a rejection; only a
        // caller passing a directory (not a file under it) ever sees it.
        const std::string relativeString = relative.generic_string();
        if (relativeString.rfind("..", 0) == 0)
            continue;

        result.resolved = true;
        result.relative = relativeString == "." ? std::string() : relativeString;
        break;
    }

    mRelativePathCache.emplace(key, result);
    if (result.resolved)
        outRelative = result.relative;
    return result.resolved;
}

TextureHandle AssetsPanel::thumbnailFor(const std::string& relativePath)
{
    const auto it = mThumbnailCache.find(relativePath);
    if (it != mThumbnailCache.end())
        return it->second;

    // sRGB: a browser thumbnail is judged by eye like any other colour image,
    // never sampled as data - the one place colorSpaceFor(slot) does not
    // apply because there is no material slot yet to ask it about.
    const TextureHandle texture = Assets().loadTexture(relativePath, ColorSpace::sRGB);
    mThumbnailCache.emplace(relativePath, texture);
    return texture;
}

std::string AssetsPanel::importOutputBase()
{
    // mImportPath is relative to whichever registered search path
    // assetRelativePath() matched it against - resolve() already tries every
    // one of them, so there is no single root to prefix it with here.
    const std::string resolved = FileSystem::getSingleton().resolve(mImportPath);
    const std::string base =
        resolved.empty() ? app().assetBrowserRoot() + "/" + mImportPath : resolved;
    const usize dot = base.find_last_of('.');
    return dot == std::string::npos ? base : base.substr(0, dot);
}

void AssetsPanel::instantiatePrefab(const std::string& relativePath)
{
    Prefab prefab;
    if (!prefab.load(relativePath))
    {
        Log::error("AssetsPanel: could not read prefab '%s'", relativePath.c_str());
        app().toasts().error("Could not read " + FileSystem::fileName(relativePath));
        return;
    }

    app().recordUndo();
    SceneLoadResult result;
    GameObject* object = prefab.instantiate(app().scene(), nullptr, result);
    if (!object)
    {
        for (const SceneDiagnostic& diagnostic : result.diagnostics)
            Log::error("AssetsPanel: prefab instantiate failed - %s: %s",
                       diagnostic.jsonPath.c_str(), diagnostic.message.c_str());
        app().toasts().error("Could not instantiate " + FileSystem::fileName(relativePath));
        return;
    }

    // Position only - rotation/scale are what the prefab was saved with, and
    // stay untouched. Children keep their own transform, local to this root.
    object->setPosition(app().cursor3D());
    // Prefab::instantiate() only queues the new objects; they are not walkable
    // (childCount(), the transform hierarchy) until the Scene's normal add
    // flush runs, ordinarily at the top of next frame. Selecting one right
    // away needs that flush now instead.
    app().scene().update(0.0f);
    app().selection().select(object->id());
    app().markDirty();
    app().toasts().success("Instantiated " + FileSystem::fileName(relativePath));
}

void AssetsPanel::drawDeletePopup()
{
    constexpr const char* kPopupId = "Delete Asset##AssetsDelete";
    if (mDeletePending)
    {
        ImGui::OpenPopup(kPopupId);
        mDeletePending = false;
    }

    ImGui::SetNextWindowSize(ImVec2(420.0f, 0.0f), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal(kPopupId, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    FileSystem& files = FileSystem::getSingleton();
    const std::string full = app().assetBrowserRoot() + "/" + mDeletePath;
    const std::string resolved = files.resolve(mDeletePath);
    const std::string target = resolved.empty() ? full : resolved;

    ImGui::TextWrapped("Delete %s?", mDeletePath.c_str());
    ImGui::TextDisabled("%s", target.c_str());
    ImGui::Separator();
    ImGui::TextWrapped("This deletes the file from disk. The editor's undo cannot bring it back, "
                       "and any scene still pointing at it will fail to load its mesh.");

    // A mesh asset never travels alone: the .material and .rskel written
    // beside it are useless without it and would otherwise be left behind as
    // orphans nobody remembers deleting.
    static const char* const companions[] = {".material", ".rskel"};
    const std::string base = FileSystem::withoutExtension(target);
    std::vector<std::string> extras;
    if (isMeshAsset(FileSystem::extensionOf(mDeletePath)) ||
        isNativeMeshAsset(FileSystem::extensionOf(mDeletePath)))
        for (const char* companion : companions)
            if (files.exists(base + companion))
                extras.push_back(base + companion);
    if (!extras.empty())
    {
        ImGui::Separator();
        ImGui::TextDisabled("Also deletes:");
        for (const std::string& extra : extras)
            ImGui::BulletText("%s", FileSystem::fileName(extra).c_str());
    }

    ImGui::Separator();
    if (ImGui::Button("Delete", ImVec2(140.0f, 0.0f)))
    {
        u32 deleted = 0;
        if (files.removeFile(target))
            ++deleted;
        for (const std::string& extra : extras)
            if (files.removeFile(extra))
                ++deleted;
        if (deleted)
        {
            mEntriesDirty = true;
            app().toasts().success("Deleted " + FileSystem::fileName(target) +
                                   (deleted > 1 ? " and " + std::to_string(deleted - 1) + " more"
                                                : std::string()));
        }
        else
            app().toasts().error("Could not delete " + FileSystem::fileName(target));
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(140.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void AssetsPanel::drawImportPopup()
{
    constexpr const char* kPopupId = "Import Mesh##AssetsImport";
    if (mImportPending)
    {
        ImGui::OpenPopup(kPopupId);
        mImportPending = false;
    }

    ImGui::SetNextWindowSize(ImVec2(340.0f, 0.0f), ImGuiCond_Appearing);
    // Import options must not capture the whole editor: while this popup is
    // open the user may still need to drag a texture into Terrain/Inspector.
    if (!ImGui::BeginPopup(kPopupId, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    ImGui::TextWrapped("%s", mImportPath.c_str());
    ImGui::Separator();
    ImGui::DragFloat3("Translate", &mImportTranslation.x, 0.01f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Offset from the 3D cursor - where the new object lands.");
    ImGui::DragFloat3("Rotate", &mImportRotationEuler.x, 0.5f, -360.0f, 360.0f, "%.1f°");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Degrees, applied before the object ever renders - an export commonly "
                          "comes in facing the wrong way for this scene's own up/forward.");
    ImGui::DragFloat("Scale", &mImportScale, 0.01f, 0.0001f, 10000.0f, "%.4f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Uniform scale applied to the new object right away, before it ever "
                          "renders - an export can land many orders of magnitude off the scene's "
                          "own scale (a whole building at 1 unit = 1cm is a common one), and "
                          "catching that here beats redoing the import after the fact.");
    ImGui::Separator();

    ImGui::Checkbox("Optimize", &mImportOptimize);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Join submeshes that share a material into one, cutting draw calls.");
    ImGui::Checkbox("Split", &mImportSplit);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Break oversized submeshes into spatially local pieces, so the scene "
                          "BVH can cull them individually instead of one box for the whole thing.");
    ImGui::BeginDisabled(!mImportSplit);
    ImGui::DragInt("Target Triangles", &mImportSplitTriangles, 50.0f, 500, 50000);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Triangle budget per split submesh.");
    ImGui::EndDisabled();
    if (!isNativeMeshAsset(extensionOf(mImportPath)) || mImportOptimize || mImportSplit)
    {
        const std::string outputBase = importOutputBase();
        ImGui::TextWrapped("Writes %s.rmesh (+ .material, + .rskel if rigged). That is the asset "
                          "the scene refers to; the source is only read here. Change the source "
                          "and import it again.",
                          outputBase.c_str());
    }
    ImGui::Separator();

    if (ImGui::Button("Import", ImVec2(120.0f, 0.0f)))
    {
        // A foreign export commonly ships as "modelname/modelname.obj" next
        // to "modelname/textures/" (Sponza is the standard example) - the
        // importer's own texture references are relative to the mesh's own
        // directory (GltfImporter/ObjImporter's `directory` param), which
        // only resolves through FileSystem if that folder is a search path.
        // Adding both here, before the load, means Sponza-shaped assets work
        // without the user finding Settings > Search Paths first.
        const std::string resolvedMeshPath = FileSystem::getSingleton().resolve(mImportPath);
        const std::string meshDir = FileSystem::directoryOf(
            resolvedMeshPath.empty() ? app().assetBrowserRoot() + "/" + mImportPath
                                     : resolvedMeshPath);
        app().addProjectSearchPath(meshDir);
        const std::string texturesDir = meshDir + "/textures";
        if (FileSystem::getSingleton().isDirectory(texturesDir))
            app().addProjectSearchPath(texturesDir);

        // importMeshFileData() rather than createMesh(MeshDesc::fromFile()):
        // this keeps the decoded MeshData around and hands it to
        // EditorApplication, so InspectorPanel can later run a mesh tool
        // (regenerate normals/tangents/planar UV) against the exact data
        // this object's mesh came from - a MeshDesc is just "File, this
        // path", nowhere to keep an edit that only lives in memory.
        MeshData meshData;
        if (!Assets().importMeshFileData(mImportPath, meshData))
        {
            Log::error("AssetsPanel: failed to import mesh '%s'", mImportPath.c_str());
            app().toasts().error("Could not import " + FileSystem::fileName(mImportPath));
        }
        else
        {
            // Optimize before Split: joining first gives the split pass whole
            // material groups to slice into spatially coherent chunks - the
            // other order would split by triangle budget and then undo it.
            if (mImportOptimize)
            {
                const usize before = meshData.submeshes.size();
                if (Assets().mergeSubmeshesByMaterial(meshData))
                    Log::info("AssetsPanel: optimize merged %zu submeshes into %zu", before,
                             meshData.submeshes.size());
            }
            if (mImportSplit)
            {
                const usize before = meshData.submeshes.size();
                Assets().splitSubMeshes(meshData, static_cast<u32>(mImportSplitTriangles));
                if (meshData.submeshes.size() != before)
                    Log::info("AssetsPanel: split %zu submeshes into %zu", before,
                             meshData.submeshes.size());
            }

            // A foreign format is never what the scene refers to: it is
            // decoded once here and written out as ours, and it is the
            // .rmesh that the recipe names from then on. That keeps the
            // runtime free of every importer, makes reopening a scene a
            // binary read instead of a re-parse, and means whatever Optimize
            // and Split did is part of the asset rather than something the
            // next load quietly undoes. Changing the source means importing
            // it again - there is no staleness tracking and no sidecar of
            // import options.
            const bool foreignSource = !isNativeMeshAsset(extensionOf(mImportPath));
            std::string recipePath = mImportPath;
            if (foreignSource || mImportOptimize || mImportSplit)
            {
                const std::string outputBase = importOutputBase();
                const std::string meshOutput = outputBase + ".rmesh";
                const std::string materialOutput = outputBase + ".material";

                std::string skeletonOutput;
                Skeleton skeleton;
                if (Assets().importSkeleton(mImportPath, skeleton) && skeleton.boneCount() > 0)
                {
                    skeletonOutput = outputBase + ".rskel";
                    if (!RadionSkeletonIO::saveSkeleton(skeletonOutput, skeleton))
                    {
                        Log::error("AssetsPanel: could not write skeleton '%s'",
                                   skeletonOutput.c_str());
                        app().toasts().warning("Could not write " +
                                               FileSystem::fileName(skeletonOutput) +
                                               " - importing without a skeleton");
                        skeletonOutput.clear();
                    }
                }

                if (!Assets().saveMesh(meshData, meshOutput, skeletonOutput))
                {
                    Log::error("AssetsPanel: could not write '%s' - the object will still be "
                               "created, but the scene cannot refer to it",
                               meshOutput.c_str());
                    app().toasts().error("Could not write " + FileSystem::fileName(meshOutput) +
                                         " - the scene will not be able to refer to it");
                }
                else
                {
                    recipePath = meshOutput;
                    const std::string stem = FileSystem::baseName(meshOutput);
                    std::vector<std::string> used;
                    u32 counter = 0;
                    for (Material& material : meshData.materials)
                    {
                        const bool unnamed =
                            material.name.empty() || material.name == " -- default --";
                        bool clash = false;
                        if (!unnamed)
                            for (const std::string& name : used)
                                if (name == material.name)
                                {
                                    clash = true;
                                    break;
                                }
                        if (unnamed || clash)
                        {
                            std::string candidate;
                            do
                            {
                                candidate = stem + "_" + std::to_string(counter++);
                                clash = false;
                                for (const std::string& name : used)
                                    if (name == candidate)
                                    {
                                        clash = true;
                                        break;
                                    }
                            } while (clash);
                            material.name = candidate;
                            material.nameHash = hashName(material.name.c_str());
                        }
                        used.push_back(material.name);
                    }
                    if (!meshData.materials.empty() &&
                        !MaterialManager::getSingleton().save(
                            materialOutput, Assets().materialsForSidecar(meshData)))
                    {
                        Log::error("AssetsPanel: could not write '%s'", materialOutput.c_str());
                        app().toasts().warning("Could not write " +
                                               FileSystem::fileName(materialOutput));
                    }
                    mEntriesDirty = true;
                    Log::info("AssetsPanel: written as '%s'", meshOutput.c_str());
                    app().toasts().success("Imported as " + FileSystem::fileName(meshOutput));
                }
            }

            const MeshHandle mesh = Assets().createMesh(meshData);
            if (!mesh.valid())
            {
                Log::error("AssetsPanel: failed to upload mesh '%s'", mImportPath.c_str());
                app().toasts().error("Could not upload " + FileSystem::fileName(mImportPath));
            }
            else
            {
                // Without this SceneSerializer finds no recipe for `mesh` and
                // silently refuses to save the reference at all - saving and
                // reopening the scene would drop the object's mesh entirely.
                Assets().registerMeshDesc(mesh, MeshDesc::fromFile(recipePath));
                app().registerImportedMesh(mesh, std::move(meshData));
                app().recordUndo();
                GameObject* object = app().scene().createGameObject(mImportName, nullptr);
                if (object)
                {
                    object->setPosition(app().cursor3D() + mImportTranslation);
                    object->setRotation(glm::quat(glm::radians(mImportRotationEuler)));
                    object->setScale(glm::vec3(mImportScale));
                    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
                    renderer->setMesh(mesh);
                    app().scene().update(0.0f);
                    app().selection().select(object->id());
                    // Submesh indices only mean anything against the mesh
                    // they were picked on - a batch left over from the last
                    // one would aim the next Delete at this new mesh's
                    // pieces instead.
                    app().submeshSelection().object = 0;
                    app().submeshSelection().indices.clear();
                    app().markDirty();
                }
            }
        }
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void AssetsPanel::drawGeneratePopup()
{
    if (mGenerateKind == GenerateKind::None ||
        !mGenerateDialog.Render(app().assetBrowserRoot(), app().assetBrowserRoot(),
                               app().assetBrowserRoot()))
        return;

    const ImGuiFileDialog::Result result = mGenerateDialog.ConsumeResult();
    const GenerateKind kind = mGenerateKind;
    const std::string sourcePath = mGenerateSourcePath;
    mGenerateKind = GenerateKind::None;
    mGenerateSourcePath.clear();
    if (!result.accepted)
        return;
    app().settings().lastSaveDirectory = result.path.parent_path().string();

    Pixmap source;
    if (!source.load(sourcePath.c_str()))
    {
        Log::error("AssetsPanel: could not read '%s'", sourcePath.c_str());
        app().toasts().error("Could not read " + FileSystem::fileName(sourcePath));
        return;
    }

    Pixmap* generated =
        kind == GenerateKind::Normal ? source.generate_normal_map() : source.generate_heightmap();
    if (!generated)
    {
        Log::error("AssetsPanel: could not generate from '%s'", sourcePath.c_str());
        app().toasts().error("Could not generate from " + FileSystem::fileName(sourcePath));
        return;
    }
    if (generated->save(result.path.string().c_str()))
    {
        Log::info("AssetsPanel: saved '%s'", result.path.string().c_str());
        app().toasts().success("Saved " + FileSystem::fileName(result.path.string()));
        mEntriesDirty = true;
    }
    else
    {
        Log::error("AssetsPanel: could not save '%s'", result.path.string().c_str());
        app().toasts().error("Could not save " + FileSystem::fileName(result.path.string()));
    }
    delete generated;
}

void AssetsPanel::openCreateFolderPopup(const std::filesystem::path& directory)
{
    mCreateTargetDirectory = directory.lexically_normal();
    std::snprintf(mNewFolderName, sizeof(mNewFolderName), "%s", "New Folder");
    mOpenCreateFolderPopup = true;
}

void AssetsPanel::openCreateScriptPopup(const std::filesystem::path& directory)
{
    mCreateTargetDirectory = directory.lexically_normal();
    std::snprintf(mNewScriptName, sizeof(mNewScriptName), "%s", "NewScript");
    mOpenCreateScriptPopup = true;
}

void AssetsPanel::drawCreateFolderPopup()
{
    if (!ImGui::BeginPopup("Create Folder"))
        return;

    ImGui::TextDisabled("In %s", mCreateTargetDirectory.string().c_str());
    ImGui::SetNextItemWidth(260.0f);
    ImGui::InputText("Name", mNewFolderName, sizeof(mNewFolderName));

    const bool validName = isValidEntryName(mNewFolderName);
    const std::filesystem::path target = mCreateTargetDirectory / mNewFolderName;
    std::error_code error;
    const bool alreadyExists = validName && std::filesystem::exists(target, error);
    if (alreadyExists)
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f), "A folder or file with that name already exists.");
    else if (!validName)
        ImGui::TextDisabled("Use one name, without / or \\.");

    ImGui::BeginDisabled(!validName || alreadyExists || error);
    if (ImGui::Button("Create", ImVec2(120.0f, 0.0f)))
    {
        error.clear();
        if (std::filesystem::create_directory(target, error))
        {
            mEntriesDirty = true;
            app().toasts().success("Created folder " + target.filename().string());
            ImGui::CloseCurrentPopup();
        }
        else
        {
            Log::error("AssetsPanel: could not create folder '%s': %s", target.string().c_str(),
                       error.message().c_str());
            app().toasts().error("Could not create folder " + target.filename().string());
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void AssetsPanel::drawCreateScriptPopup()
{
    if (!ImGui::BeginPopup("Create Script"))
        return;

    ImGui::TextDisabled("In %s", mCreateTargetDirectory.string().c_str());
    ImGui::SetNextItemWidth(260.0f);
    ImGui::InputText("Name", mNewScriptName, sizeof(mNewScriptName));
    ImGui::TextDisabled("Creates a .py script with class Name(ScriptComponent).");

    const bool validName = isValidEntryName(mNewScriptName);
    const std::filesystem::path target = mCreateTargetDirectory / scriptFileName(mNewScriptName);
    std::error_code error;
    const bool alreadyExists = validName && std::filesystem::exists(target, error);
    if (alreadyExists)
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f), "%s already exists.",
                           target.filename().string().c_str());
    else if (!validName)
        ImGui::TextDisabled("Use one name, without / or \\.");

    ImGui::BeginDisabled(!validName || alreadyExists || error);
    if (ImGui::Button("Create", ImVec2(120.0f, 0.0f)))
    {
        std::ofstream file(target, std::ios::binary | std::ios::trunc);
        file << makeScriptTemplate(mNewScriptName);
        file.close();
        if (file.good())
        {
            mEntriesDirty = true;
            app().toasts().success("Created script " + target.filename().string());
            app().openScriptEditor(target.string());
            ImGui::CloseCurrentPopup();
        }
        else
        {
            Log::error("AssetsPanel: could not create script '%s'", target.string().c_str());
            app().toasts().error("Could not create script " + target.filename().string());
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void AssetsPanel::onImGui()
{
    if (const std::string root = app().assetBrowserRoot(); root != mLastBrowserRoot)
    {
        mLastBrowserRoot = root;
        mCurrentDirectory = std::filesystem::path(root).lexically_normal();
        mHistory = {mCurrentDirectory};
        mHistoryPosition = 0;
        mEntriesDirty = true;
    }

    EditorSettings& editorSettings = app().settings();
    editorSettings.assetsDirectory = mCurrentDirectory.string();
    editorSettings.assetsViewMode = static_cast<int>(mViewMode);
    editorSettings.assetsThumbnailSize = mThumbnailSize;
    if (const std::string reveal = app().takeRevealAssetRequest(); !reveal.empty())
    {
        const usize slash = reveal.find_last_of('/');
        const std::string directory =
            slash == std::string::npos ? std::string() : reveal.substr(0, slash);
        const std::filesystem::path absolute =
            (std::filesystem::path(app().assetBrowserRoot()) / directory).lexically_normal();
        Log::info("AssetsPanel: reveal '%s' -> directory '%s' (currently '%s')", reveal.c_str(),
                 absolute.string().c_str(), mCurrentDirectory.string().c_str());
        navigateTo(absolute);
        // A reveal commonly follows an asset write. Refresh even when the
        // requested file already belongs to the currently visible folder.
        mEntriesDirty = true;
    }

    drawImportPopup();
    drawDeletePopup();
    drawGeneratePopup();

    const ImGuiStyle& style = ImGui::GetStyle();
    if (ImGui::Button(ICON_MDI_HOME))
        navigateTo(app().assetBrowserRoot());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Back to %s", app().assetBrowserRoot().c_str());
    ImGui::SameLine();
    ImGui::BeginDisabled(mHistoryPosition == 0);
    if (ImGui::Button(ICON_MDI_ARROW_LEFT))
    {
        --mHistoryPosition;
        mCurrentDirectory = mHistory[mHistoryPosition];
        mEntriesDirty = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(mHistoryPosition + 1 >= mHistory.size());
    if (ImGui::Button(ICON_MDI_ARROW_RIGHT))
    {
        ++mHistoryPosition;
        mCurrentDirectory = mHistory[mHistoryPosition];
        mEntriesDirty = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    // Clickable breadcrumb over the whole absolute path - the browser is not
    // confined to any one root, so every segment down to the filesystem root
    // is shown and jumps straight to that depth instead of clicking ".."
    // repeatedly.
    {
        std::filesystem::path accumulated;
        bool first = true;
        for (const std::filesystem::path& part : mCurrentDirectory)
        {
            accumulated /= part;
            // On POSIX, iterating an absolute path yields the root directory
            // itself ("/") as its own first part - printing a separator
            // ahead of the next part on top of that is what doubled up into
            // "//media/...": the root's own label already IS the separator.
            const bool isRoot = first && part == part.root_directory();
            const std::string label = part.string();
            if (!first && !isRoot)
            {
                ImGui::SameLine(0.0f, 2.0f);
                ImGui::TextUnformatted("/");
            }
            first = false;
            if (isRoot)
                continue;
            ImGui::SameLine(0.0f, 2.0f);
            ImGui::PushID(accumulated.string().c_str());
            if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_None,
                                  ImVec2(ImGui::CalcTextSize(label.c_str()).x, 0.0f)))
                navigateTo(accumulated);
            ImGui::PopID();
        }
    }
    ImGui::SameLine();
    // Only Grid view has a cell size to zoom - List/Details are plain rows,
    // widening them would just add blank space either side of the text.
    const f32 zoomSliderWidth = mViewMode == ViewMode::Grid ? 100.0f : 0.0f;
    const f32 viewButtonsWidth = ImGui::CalcTextSize(ICON_MDI_VIEW_GRID).x +
                                 ImGui::CalcTextSize(ICON_MDI_VIEW_LIST).x +
                                 ImGui::CalcTextSize(ICON_MDI_TABLE).x +
                                 ImGui::CalcTextSize(ICON_MDI_REFRESH).x +
                                 style.FramePadding.x * 8.0f + style.ItemSpacing.x * 3.0f +
                                 (zoomSliderWidth > 0.0f ? zoomSliderWidth + style.ItemSpacing.x
                                                        : 0.0f);
    ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - viewButtonsWidth);
    if (zoomSliderWidth > 0.0f)
    {
        ImGui::SetNextItemWidth(zoomSliderWidth);
        ImGui::SliderFloat("##Zoom", &mThumbnailSize, 48.0f, 256.0f, ICON_MDI_MAGNIFY " %.0f");
        ImGui::SameLine();
    }
    if (ImGui::Button(ICON_MDI_REFRESH))
        mEntriesDirty = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Refresh assets");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_VIEW_GRID))
        mViewMode = ViewMode::Grid;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Icon grid");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_VIEW_LIST))
        mViewMode = ViewMode::List;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("List");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_TABLE))
        mViewMode = ViewMode::Details;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Details");

    // A folder reached only through the breadcrumb/Filesystem bookmark, not
    // under any registered root, cannot resolve a relative path for
    // anything in it - assetRelativePath() fails for every entry, which is
    // what silently drops the per-item context menu (BeginPopupContextItem()
    // returns early past that failure) without any error to explain why.
    // Registering the folder here is the fix, not a workaround: everything
    // downstream already assumes an entry has a root-relative path.
    {
        std::string ignored;
        if (!assetRelativePath(mCurrentDirectory, ignored))
        {
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                               "Not under a registered search path - right-click menus (Load, "
                               "Convert, Generate Animation...) will not appear here.");
            ImGui::SameLine();
            if (ImGui::Button("Add as search path"))
            {
                app().addProjectSearchPath(mCurrentDirectory.string());
                const std::filesystem::path texturesDir = mCurrentDirectory / "textures";
                std::error_code error;
                if (std::filesystem::is_directory(texturesDir, error))
                    app().addProjectSearchPath(texturesDir.string());
                // Every relPath computed under the old, unregistered root was
                // cached as unresolved - the cache has to go, not just the
                // entry list, or every context menu stays broken until the
                // next full editor restart.
                mRelativePathCache.clear();
                mEntriesDirty = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Also adds this folder's own textures/ subfolder, if it has "
                                  "one - the same convention Import already follows.");
        }
    }
    ImGui::Separator();

    // Left-hand folder tree, the panel's own take on the Blender file
    // browser's sidebar: a handful of fixed bookmarks (project Assets,
    // engine Assets, extra search paths, and the filesystem itself as an
    // escape hatch) rather than one project-locked root, so a bookmark click
    // or the breadcrumb can reach anywhere on disk. Horizontal scrollbar:
    // deep nesting pushes labels past the panel's width, and without it
    // there was no way to read - or reach - what a folder that deep was
    // even called.
    ImGui::BeginChild("##assets_tree", ImVec2(mTreeWidth, 0.0f), true,
                      ImGuiWindowFlags_HorizontalScrollbar);
    {
        if (app().hasProject())
            drawBookmark("Project", app().assetBrowserRoot());
        drawBookmark("Engine", RADION_ASSET_DIR);
        for (const std::string& path : app().projectSearchPaths())
            drawBookmark(FileSystem::fileName(path).c_str(), path);
        drawBookmark("Filesystem", std::filesystem::path("/"));
    }
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::InvisibleButton("##assets_tree_splitter", ImVec2(4.0f, ImGui::GetContentRegionAvail().y));
    if (ImGui::IsItemActive())
        mTreeWidth = std::clamp(mTreeWidth + ImGui::GetIO().MouseDelta.x, 120.0f, 420.0f);
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    ImGui::SameLine();

    ImGui::BeginChild("##assets_content", ImVec2(0.0f, 0.0f), false);

    if (mCurrentDirectory.has_parent_path() && mCurrentDirectory.parent_path() != mCurrentDirectory)
    {
        if (ImGui::Selectable(ICON_MDI_ARROW_UP " .."))
            navigateTo(mCurrentDirectory.parent_path());
    }

    refreshEntries(mCurrentDirectory);

    const auto openEntry = [this](const FileSystem::DirEntry& entry, bool doubleClicked)
    {
        if (entry.isDirectory)
        {
            mPendingNavigation = mCurrentDirectory / entry.name;
            return;
        }

        const std::string extension = extensionOf(entry.name);
        if (isScriptAsset(extension) && doubleClicked)
        {
            app().openScriptEditor((mCurrentDirectory / entry.name).string());
            return;
        }
        if (isPrefabAsset(extension) && doubleClicked)
        {
            std::string relPath;
            if (assetRelativePath(mCurrentDirectory / entry.name, relPath))
                instantiatePrefab(relPath);
        }
    };

    const auto createMenu = [this](const std::filesystem::path& directory)
    {
        if (ImGui::MenuItem(ICON_MDI_FOLDER_PLUS " Create Folder"))
            openCreateFolderPopup(directory);
        if (ImGui::MenuItem(ICON_MDI_FILE_PLUS " Create Script"))
            openCreateScriptPopup(directory);
    };

    // Empty-space context uses the open directory; an existing folder gets
    // the same menu targeted at that folder itself (below).
    const auto directoryContextMenu = [&createMenu, this]()
    {
        if (ImGui::BeginPopupContextWindow("##assets_directory_context",
                                           ImGuiPopupFlags_MouseButtonRight |
                                               ImGuiPopupFlags_NoOpenOverItems))
        {
            createMenu(mCurrentDirectory);
            ImGui::EndPopup();
        }
    };

    const auto dragEntry = [this](const FileSystem::DirEntry& entry)
    {
        if (entry.isDirectory)
            return;
        std::string relPath;
        if (!assetRelativePath(mCurrentDirectory / entry.name, relPath))
            return;
        if (ImGui::BeginDragDropSource())
        {
            // A context/import popup can be above the asset grid. Close it at
            // the exact moment dragging starts so the payload can reach a
            // Terrain drop target in the Inspector.
            ImGui::ClosePopupToLevel(0, true);
            ImGui::SetDragDropPayload(kAssetFileDragPayload, relPath.data(), relPath.size());
            ImGui::TextUnformatted(relPath.c_str());
            ImGui::EndDragDropSource();
        }
    };

    // Right-click on a mesh file for Load/Import - the same action a
    // double-click could trigger, but explicit: this panel also uses a plain
    // click to enter folders, and reusing that for "drop a new object into
    // the scene" on a misclick would be a nasty surprise. Queues
    // drawImportPopup() rather than creating the object here - the popup is
    // what actually asks for a scale and does the work once confirmed.
    // "Load" for .rmesh/.rstm (already Radion's own format), "Import" for
    // everything else (goes through a foreign-format importer instead).
    const auto contextMenu = [this, &createMenu](const FileSystem::DirEntry& entry)
    {
        if (entry.isDirectory)
        {
            if (ImGui::BeginPopupContextItem())
            {
                createMenu(mCurrentDirectory / entry.name);
                ImGui::EndPopup();
            }
            return;
        }
        const std::string extension = extensionOf(entry.name);
        const bool mesh = isMeshAsset(extension);
        const bool image = isImageAsset(extension);
        const bool script = isScriptAsset(extension);
        const bool prefab = isPrefabAsset(extension);
        if (!mesh && !image && !script && !prefab)
            return;
        // Load/Import/Convert/Generate all end up writing beside the source
        // or feeding it to AssetManager by search-path-relative name -
        // neither makes sense for a file browsed to outside every registered
        // root, so the menu simply does not offer them there.
        std::string relPath;
        if (!assetRelativePath(mCurrentDirectory / entry.name, relPath))
            return;
        const usize dot = entry.name.find_last_of('.');
        const std::string stem = dot == std::string::npos ? entry.name : entry.name.substr(0, dot);

        const auto convert = [this, relPath]()
        {
            // Conversion is deliberately kept on the decoded CPU-side data:
            // no MeshHandle/GameObject is created, so this action never loads
            // the source asset into the engine. The native files are written
            // beside the source, preserving the asset browser's layout.
            const std::string source = FileSystem::getSingleton().resolve(relPath);
            const std::string base = source.empty()
                                         ? app().assetBrowserRoot() + "/" + relPath
                                         : source;
            const usize sourceDot = base.find_last_of('.');
            const std::string outputBase = sourceDot == std::string::npos
                                               ? base
                                               : base.substr(0, sourceDot);
            const std::string meshOutput = outputBase + ".rmesh";

            MeshData meshData;
            if (!Assets().importMeshFileData(relPath, meshData))
            {
                Log::error("AssetsPanel: conversion failed while importing '%s'", relPath.c_str());
                app().toasts().error("Could not convert " + FileSystem::fileName(relPath));
                return;
            }

            std::string skeletonOutput;
            Skeleton skeleton;
            // Animated formats carry their rig in the source file. Saving it
            // next to the converted mesh also makes the result usable by the
            // Animator workflow without asking the renderer to load anything.
            if (Assets().importSkeleton(relPath, skeleton) && skeleton.boneCount() > 0)
            {
                skeletonOutput = outputBase + ".rskel";
                if (!RadionSkeletonIO::saveSkeleton(skeletonOutput, skeleton))
                {
                    Log::error("AssetsPanel: could not write skeleton '%s'", skeletonOutput.c_str());
                    app().toasts().error("Could not write " + FileSystem::fileName(skeletonOutput));
                    return;
                }
            }

            if (!Assets().saveMesh(meshData, meshOutput, skeletonOutput))
            {
                Log::error("AssetsPanel: could not write mesh '%s'", meshOutput.c_str());
                app().toasts().error("Could not write " + FileSystem::fileName(meshOutput));
                return;
            }

            const std::string materialOutput = outputBase + ".material";
            if (!meshData.materials.empty() &&
                !MaterialManager::getSingleton().save(materialOutput,
                                                      Assets().materialsForSidecar(meshData)))
            {
                Log::error("AssetsPanel: could not write material '%s'", materialOutput.c_str());
                app().toasts().error("Could not write " + FileSystem::fileName(materialOutput));
                return;
            }

            mEntriesDirty = true;
            Log::info("AssetsPanel: converted '%s' to '%s'%s", relPath.c_str(), meshOutput.c_str(),
                      skeletonOutput.empty() ? "" : " (skeleton saved beside it)");
            app().toasts().success("Converted to " + FileSystem::fileName(meshOutput));
        };

        // .fbx/.gltf/.glb/.b3d/.ms3d clips carry animation stacks that only
        // mean something against a rig - the same .rskel a mesh's own
        // Convert writes beside it. This does what dropping the clip on an
        // Animator's clip list in the Inspector does (ensureAnimationFile()
        // there), offered here as a one-click action per file so a whole
        // Mixamo-style pack can be run through the folder one at a time
        // instead of opening an Animator for each clip.
        const bool animatable = extension == "fbx" || extension == "gltf" || extension == "glb" ||
                                extension == "b3d" || extension == "ms3d";
        // keepRootMotion mirrors loadFbxAnimation's own parameter
        // (FbxImporter.h) - false pins the functional root's horizontal
        // position to the bind pose (only vertical bob survives), turning a
        // locomotion clip like "walk"/"run" into one that plays in place
        // instead of dragging the object across the scene. Rotations are
        // untouched either way.
        const auto generateAnimation = [this, relPath](bool keepRootMotion)
        {
            std::string skeletonPath;
            for (const FileSystem::DirEntry& sibling : mEntries)
            {
                if (sibling.isDirectory || extensionOf(sibling.name) != "rskel")
                    continue;
                if (!assetRelativePath(mCurrentDirectory / sibling.name, skeletonPath))
                    continue;
                break;
            }
            if (skeletonPath.empty())
            {
                Log::error("AssetsPanel: no .rskel next to '%s' to bind the animation against",
                          relPath.c_str());
                app().toasts().error("No .rskel in this folder - convert the mesh first");
                return;
            }

            Skeleton skeleton;
            if (!RadionSkeletonIO::loadSkeleton(skeletonPath, skeleton))
            {
                Log::error("AssetsPanel: could not load skeleton '%s'", skeletonPath.c_str());
                app().toasts().error("Could not read " + FileSystem::fileName(skeletonPath));
                return;
            }

            AnimationClip clip;
            if (!Assets().importAnimation(relPath, skeleton, clip, keepRootMotion))
            {
                Log::error("AssetsPanel: could not import animation from '%s'", relPath.c_str());
                app().toasts().error("Could not read animation from " +
                                     FileSystem::fileName(relPath));
                return;
            }

            // Every clip in a Mixamo pack carries the exact same in-file name
            // ("mixamo.com") - loading a walk, an attack and an idle into
            // one Animator under that name would make play() unable to
            // tell them apart, only ever finding the first. The source
            // file's own name is unique by construction (it is a file in
            // this folder), so it becomes the clip's name here regardless
            // of what the FBX itself called it - Animator::play() then
            // takes exactly the stem this file's icon already shows.
            const std::string relStem = FileSystem::baseName(relPath);
            clip.setName(relStem);

            const std::string source = FileSystem::getSingleton().resolve(relPath);
            const std::string base =
                source.empty() ? app().assetBrowserRoot() + "/" + relPath : source;
            const usize sourceDot = base.find_last_of('.');
            const std::string animOutput =
                (sourceDot == std::string::npos ? base : base.substr(0, sourceDot)) + ".ranim";

            if (!RadionSkeletonIO::saveAnimation(animOutput, skeleton, clip))
            {
                Log::error("AssetsPanel: could not write animation '%s'", animOutput.c_str());
                app().toasts().error("Could not write " + FileSystem::fileName(animOutput));
                return;
            }

            mEntriesDirty = true;
            Log::info("AssetsPanel: generated '%s' from '%s' against '%s'", animOutput.c_str(),
                      relPath.c_str(), skeletonPath.c_str());
            app().toasts().success("Generated " + FileSystem::fileName(animOutput));
        };

        if (!ImGui::BeginPopupContextItem())
            return;
        if (mesh)
        {
            const char* label = isNativeMeshAsset(extension) ? ICON_MDI_FILE_IMPORT " Load"
                                                              : ICON_MDI_IMPORT " Import";
            if (ImGui::MenuItem(label))
            {
                mImportPath = relPath;
                mImportName = stem;
                mImportTranslation = glm::vec3(0.0f);
                mImportRotationEuler = glm::vec3(0.0f);
                mImportScale = 1.0f;
                mImportPending = true;
            }
            if (!isNativeMeshAsset(extension) && ImGui::MenuItem(ICON_MDI_SWAP_HORIZONTAL " Convert"))
                convert();
            if (animatable && ImGui::MenuItem(ICON_MDI_RUN " Generate Animation"))
                generateAnimation(true);
            if (animatable && ImGui::MenuItem(ICON_MDI_RUN_FAST " Generate Animation (in place)"))
                generateAnimation(false);
            if (extension == "rmesh" && ImGui::MenuItem(ICON_MDI_EXPORT " Export OBJ"))
            {
                const std::string source = FileSystem::getSingleton().resolve(relPath);
                const std::string base = source.empty()
                                             ? app().assetBrowserRoot() + "/" + relPath
                                             : source;
                const usize sourceDot = base.find_last_of('.');
                const std::string outputBase = sourceDot == std::string::npos
                                                   ? base
                                                   : base.substr(0, sourceDot);
                const std::string objOutput = outputBase + ".obj";
                if (exportRmeshToObj(relPath, objOutput))
                {
                    mEntriesDirty = true;
                    Log::info("AssetsPanel: exported '%s' to '%s' and '%s.mtl'", relPath.c_str(),
                              objOutput.c_str(), outputBase.c_str());
                    app().toasts().success("Exported " + FileSystem::fileName(objOutput));
                }
                else
                {
                    Log::error("AssetsPanel: could not export '%s' to OBJ", relPath.c_str());
                    app().toasts().error("Could not export " + FileSystem::fileName(relPath) +
                                         " to OBJ");
                }
            }
        }
        if (image)
        {
            if (ImGui::MenuItem(ICON_MDI_IMAGE_FILTER_HDR " Generate Normal Map..."))
            {
                mGenerateSourcePath = relPath;
                mGenerateKind = GenerateKind::Normal;
                mGenerateDialog.Open(ImGuiFileDialog::Mode::SaveFile,
                                     app().settings().lastSaveDirectory.empty()
                                         ? app().assetBrowserRoot()
                                         : app().settings().lastSaveDirectory,
                                     stem + "_normal.png");
            }
            if (ImGui::MenuItem(ICON_MDI_TERRAIN " Generate Heightmap..."))
            {
                mGenerateSourcePath = relPath;
                mGenerateKind = GenerateKind::Height;
                mGenerateDialog.Open(ImGuiFileDialog::Mode::SaveFile,
                                     app().settings().lastSaveDirectory.empty()
                                         ? app().assetBrowserRoot()
                                         : app().settings().lastSaveDirectory,
                                     stem + "_height.png");
            }
        }
        if (prefab && ImGui::MenuItem(ICON_MDI_PACKAGE_VARIANT " Instantiate"))
            instantiatePrefab(relPath);
        if (script && ImGui::MenuItem(ICON_MDI_CONTENT_COPY " Copy Script Path"))
            ImGui::SetClipboardText(relPath.c_str());
        ImGui::Separator();
        if (ImGui::MenuItem(ICON_MDI_DELETE " Delete..."))
        {
            mDeletePath = relPath;
            mDeletePending = true;
        }
        ImGui::EndPopup();
    };

    if (mViewMode == ViewMode::List)
    {
        ImGui::Indent(14.0f);
        // Clipped by row: a folder the size of a texture library (the bistro
        // set runs to several hundred files) would otherwise rebuild every
        // row's widgets every frame regardless of what's actually scrolled
        // into view - that is what pegged the CPU the moment this panel had
        // focus.
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(mEntries.size()));
        while (clipper.Step())
        {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
            {
                const FileSystem::DirEntry& entry = mEntries[static_cast<usize>(i)];
                ImGui::PushID(entry.name.c_str());
                ImGui::PushStyleColor(ImGuiCol_Text, iconColorForAsset(entry));
                ImGui::TextUnformatted(iconForAsset(entry));
                ImGui::PopStyleColor();
                ImGui::SameLine();
                const bool clicked = ImGui::Selectable(entry.name.c_str());
                const bool doubleClicked = ImGui::IsItemHovered() &&
                                           ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
                if (clicked || doubleClicked)
                    openEntry(entry, doubleClicked);
                dragEntry(entry);
                contextMenu(entry);
                ImGui::PopID();
            }
        }
        ImGui::Unindent(14.0f);
        ImGui::EndChild();
        return;
    }

    if (mViewMode == ViewMode::Details)
    {
        if (ImGui::BeginTable("assets.details", 3,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Size", ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableHeadersRow();
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(mEntries.size()));
            while (clipper.Step())
            {
                for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
                {
                    const FileSystem::DirEntry& entry = mEntries[static_cast<usize>(row)];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushID(entry.name.c_str());
                    ImGui::Indent(14.0f);
                    ImGui::PushStyleColor(ImGuiCol_Text, iconColorForAsset(entry));
                    ImGui::TextUnformatted(iconForAsset(entry));
                    ImGui::PopStyleColor();
                    ImGui::SameLine();
                    const bool clicked = ImGui::Selectable(entry.name.c_str(), false,
                                                           ImGuiSelectableFlags_SpanAllColumns);
                    const bool doubleClicked = ImGui::IsItemHovered() &&
                                               ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
                    if (clicked || doubleClicked)
                        openEntry(entry, doubleClicked);
                    dragEntry(entry);
                    contextMenu(entry);
                    ImGui::Unindent(14.0f);
                    ImGui::PopID();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(entry.isDirectory ? "Folder"
                                                             : extensionOf(entry.name).c_str());
                    ImGui::TableSetColumnIndex(2);
                    if (!entry.isDirectory)
                        ImGui::Text("%llu", static_cast<unsigned long long>(entry.size));
                }
            }
            ImGui::EndTable();
        }
        directoryContextMenu();
        ImGui::EndChild();
        if (mOpenCreateFolderPopup)
        {
            mOpenCreateFolderPopup = false;
            ImGui::OpenPopup("Create Folder");
        }
        if (mOpenCreateScriptPopup)
        {
            mOpenCreateScriptPopup = false;
            ImGui::OpenPopup("Create Script");
        }
        drawCreateFolderPopup();
        drawCreateScriptPopup();
        return;
    }

    const f32 cellWidth = mThumbnailSize + 16.0f;
    const f32 availableWidth = ImGui::GetContentRegionAvail().x;
    int columns = static_cast<int>(availableWidth / cellWidth);
    if (columns < 1)
        columns = 1;

    GPU& gpu = app().engine().getGPU();

    // Row-clipped, not one flat loop over mEntries: a texture library the
    // size of the bistro set runs to several hundred files, and rebuilding
    // every cell's ImageButton and draw call every frame regardless of
    // scroll is what pegged the CPU the moment this panel had focus. Row
    // height is an estimate (the wrapped name can take one or two lines) -
    // close enough for the clipper's stepping, not pixel-exact.
    const int rows = (static_cast<int>(mEntries.size()) + columns - 1) / columns;
    const f32 rowHeight =
        mThumbnailSize + ImGui::GetTextLineHeightWithSpacing() * 2.0f + ImGui::GetStyle().ItemSpacing.y;
    ImGuiListClipper clipper;
    clipper.Begin(rows, rowHeight);
    while (clipper.Step())
    {
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
        {
            for (int col = 0; col < columns; ++col)
            {
                const usize index =
                    static_cast<usize>(row) * static_cast<usize>(columns) + static_cast<usize>(col);
                if (index >= mEntries.size())
                    break;
                if (col > 0)
                    ImGui::SameLine();

                const FileSystem::DirEntry& entry = mEntries[index];
                ImGui::PushID(entry.name.c_str());
                ImGui::BeginGroup();

                std::string relPath;
                const bool isImage = !entry.isDirectory && isImageAsset(extensionOf(entry.name)) &&
                                     assetRelativePath(mCurrentDirectory / entry.name, relPath);
                const TextureHandle thumbnail = isImage ? thumbnailFor(relPath) : TextureHandle();
                const ImVec2 iconSize(mThumbnailSize, mThumbnailSize);

                bool clicked = false;
                if (thumbnail.valid())
                {
                    const ImTextureID textureId = static_cast<ImTextureID>(
                        static_cast<uintptr_t>(gpu.nativeTextureId(thumbnail)));
                    clicked = ImGui::ImageButton("##thumbnail", textureId, iconSize,
                                                 ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
                    if (ImGui::IsItemHovered())
                    {
                        // Bigger read of the same texture, not a second load -
                        // the point is seeing detail the grid cell is too
                        // small for, the same job Lumos's ResourcePanel hover
                        // tooltip does.
                        ImGui::BeginTooltip();
                        constexpr f32 previewSize = 256.0f;
                        ImGui::Image(textureId, ImVec2(previewSize, previewSize),
                                    ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
                        ImGui::TextUnformatted(entry.name.c_str());
                        ImGui::EndTooltip();
                    }
                }
                else
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, iconColorForAsset(entry));
                    clicked = ImGui::Button(iconForAsset(entry), iconSize);
                    ImGui::PopStyleColor();
                }
                if (!entry.isDirectory)
                {
                    std::string extension = extensionOf(entry.name);
                    std::transform(extension.begin(), extension.end(), extension.begin(),
                                   [](unsigned char character)
                                   {
                                       return static_cast<char>(std::toupper(character));
                                   });
                    if (!extension.empty())
                    {
                        ImDrawList* draw = ImGui::GetWindowDrawList();
                        const ImVec2 itemMax = ImGui::GetItemRectMax();
                        const ImVec2 textSize = ImGui::CalcTextSize(extension.c_str());
                        const ImVec2 badgeMin(itemMax.x - textSize.x - 8.0f,
                                              itemMax.y - textSize.y - 6.0f);
                        draw->AddRectFilled(badgeMin, itemMax, IM_COL32(20, 24, 30, 210), 3.0f);
                        draw->AddText(ImVec2(badgeMin.x + 4.0f, badgeMin.y + 2.0f),
                                      IM_COL32(235, 238, 245, 255), extension.c_str());
                    }
                }
                const bool doubleClicked = ImGui::IsItemHovered() &&
                                           ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
                if (clicked || doubleClicked)
                    openEntry(entry, doubleClicked);
                dragEntry(entry);
                contextMenu(entry);

                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + mThumbnailSize);
                ImGui::TextWrapped("%s", entry.name.c_str());
                ImGui::PopTextWrapPos();

                ImGui::EndGroup();
                ImGui::PopID();
            }
        }
    }

    if (!mPendingNavigation.empty())
    {
        navigateTo(mPendingNavigation);
        mPendingNavigation.clear();
    }

    directoryContextMenu();
    ImGui::EndChild();
    if (mOpenCreateFolderPopup)
    {
        mOpenCreateFolderPopup = false;
        ImGui::OpenPopup("Create Folder");
    }
    if (mOpenCreateScriptPopup)
    {
        mOpenCreateScriptPopup = false;
        ImGui::OpenPopup("Create Script");
    }
    drawCreateFolderPopup();
    drawCreateScriptPopup();
}

} // namespace Radion

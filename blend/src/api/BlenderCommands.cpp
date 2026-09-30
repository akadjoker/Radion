#include "PCH.h"
#include "api/BlenderCommands.h"

#include "AssetManager.h"
#include "BlenderApplication.h"
#include "FileSystem.h"
#include "mesh/MeshTopology.h"
#include "Pixmap.h"
#include "ProceduralShapes.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <fstream>
#include <iterator>

using namespace Radion;

namespace Radion::BlenderApi
{

namespace
{
// Bounds on what one call may ask for. Arguments arrive from outside the
// process; none of these limits is a modelling limit, they keep a typo (or a
// model run away with itself) from allocating gigabytes or hanging the editor.
constexpr size_t kMaxCustomVertices = 200000;
constexpr size_t kMaxCustomIndices = 600000;
constexpr double kMaxCoordinate = 100000.0;
constexpr size_t kMaxSelectionIndices = 1000000;
constexpr size_t kMaxListedIndices = 500;
constexpr size_t kMaxDumpVertices = 2000;
constexpr int kMaxCaptureSize = 1920;

// ---------------------------------------------------------------- errors

[[noreturn]] void invalid(const std::string& message)
{
    throw CommandError(CommandStatus::InvalidParams, message);
}

[[noreturn]] void failed(const std::string& message)
{
    throw CommandError(CommandStatus::Failed, message);
}

// ------------------------------------------------------------ JSON shapes

Json vec3Json(const glm::vec3& v)
{
    return Json::array({v.x, v.y, v.z});
}

Json boundsJson(const AABB& box)
{
    if (box.empty())
        return Json(nullptr);
    return {{"min", vec3Json(box.min)},
            {"max", vec3Json(box.max)},
            {"size", vec3Json(box.max - box.min)},
            {"center", vec3Json(box.center())}};
}

// Schema builders. The description of every argument is what a model reads to
// decide how to call the command, so they are written for that reader.
Json numberSchema(const char* description)
{
    return {{"type", "number"}, {"description", description}};
}

Json integerSchema(const char* description)
{
    return {{"type", "integer"}, {"description", description}};
}

Json stringSchema(const char* description)
{
    return {{"type", "string"}, {"description", description}};
}

Json boolSchema(const char* description)
{
    return {{"type", "boolean"}, {"description", description}};
}

Json vec3Schema(const char* description)
{
    return {{"type", "array"},
            {"items", {{"type", "number"}}},
            {"minItems", 3},
            {"maxItems", 3},
            {"description", description}};
}

Json choiceSchema(const char* description, const std::vector<std::string>& values)
{
    return {{"type", "string"}, {"enum", values}, {"description", description}};
}

Json objectSchema(Json properties, const std::vector<std::string>& required = {})
{
    Json schema = {{"type", "object"}, {"properties", std::move(properties)}};
    if (!required.empty())
        schema["required"] = required;
    return schema;
}

Json partRefSchema()
{
    return {{"description", "A part, by its index (integer) or by its name (string)."},
            {"oneOf", Json::array({{{"type", "integer"}}, {{"type", "string"}}})}};
}

// The arguments every command that places a new part takes.
void addPlacementProperties(Json& properties)
{
    properties["position"] = vec3Schema("Where the part's origin goes. Default [0,0,0]. +Y is up.");
    properties["rotation"] = vec3Schema(
        "Euler angles in degrees, applied about X first, then Y, then Z. Default [0,0,0].");
    properties["scale"] = {{"description", "Number (uniform) or [x,y,z]. Default 1. Non-uniform "
                                          "scale turns a sphere into an ellipsoid."},
                           {"oneOf", Json::array({{{"type", "number"}},
                                                  {{"type", "array"},
                                                   {"items", {{"type", "number"}}},
                                                   {"minItems", 3},
                                                   {"maxItems", 3}}})}};
}

void addStyleProperties(Json& properties)
{
    properties["name"] =
        stringSchema("Name of the part (its material name). Use it later to refer to the part.");
    properties["color"] = {
        {"description", "Base colour as it should look (sRGB): \"#rrggbb\" or [r,g,b] / [r,g,b,a] with components 0..1."},
        {"oneOf", Json::array({{{"type", "string"}},
                               {{"type", "array"},
                                {"items", {{"type", "number"}}},
                                {"minItems", 3},
                                {"maxItems", 4}}})}};
    properties["roughness"] = numberSchema("Surface roughness 0 (mirror) to 1 (matte).");
    properties["metallic"] = numberSchema("Metalness 0 (paint/plastic) to 1 (bare metal).");
}

// -------------------------------------------------------- argument helpers

void requireFiniteRange(const char* name, const std::vector<double>& values, double limit)
{
    for (const double value : values)
    {
        if (std::abs(value) > limit)
            invalid(std::string("argument '") + name + "' has a value beyond +/-" +
                    std::to_string(static_cast<long long>(limit)));
    }
}

glm::vec3 toVec3(const std::vector<double>& v)
{
    return glm::vec3(static_cast<f32>(v[0]), static_cast<f32>(v[1]), static_cast<f32>(v[2]));
}

glm::vec3 vec3Arg(const CommandArgs& args, const char* name, const glm::vec3& fallback)
{
    if (!args.has(name))
        return fallback;
    const std::vector<double> values = args.requireNumbers(name, 3);
    requireFiniteRange(name, values, kMaxCoordinate);
    return toVec3(values);
}

// translate * rotateZ * rotateY * rotateX * scale: scale and rotate the part
// about its own origin, then move it. The order the arguments are documented in.
glm::mat4 composeTransform(const glm::vec3& position, const glm::vec3& rotationDegrees,
                           const glm::vec3& scale)
{
    glm::mat4 matrix = glm::translate(glm::mat4(1.0f), position);
    matrix = glm::rotate(matrix, glm::radians(rotationDegrees.z), glm::vec3(0.0f, 0.0f, 1.0f));
    matrix = glm::rotate(matrix, glm::radians(rotationDegrees.y), glm::vec3(0.0f, 1.0f, 0.0f));
    matrix = glm::rotate(matrix, glm::radians(rotationDegrees.x), glm::vec3(1.0f, 0.0f, 0.0f));
    return glm::scale(matrix, scale);
}

glm::vec3 scaleArg(const CommandArgs& args, const char* name)
{
    const std::vector<double> values = args.numbersOrScalar(name, 3, {1.0, 1.0, 1.0});
    requireFiniteRange(name, values, kMaxCoordinate);
    for (const double value : values)
    {
        // A zero scale flattens the part to nothing and makes the normal matrix
        // singular; a mirror is a -1, not a 0.
        if (std::abs(value) < 1.0e-6)
            invalid(std::string("argument '") + name + "' must not contain 0");
    }
    return toVec3(values);
}

glm::mat4 placementArg(const CommandArgs& args)
{
    return composeTransform(vec3Arg(args, "position", glm::vec3(0.0f)),
                            vec3Arg(args, "rotation", glm::vec3(0.0f)), scaleArg(args, "scale"));
}

bool parseHexColor(const std::string& text, glm::vec4& out)
{
    if (text.size() < 2 || text[0] != '#')
        return false;
    const std::string digits = text.substr(1);
    if (digits.size() != 3 && digits.size() != 6 && digits.size() != 8)
        return false;

    std::vector<int> nibbles;
    for (const char c : digits)
    {
        if (!std::isxdigit(static_cast<unsigned char>(c)))
            return false;
        nibbles.push_back(std::stoi(std::string(1, c), nullptr, 16));
    }

    std::vector<f32> channels;
    if (digits.size() == 3)
    {
        for (const int nibble : nibbles)
            channels.push_back(static_cast<f32>(nibble * 17) / 255.0f);
    }
    else
    {
        for (size_t i = 0; i + 1 < nibbles.size(); i += 2)
            channels.push_back(static_cast<f32>(nibbles[i] * 16 + nibbles[i + 1]) / 255.0f);
    }
    out = glm::vec4(channels[0], channels[1], channels[2], channels.size() > 3 ? channels[3] : 1.0f);
    return true;
}

// Colours cross the API as sRGB - what a person (or a colour picker, or a hex
// code) means - and are stored linear, which is what the renderer multiplies.
f32 srgbToLinear(f32 c)
{
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

f32 linearToSrgb(f32 c)
{
    c = glm::clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

glm::vec4 srgbToLinear(const glm::vec4& c)
{
    return glm::vec4(srgbToLinear(c.r), srgbToLinear(c.g), srgbToLinear(c.b), c.a);
}

std::string hexColor(const glm::vec4& linear)
{
    char buffer[16];
    auto byte = [](f32 c) { return static_cast<int>(std::lround(linearToSrgb(c) * 255.0f)); };
    std::snprintf(buffer, sizeof(buffer), "#%02x%02x%02x", byte(linear.r), byte(linear.g), byte(linear.b));
    return buffer;
}

bool readColor(const CommandArgs& args, glm::vec4& out)
{
    const Json* value = args.raw("color");
    if (!value)
        return false;

    if (value->is_string())
    {
        if (!parseHexColor(value->get<std::string>(), out))
            invalid("argument 'color' must look like \"#rrggbb\"");
        out = srgbToLinear(out);
        return true;
    }

    if (value->is_array() && (value->size() == 3 || value->size() == 4))
    {
        glm::vec4 color(1.0f);
        for (size_t i = 0; i < value->size(); ++i)
        {
            const Json& item = (*value)[i];
            if (!item.is_number() || item.get<double>() < 0.0 || item.get<double>() > 1.0)
                invalid("argument 'color' components must be numbers from 0 to 1");
            color[static_cast<glm::length_t>(i)] = item.get<f32>();
        }
        out = srgbToLinear(color);
        return true;
    }

    invalid("argument 'color' must be \"#rrggbb\" or an array of 3 or 4 numbers from 0 to 1");
}

BlenderApplication::PartStyle styleArg(const CommandArgs& args)
{
    BlenderApplication::PartStyle style;
    style.name = args.string("name", "");
    style.hasColor = readColor(args, style.color);
    if (args.has("roughness"))
    {
        style.hasRoughness = true;
        style.roughness = static_cast<f32>(args.number("roughness", 0.5, 0.0, 1.0));
    }
    if (args.has("metallic"))
    {
        style.hasMetallic = true;
        style.metallic = static_cast<f32>(args.number("metallic", 0.0, 0.0, 1.0));
    }
    return style;
}

// ----------------------------------------------------------- mesh helpers

MeshData& requireMesh(BlenderApplication& app)
{
    MeshData* mesh = app.currentMeshData();
    if (!mesh || mesh->positions.empty())
        failed("the document has no mesh yet - add a primitive or load a file first");
    return *mesh;
}

std::string partName(const MeshData& mesh, u32 index)
{
    const u32 slot = mesh.submeshes[index].materialSlot;
    if (slot < mesh.materials.size() && !mesh.materials[slot].name.empty())
        return mesh.materials[slot].name;
    return "part" + std::to_string(index);
}

u32 resolvePart(BlenderApplication& app, const CommandArgs& args, const char* argName = "part")
{
    const Json* value = args.raw(argName);
    if (!value)
        invalid(std::string("argument '") + argName + "' is required");

    const MeshData& mesh = requireMesh(app);
    const u32 count = static_cast<u32>(mesh.submeshes.size());

    if (value->is_number_integer())
    {
        const long long index = value->get<long long>();
        if (index < 0 || index >= static_cast<long long>(count))
            invalid(std::string("argument '") + argName + "': no part " + std::to_string(index) +
                    " (the mesh has " + std::to_string(count) + " parts)");
        return static_cast<u32>(index);
    }

    if (value->is_string())
    {
        const std::string wanted = value->get<std::string>();
        s32 found = -1;
        for (u32 i = 0; i < count; ++i)
        {
            if (partName(mesh, i) != wanted)
                continue;
            if (found >= 0)
                invalid("several parts are named '" + wanted + "' - use the index instead");
            found = static_cast<s32>(i);
        }
        if (found < 0)
            invalid("no part named '" + wanted + "'");
        return static_cast<u32>(found);
    }

    invalid(std::string("argument '") + argName + "' must be a part index or name");
}

Json partJson(BlenderApplication& app, const MeshData& mesh, u32 index)
{
    const SubMesh& submesh = mesh.submeshes[index];
    Json part = {{"index", index},
                 {"name", partName(mesh, index)},
                 {"triangles", submesh.indexCount / 3},
                 {"visible", app.isSubmeshVisible(index)},
                 {"bounds", boundsJson(submesh.bounds)}};
    if (submesh.materialSlot < mesh.materials.size())
    {
        const Material& material = mesh.materials[submesh.materialSlot];
        part["color"] = hexColor(material.params.baseColor);
        part["roughness"] = material.params.surface.x;
        part["metallic"] = material.params.surface.y;
    }
    return part;
}

Json selectionJson(BlenderApplication& app)
{
    BlenderSelection& selection = app.selection();
    Json json = {{"mode", selection.mode() == BlenderSelection::SelectionMode::Face ? "face"
                          : selection.mode() == BlenderSelection::SelectionMode::Edge ? "edge"
                                                                                      : "vertex"},
                 {"vertexCount", selection.selectedVertexCount()},
                 {"faceCount", selection.selectedFaceCount()},
                 {"edgeCount", selection.selectedEdgeCount()}};

    const MeshData* mesh = app.currentMeshData();
    if (mesh && selection.selectedVertexCount() > 0)
    {
        AABB box;
        for (const u32 index : selection.selectedVertices())
        {
            if (index < mesh->positions.size())
                box.expand(mesh->positions[index]);
        }
        json["vertexBounds"] = boundsJson(box);

        Json listed = Json::array();
        for (const u32 index : selection.selectedVertices())
        {
            if (listed.size() >= kMaxListedIndices)
                break;
            listed.push_back(index);
        }
        json["vertices"] = listed;
    }
    if (selection.selectedFaceCount() > 0)
    {
        Json listed = Json::array();
        for (const u32 index : selection.selectedFaces())
        {
            if (listed.size() >= kMaxListedIndices)
                break;
            listed.push_back(index);
        }
        json["faces"] = listed;
    }
    if (selection.selectedEdgeCount() > 0)
    {
        // [a, b] vertex pairs - the same form `select` takes.
        Json listed = Json::array();
        AABB box;
        for (const u64 key : selection.selectedEdges())
        {
            const u32 a = static_cast<u32>(key >> 32);
            const u32 b = static_cast<u32>(key & 0xFFFFFFFFu);
            if (mesh && a < mesh->positions.size() && b < mesh->positions.size())
            {
                box.expand(mesh->positions[a]);
                box.expand(mesh->positions[b]);
            }
            if (listed.size() < kMaxListedIndices)
                listed.push_back(Json::array({a, b}));
        }
        json["edges"] = listed;
        json["edgeBounds"] = boundsJson(box);
    }
    return json;
}

Json statusJson(BlenderApplication& app)
{
    const MeshData* mesh = app.currentMeshData();
    const bool hasMesh = mesh && !mesh->positions.empty();

    Json status = {{"hasMesh", hasMesh},
                   {"dirty", app.isDirty()},
                   {"canUndo", app.canUndo()},
                   {"canRedo", app.canRedo()},
                   {"lastFile", app.settings().general().lastOpenedMesh},
                   {"selection", selectionJson(app)}};

    status["hiddenTriangles"] = app.hiddenFaceCount();

    if (hasMesh)
    {
        status["vertices"] = mesh->positions.size();
        status["triangles"] = mesh->indices.size() / 3;
        status["bounds"] = boundsJson(mesh->bounds);
        Json parts = Json::array();
        for (u32 i = 0; i < static_cast<u32>(mesh->submeshes.size()); ++i)
            parts.push_back(partJson(app, *mesh, i));
        status["parts"] = parts;
    }

    if (app.hasSkeleton())
        status["skeleton"] = {{"bones", app.skeleton().boneCount()}};

    Json clips = Json::array();
    for (usize i = 0; i < app.animationClipCount(); ++i)
        clips.push_back({{"index", i},
                         {"name", app.animationClip(i).name()},
                         {"duration", app.animationClip(i).duration()}});
    status["animation"] = {{"clips", clips},
                           {"active", app.activeAnimationClip()},
                           {"frame", app.currentFrame()},
                           {"totalFrames", app.totalFrames()},
                           {"playing", app.isPlaying()}};
    return status;
}

CommandResult result(Json data)
{
    CommandResult out;
    out.data = std::move(data);
    return out;
}

// What a geometry-adding command reports back: enough to place the next part
// relative to this one without a separate query.
CommandResult partAdded(BlenderApplication& app, s32 submesh)
{
    const MeshData& mesh = requireMesh(app);
    if (submesh < 0 || static_cast<usize>(submesh) >= mesh.submeshes.size())
        failed("the part was added but could not be found afterwards");
    Json data = partJson(app, mesh, static_cast<u32>(submesh));
    data["totalVertices"] = mesh.positions.size();
    data["totalTriangles"] = mesh.indices.size() / 3;
    return result(std::move(data));
}

// Vertices to edit for the current selection (see BlenderApplication::editVertices).
std::vector<u32> selectedVertexSet(BlenderApplication& app)
{
    requireMesh(app);
    return app.editVertices();
}

// ----------------------------------------------------------------- base64

std::string base64(const std::string& bytes)
{
    static const char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    for (size_t i = 0; i < bytes.size(); i += 3)
    {
        const unsigned a = static_cast<unsigned char>(bytes[i]);
        const unsigned b = i + 1 < bytes.size() ? static_cast<unsigned char>(bytes[i + 1]) : 0u;
        const unsigned c = i + 2 < bytes.size() ? static_cast<unsigned char>(bytes[i + 2]) : 0u;
        const unsigned triple = (a << 16) | (b << 8) | c;
        out.push_back(kAlphabet[(triple >> 18) & 63]);
        out.push_back(kAlphabet[(triple >> 12) & 63]);
        out.push_back(i + 1 < bytes.size() ? kAlphabet[(triple >> 6) & 63] : '=');
        out.push_back(i + 2 < bytes.size() ? kAlphabet[triple & 63] : '=');
    }
    return out;
}

// PNG bytes for RGBA pixels. The image writer in the engine only knows how to
// write to a file, so the picture takes a short trip through one.
std::string encodePng(const std::vector<u8>& rgba, int width, int height)
{
    const std::string path =
        FileSystem::getSingleton().prefPath("Radion", "Blender") + "api_capture.png";
    if (path.size() <= 4)
        failed("no writable folder for the screenshot");

    Pixmap pixmap(width, height, 4, const_cast<u8*>(rgba.data()));
    if (!pixmap.save(path.c_str()))
        failed("could not encode the screenshot");

    std::ifstream file(path, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    std::remove(path.c_str());
    if (bytes.empty())
        failed("could not read the screenshot back");
    return bytes;
}

CameraView viewFromName(const std::string& name)
{
    if (name == "top")
        return CameraView::Top;
    if (name == "bottom")
        return CameraView::Bottom;
    if (name == "front")
        return CameraView::Front;
    if (name == "back")
        return CameraView::Back;
    if (name == "left")
        return CameraView::Left;
    if (name == "right")
        return CameraView::Right;
    return CameraView::Perspective;
}

} // namespace

// ============================================================ registration

void registerBlenderCommands(CommandRegistry& registry, BlenderApplication& app)
{
    BlenderApplication* editor = &app;

    auto add = [&registry](const char* name, const char* description, Json schema, bool readOnly,
                           CommandHandler handler)
    {
        CommandDef def;
        def.name = name;
        def.description = description;
        def.inputSchema = std::move(schema);
        def.readOnly = readOnly;
        def.handler = std::move(handler);
        registry.add(std::move(def));
    };

    // ------------------------------------------------------------ inspect

    add("get_status",
        "Everything about the document in one call: mesh size and bounds, the list of parts "
        "(index, name, triangles, colour, bounds), the selection, animation clips and whether "
        "undo/redo are available. Call this first, and again after edits to check the result.",
        objectSchema(Json::object()), true,
        [editor](const CommandArgs&) { return result(statusJson(*editor)); });

    {
        Json properties = {{"part", partRefSchema()},
                           {"max_vertices",
                            integerSchema("Most vertices to return (default 500, at most 2000).")}};
        add("get_mesh_data",
            "Raw geometry of one part (or the whole mesh when 'part' is omitted): vertex positions "
            "and triangles as index triples into that list; 'vertexIds' gives each entry's editor "
            "vertex index. For checking shapes; large meshes are cut off.",
            objectSchema(properties), true,
            [editor](const CommandArgs& args)
            {
                const MeshData& mesh = requireMesh(*editor);
                const size_t limit = static_cast<size_t>(
                    args.integer("max_vertices", 500, 1, static_cast<long long>(kMaxDumpVertices)));

                std::vector<u32> indices;
                std::string scope = "mesh";
                if (args.has("part"))
                {
                    const u32 part = resolvePart(*editor, args);
                    const SubMesh& submesh = mesh.submeshes[part];
                    indices.assign(mesh.indices.begin() + submesh.indexOffset,
                                   mesh.indices.begin() + submesh.indexOffset + submesh.indexCount);
                    scope = partName(mesh, part);
                }
                else
                {
                    indices = mesh.indices;
                }

                // Renumber to the vertices actually used, in first-use order, so
                // a part of a big mesh comes out as a small self-contained list.
                std::unordered_map<u32, u32> remap;
                Json positions = Json::array();
                Json vertexIds = Json::array();
                Json triangles = Json::array();
                bool truncated = false;
                for (usize i = 0; i + 2 < indices.size(); i += 3)
                {
                    u32 local[3];
                    size_t fresh = 0;
                    for (int corner = 0; corner < 3; ++corner)
                        if (!remap.count(indices[i + corner]))
                            ++fresh;
                    if (remap.size() + fresh > limit)
                    {
                        truncated = true;
                        break;
                    }
                    for (int corner = 0; corner < 3; ++corner)
                    {
                        const u32 original = indices[i + corner];
                        auto found = remap.find(original);
                        if (found == remap.end())
                        {
                            found = remap.emplace(original, static_cast<u32>(remap.size())).first;
                            vertexIds.push_back(original);
                            const glm::vec3& p = mesh.positions[original];
                            positions.push_back(Json::array({std::round(p.x * 10000.0f) / 10000.0f,
                                                             std::round(p.y * 10000.0f) / 10000.0f,
                                                             std::round(p.z * 10000.0f) / 10000.0f}));
                        }
                        local[corner] = found->second;
                    }
                    triangles.push_back(Json::array({local[0], local[1], local[2]}));
                }
                return result({{"scope", scope},
                               {"vertexCount", positions.size()},
                               {"triangleCount", triangles.size()},
                               {"truncated", truncated},
                               {"positions", positions},
                               // Triangles index `positions`; this says which editor vertex
                               // each entry is, for commands that take vertex indices
                               // (select, loop_cut, ...).
                               {"vertexIds", vertexIds},
                               {"triangles", triangles}});
            });
    }

    {
        Json properties = {
            {"width", integerSchema("Image width in pixels, 64-1920. Default 960.")},
            {"height", integerSchema("Image height in pixels, 64-1920. Default 540.")},
            {"view", choiceSchema("Camera. 'perspective' orbits using azimuth/elevation; the others "
                                  "are orthographic looks straight down an axis (front looks "
                                  "toward -Z, right looks toward -X, top looks down).",
                                  {"perspective", "front", "back", "left", "right", "top", "bottom"})},
            {"azimuth", numberSchema("Perspective only: degrees around the model. 0 = seen from +Z "
                                     "(front), 90 = from +X (right). Default 35.")},
            {"elevation", numberSchema("Perspective only: degrees above the horizon, -89 to 89. "
                                       "Default 25.")},
            {"frame", boolSchema("Fit the whole mesh in view (default true). When false, 'target' "
                                 "and 'distance' set the camera.")},
            {"target", vec3Schema("Point the camera looks at when frame is false.")},
            {"distance", numberSchema("Camera distance (perspective) or view height in world units "
                                      "(orthographic) when frame is false.")},
            {"shading", choiceSchema("'textured' shows part colours and is the default; 'solid' is "
                                     "neutral grey; 'wireframe' shows the triangles.",
                                     {"textured", "solid", "wireframe"})},
            {"wireframe_overlay", boolSchema("Draw the triangle edges over the shading.")},
            {"color_by_part", boolSchema("Give each part its own flat tint to tell them apart.")},
            {"grid", boolSchema("Draw the ground grid (default true).")}};
        add("screenshot",
            "Renders the model offscreen and returns a PNG image. The way to SEE what has been "
            "built - take one after every few edits, from more than one angle (try "
            "perspective, front, top and right).",
            objectSchema(properties), true,
            [editor](const CommandArgs& args)
            {
                requireMesh(*editor);

                BlenderApplication::CaptureParams params;
                params.width = static_cast<s32>(args.integer("width", 960, 64, kMaxCaptureSize));
                params.height = static_cast<s32>(args.integer("height", 540, 64, kMaxCaptureSize));
                const std::string view =
                    args.choice("view",
                                {"perspective", "front", "back", "left", "right", "top", "bottom"},
                                "perspective");
                params.view = viewFromName(view);

                const std::string shading =
                    args.choice("shading", {"textured", "solid", "wireframe"}, "textured");
                params.shading = shading == "wireframe" ? MiniRenderMode::Wireframe
                                 : shading == "solid"   ? MiniRenderMode::Solid
                                                        : MiniRenderMode::Textured;
                params.wireframeOverlay = args.boolean("wireframe_overlay", false);
                params.colorBySubmesh = args.boolean("color_by_part", false);
                params.grid = args.boolean("grid", true);
                params.frame = args.boolean("frame", true);

                // The orbit camera's yaw/pitch run opposite to the way a person
                // says "from the right, looking down": azimuth toward +X and
                // elevation above the horizon are the natural reading.
                const f32 azimuth = static_cast<f32>(args.number("azimuth", 35.0, -3600.0, 3600.0));
                const f32 elevation = static_cast<f32>(args.number("elevation", 25.0, -89.0, 89.0));
                params.camera.yaw = -glm::radians(azimuth);
                params.camera.pitch = -glm::radians(elevation);
                params.camera.target = vec3Arg(args, "target", glm::vec3(0.0f));
                params.camera.distance =
                    static_cast<f32>(args.number("distance", 6.0, 0.05, kMaxCoordinate));

                std::vector<u8> rgba;
                if (!editor->captureViewport(params, rgba))
                    failed("the screenshot could not be rendered");

                CommandResult out;
                out.data = {{"width", params.width},
                            {"height", params.height},
                            {"view", view},
                            {"shading", shading}};
                out.image = CommandImage{"image/png", base64(encodePng(rgba, params.width, params.height))};
                return out;
            });
    }

    // ----------------------------------------------------------- document

    add("new_document",
        "Discards the current mesh, skeleton, animations and undo history and starts empty. "
        "Cannot be undone.",
        objectSchema(Json::object()), false,
        [editor](const CommandArgs&)
        {
            editor->newDocument();
            return result(statusJson(*editor));
        });

    add("load_mesh", "Opens a mesh file (.rmesh, .obj, .fbx, .gltf, ...), replacing the document.",
        objectSchema({{"path", stringSchema("File path on the machine running the editor.")}},
                     {"path"}),
        false,
        [editor](const CommandArgs& args)
        {
            const std::string path = args.requireString("path");
            if (!editor->loadMesh(path))
                failed("could not load '" + path + "'");
            return result(statusJson(*editor));
        });

    add("append_mesh", "Merges another mesh file into the document as additional parts.",
        objectSchema({{"path", stringSchema("File path on the machine running the editor.")}},
                     {"path"}),
        false,
        [editor](const CommandArgs& args)
        {
            const std::string path = args.requireString("path");
            if (!editor->appendMesh(path))
                failed("could not append '" + path + "'");
            return result(statusJson(*editor));
        });

    add("save_mesh",
        "Saves the document in the engine's own .rmesh format (the extension is forced to "
        ".rmesh), with a .material file beside it.",
        objectSchema({{"path", stringSchema("Where to save, e.g. /home/me/helicopter.rmesh.")}},
                     {"path"}),
        false,
        [editor](const CommandArgs& args)
        {
            const std::string path = args.requireString("path");
            requireMesh(*editor);
            if (!editor->saveAs(path))
                failed("could not save '" + path + "'");
            return result({{"saved", FileSystem::withoutExtension(path) + ".rmesh"}});
        });

    add("export_obj", "Exports the document as a Wavefront .obj file.",
        objectSchema({{"path", stringSchema("Where to write the .obj file.")}}, {"path"}), false,
        [editor](const CommandArgs& args)
        {
            const std::string path = args.requireString("path");
            requireMesh(*editor);
            if (!editor->exportObj(path))
                failed("could not export '" + path + "'");
            return result({{"exported", path}});
        });

    add("export_gltf",
        "Exports the document as a binary glTF 2.0 file (.glb): one mesh with a primitive and a "
        "PBR material (colour, roughness, metalness) per part. Static geometry only. This is the "
        "format to hand a model to a game or another tool.",
        objectSchema({{"path", stringSchema("Where to write the file, e.g. /home/me/helicopter.glb.")}},
                     {"path"}),
        false,
        [editor](const CommandArgs& args)
        {
            const std::string path = args.requireString("path");
            requireMesh(*editor);
            std::string why;
            if (!editor->exportGltf(path, &why))
                failed("could not export '" + path + "': " + why);
            return result({{"exported", path}});
        });

    {
        Json properties = {{"steps", integerSchema("How many steps to go back (default 1).")}};
        add("undo", "Undoes the last edit(s).", objectSchema(properties), false,
            [editor](const CommandArgs& args)
            {
                const long long steps = args.integer("steps", 1, 1, 100);
                long long done = 0;
                for (; done < steps && editor->canUndo(); ++done)
                    editor->undo();
                if (done == 0)
                    failed("nothing to undo");
                Json data = statusJson(*editor);
                data["stepsUndone"] = done;
                return result(std::move(data));
            });
        add("redo", "Redoes edits that were undone.", objectSchema(properties), false,
            [editor](const CommandArgs& args)
            {
                const long long steps = args.integer("steps", 1, 1, 100);
                long long done = 0;
                for (; done < steps && editor->canRedo(); ++done)
                    editor->redo();
                if (done == 0)
                    failed("nothing to redo");
                Json data = statusJson(*editor);
                data["stepsRedone"] = done;
                return result(std::move(data));
            });
    }

    // -------------------------------------------------------- build parts

    {
        Json properties = {
            {"type", choiceSchema("Shape. Box and plane use 'size'; sphere, cylinder, cone, capsule "
                                  "and torus use 'radius' (and 'height' / 'minor_radius').",
                                  {"box", "plane", "sphere", "cylinder", "cone", "capsule", "torus"})},
            {"size", vec3Schema("Box size [x,y,z], or plane extent [x,_,z]. Default [1,1,1].")},
            {"radius", numberSchema("Sphere/cylinder/cone/capsule radius, torus major radius. "
                                    "Default 0.5.")},
            {"minor_radius", numberSchema("Torus tube radius. Default 0.2.")},
            {"height", numberSchema("Cylinder/cone/capsule height along Y. Default 1. A cylinder "
                                    "or cone is centred on its origin.")},
            {"slices", integerSchema("Segments around the axis, 3-256. Default 24.")},
            {"rings", integerSchema("Rings (sphere/capsule) or tube segments (torus), 2-256. "
                                    "Default 16.")},
            {"segments_x", integerSchema("Plane subdivisions along X, 1-256. Default 8.")},
            {"segments_z", integerSchema("Plane subdivisions along Z, 1-256. Default 8.")},
            {"replace", boolSchema("Start a new mesh instead of adding a part (default false).")}};
        addPlacementProperties(properties);
        addStyleProperties(properties);
        add("add_primitive",
            "Adds a basic solid as a new part, already placed and coloured. Every other shape is "
            "built from these: stretch a sphere with a non-uniform 'scale' for a fuselage, use "
            "boxes for blades and struts, cylinders for masts and skids. Returns the part's "
            "index, name and bounds.",
            objectSchema(properties, {"type"}), false,
            [editor](const CommandArgs& args)
            {
                BlenderApplication::PrimitiveParams params;
                const std::string type = args.requireChoice(
                    "type", {"box", "plane", "sphere", "cylinder", "cone", "capsule", "torus"});
                if (!BlenderApplication::primitiveTypeFromName(type, params.type))
                    invalid("unknown primitive '" + type + "'");

                const std::vector<double> size = args.numbers("size", 3, {1.0, 1.0, 1.0});
                requireFiniteRange("size", size, kMaxCoordinate);
                for (const double value : size)
                    if (value <= 0.0)
                        invalid("argument 'size' must be positive");
                params.size = toVec3(size);
                params.radius = static_cast<f32>(args.number("radius", 0.5, 0.0001, kMaxCoordinate));
                params.minorRadius =
                    static_cast<f32>(args.number("minor_radius", 0.2, 0.0001, kMaxCoordinate));
                params.height = static_cast<f32>(args.number("height", 1.0, 0.0001, kMaxCoordinate));
                params.slices = static_cast<s32>(args.integer("slices", 24, 3, 256));
                params.rings = static_cast<s32>(args.integer("rings", 16, 2, 256));
                params.segmentsX = static_cast<s32>(args.integer("segments_x", 8, 1, 256));
                params.segmentsZ = static_cast<s32>(args.integer("segments_z", 8, 1, 256));

                s32 submesh = -1;
                if (!editor->createPrimitive(params, placementArg(args), styleArg(args),
                                             args.boolean("replace", false), &submesh))
                    failed("could not build the " + type);
                return partAdded(*editor, submesh);
            });
    }

    {
        Json properties = {
            {"profile", {{"type", "array"},
                         {"minItems", 2},
                         {"items", {{"type", "array"},
                                    {"items", {{"type", "number"}}},
                                    {"minItems", 2},
                                    {"maxItems", 2}}},
                         {"description", "Outline as [radius, y] pairs from one end to the other, "
                                         "spun around the Y axis. A radius of 0 closes that end to "
                                         "a point. Example, a bullet nose: [[0,2],[0.4,1.8],[0.6,1],[0.6,0]]."}}},
            {"slices", integerSchema("Segments around the axis, 3-256. Default 24.")},
            {"cap_start", boolSchema("Close an open first end with a disc (default true).")},
            {"cap_end", boolSchema("Close an open last end with a disc (default true).")}};
        addPlacementProperties(properties);
        addStyleProperties(properties);
        add("add_lathe",
            "Adds a part made by spinning an outline around the Y axis: nose cones, tapered "
            "booms, domes, hubs, fuel tanks - anything round that a primitive cannot give. "
            "Rotate it afterwards with 'rotation' to lay it along X or Z.",
            objectSchema(properties, {"profile"}), false,
            [editor](const CommandArgs& args)
            {
                const Json* raw = args.raw("profile");
                if (!raw || !raw->is_array())
                    invalid("argument 'profile' is required and must be an array of [radius, y]");
                if (raw->size() > 256)
                    invalid("argument 'profile' may hold at most 256 points");

                LatheParams params;
                for (const Json& point : *raw)
                {
                    if (!point.is_array() || point.size() != 2 || !point[0].is_number() ||
                        !point[1].is_number())
                        invalid("every profile point must be [radius, y]");
                    const double radius = point[0].get<double>();
                    const double y = point[1].get<double>();
                    if (std::abs(radius) > kMaxCoordinate || std::abs(y) > kMaxCoordinate)
                        invalid("profile values are out of range");
                    params.profile.push_back(
                        glm::vec2(static_cast<f32>(radius), static_cast<f32>(y)));
                }
                params.slices = static_cast<u32>(args.integer("slices", 24, 3, 256));
                params.capStart = args.boolean("cap_start", true);
                params.capEnd = args.boolean("cap_end", true);

                MeshData part;
                std::string error;
                if (!buildLathe(params, part, &error))
                    invalid(error);

                s32 submesh = -1;
                if (!editor->appendPart(std::move(part), placementArg(args), styleArg(args), "Lathe",
                                        false, &submesh))
                    failed("could not add the lathe part");
                return partAdded(*editor, submesh);
            });
    }

    {
        Json section = objectSchema(
            {{"at", numberSchema("Position of this cross-section along the axis.")},
             {"width", numberSchema("Full width across the first cross axis (X for axis z/y, Y for "
                                    "axis x). 0 pinches to a point.")},
             {"height", numberSchema("Full height across the second cross axis (Y for axis z/x, Z for "
                                     "axis y). 0 pinches to a point.")},
             {"offset", {{"type", "array"},
                         {"items", {{"type", "number"}}},
                         {"minItems", 2},
                         {"maxItems", 2},
                         {"description", "Shifts the section's centre across [first, second] axes - "
                                         "e.g. to make a tail boom rise."}}},
             {"exponent", numberSchema("Shape: 2 = ellipse (default), 4-6 = rounded box, 0.5-1 = "
                                       "pointed diamond.")}},
            {"at", "width", "height"});
        Json properties = {
            {"axis", choiceSchema("Direction the shape runs along. Default 'z'.", {"x", "y", "z"})},
            {"sections", {{"type", "array"},
                          {"minItems", 2},
                          {"items", section},
                          {"description", "Cross-sections in increasing 'at' order. The surface is "
                                          "drawn smoothly through them: a fuselage is a handful of "
                                          "sections from nose to tail, each with its own width and "
                                          "height."}}},
            {"segments", integerSchema("Segments around each section, 3-256. Default 24.")},
            {"cap_start", boolSchema("Close an open first end with a disc (default true).")},
            {"cap_end", boolSchema("Close an open last end with a disc (default true).")}};
        addPlacementProperties(properties);
        addStyleProperties(properties);
        add("add_loft",
            "Adds a part by stringing elliptical (or boxier) cross-sections along an axis - the "
            "way to model a fuselage, a tapering tail boom, a wing or a float that is not a "
            "straight solid. Far fewer calls and a smoother result than stacking primitives.",
            objectSchema(properties, {"sections"}), false,
            [editor](const CommandArgs& args)
            {
                const Json* raw = args.raw("sections");
                if (!raw || !raw->is_array())
                    invalid("argument 'sections' is required and must be an array");
                if (raw->size() > 256)
                    invalid("argument 'sections' may hold at most 256 entries");

                LoftParams params;
                const std::string axis = args.choice("axis", {"x", "y", "z"}, "z");
                params.axis = axis == "x" ? 0 : axis == "y" ? 1 : 2;
                for (const Json& item : *raw)
                {
                    if (!item.is_object())
                        invalid("every section must be an object with at, width and height");
                    const CommandArgs fields(item);
                    LoftSection section;
                    section.at = static_cast<f32>(fields.requireNumber("at"));
                    section.width = static_cast<f32>(fields.requireNumber("width"));
                    section.height = static_cast<f32>(fields.requireNumber("height"));
                    const std::vector<double> offset = fields.numbers("offset", 2, {0.0, 0.0});
                    section.offset = glm::vec2(static_cast<f32>(offset[0]), static_cast<f32>(offset[1]));
                    section.exponent = static_cast<f32>(fields.number("exponent", 2.0));
                    if (std::abs(section.at) > kMaxCoordinate || section.width > kMaxCoordinate ||
                        section.height > kMaxCoordinate || std::abs(section.offset.x) > kMaxCoordinate ||
                        std::abs(section.offset.y) > kMaxCoordinate)
                        invalid("section values are out of range");
                    params.sections.push_back(section);
                }
                params.segments = static_cast<u32>(args.integer("segments", 24, 3, 256));
                params.capStart = args.boolean("cap_start", true);
                params.capEnd = args.boolean("cap_end", true);

                MeshData part;
                std::string error;
                if (!buildLoft(params, part, &error))
                    invalid(error);

                s32 submesh = -1;
                if (!editor->appendPart(std::move(part), placementArg(args), styleArg(args), "Loft",
                                        false, &submesh))
                    failed("could not add the loft part");
                return partAdded(*editor, submesh);
            });
    }

    {
        Json properties = {
            {"positions", {{"type", "array"},
                           {"items", {{"type", "array"},
                                      {"items", {{"type", "number"}}},
                                      {"minItems", 3},
                                      {"maxItems", 3}}},
                           {"description", "Vertex positions as [x,y,z] triples."}}},
            {"triangles", {{"type", "array"},
                           {"items", {{"type", "array"},
                                      {"items", {{"type", "integer"}}},
                                      {"minItems", 3},
                                      {"maxItems", 3}}},
                           {"description", "Triangles as [a,b,c] vertex indices, counter-clockwise "
                                           "seen from outside."}}},
            {"uvs", {{"type", "array"},
                     {"items", {{"type", "array"},
                                {"items", {{"type", "number"}}},
                                {"minItems", 2},
                                {"maxItems", 2}}},
                     {"description", "Optional [u,v] per vertex."}}},
            {"smooth", boolSchema("Smooth normals across shared vertices (default true); false gives "
                                  "flat faces.")}};
        addPlacementProperties(properties);
        addStyleProperties(properties);
        add("add_mesh",
            "Adds a part from raw vertices and triangles, for shapes none of the other add_* "
            "commands can describe. Normals are computed for you.",
            objectSchema(properties, {"positions", "triangles"}), false,
            [editor](const CommandArgs& args)
            {
                const Json* positions = args.raw("positions");
                const Json* triangles = args.raw("triangles");
                if (!positions || !positions->is_array())
                    invalid("argument 'positions' is required and must be an array of [x,y,z]");
                if (!triangles || !triangles->is_array())
                    invalid("argument 'triangles' is required and must be an array of [a,b,c]");
                if (positions->size() < 3 || positions->size() > kMaxCustomVertices)
                    invalid("'positions' must hold between 3 and " +
                            std::to_string(kMaxCustomVertices) + " vertices");
                if (triangles->empty() || triangles->size() * 3 > kMaxCustomIndices)
                    invalid("'triangles' must hold between 1 and " +
                            std::to_string(kMaxCustomIndices / 3) + " triangles");

                MeshData part;
                part.positions.reserve(positions->size());
                for (const Json& point : *positions)
                {
                    if (!point.is_array() || point.size() != 3)
                        invalid("every position must be [x,y,z]");
                    glm::vec3 p;
                    for (glm::length_t axis = 0; axis < 3; ++axis)
                    {
                        if (!point[static_cast<size_t>(axis)].is_number())
                            invalid("position values must be numbers");
                        const double value = point[static_cast<size_t>(axis)].get<double>();
                        if (!std::isfinite(value) || std::abs(value) > kMaxCoordinate)
                            invalid("position values must be finite and within +/-100000");
                        p[axis] = static_cast<f32>(value);
                    }
                    part.positions.push_back(p);
                }

                part.indices.reserve(triangles->size() * 3);
                for (const Json& triangle : *triangles)
                {
                    if (!triangle.is_array() || triangle.size() != 3)
                        invalid("every triangle must be [a,b,c]");
                    for (size_t corner = 0; corner < 3; ++corner)
                    {
                        if (!triangle[corner].is_number_integer())
                            invalid("triangle entries must be integers");
                        const long long index = triangle[corner].get<long long>();
                        if (index < 0 || index >= static_cast<long long>(part.positions.size()))
                            invalid("triangle index " + std::to_string(index) +
                                    " is outside the " + std::to_string(part.positions.size()) +
                                    " positions");
                        part.indices.push_back(static_cast<u32>(index));
                    }
                }

                if (const Json* uvs = args.raw("uvs"))
                {
                    if (!uvs->is_array() || uvs->size() != positions->size())
                        invalid("'uvs' must hold one [u,v] per position");
                    for (const Json& uv : *uvs)
                    {
                        if (!uv.is_array() || uv.size() != 2 || !uv[0].is_number() || !uv[1].is_number())
                            invalid("every uv must be [u,v]");
                        part.uvs.push_back(glm::vec2(uv[0].get<f32>(), uv[1].get<f32>()));
                    }
                }
                else
                {
                    part.uvs.assign(part.positions.size(), glm::vec2(0.0f));
                }

                Assets().recalculateNormals(part, args.boolean("smooth", true));

                s32 submesh = -1;
                if (!editor->appendPart(std::move(part), placementArg(args), styleArg(args), "Mesh",
                                        false, &submesh))
                    failed("could not add the mesh part");
                return partAdded(*editor, submesh);
            });
    }

    // ------------------------------------------------------- edit a part

    {
        Json properties = {{"part", partRefSchema()},
                           {"pivot", vec3Schema("Point the rotation and scale act about. Default: the "
                                                "centre of the part's bounds.")},
                           {"pivot_origin", boolSchema("Use the world origin as the pivot instead.")}};
        addPlacementProperties(properties);
        properties["position"] = vec3Schema("Translation added to the part, [x,y,z]. Default [0,0,0].");
        add("transform_part",
            "Moves, rotates and/or scales an existing part. 'position' is an offset added to where "
            "the part is now, not an absolute location; rotation and scale act about the pivot.",
            objectSchema(properties, {"part"}), false,
            [editor](const CommandArgs& args)
            {
                const u32 part = resolvePart(*editor, args);
                const MeshData& mesh = requireMesh(*editor);
                glm::vec3 pivot = mesh.submeshes[part].bounds.center();
                if (args.boolean("pivot_origin", false))
                    pivot = glm::vec3(0.0f);
                else if (args.has("pivot"))
                    pivot = vec3Arg(args, "pivot", pivot);

                const glm::vec3 offset = vec3Arg(args, "position", glm::vec3(0.0f));
                const glm::mat4 about = composeTransform(glm::vec3(0.0f), vec3Arg(args, "rotation", glm::vec3(0.0f)),
                                                         scaleArg(args, "scale"));
                const glm::mat4 matrix = glm::translate(glm::mat4(1.0f), offset) * about;
                if (!editor->transformSubmesh(part, matrix, pivot))
                    failed("the part has no vertices to move");
                return partAdded(*editor, static_cast<s32>(part));
            });
    }

    {
        Json properties = {
            {"part", partRefSchema()},
            {"mirror", choiceSchema("Mirror the copy across the plane through the origin "
                                    "perpendicular to this axis (x = left/right twin).",
                                    {"x", "y", "z"})}};
        addPlacementProperties(properties);
        addStyleProperties(properties);
        add("duplicate_part",
            "Copies a part. With 'mirror' the copy is the mirror image - the quick way to make the "
            "second skid, the other stub wing or the twin engine. 'position', 'rotation' and "
            "'scale' are applied to the copy after mirroring; 'name'/'color' restyle only the copy.",
            objectSchema(properties, {"part"}), false,
            [editor](const CommandArgs& args)
            {
                const u32 part = resolvePart(*editor, args);
                glm::mat4 matrix = placementArg(args);
                const std::string mirror = args.choice("mirror", {"x", "y", "z"}, "");
                if (!mirror.empty())
                {
                    glm::vec3 flip(1.0f);
                    flip[mirror == "x" ? 0 : mirror == "y" ? 1 : 2] = -1.0f;
                    matrix = matrix * glm::scale(glm::mat4(1.0f), flip);
                }

                s32 copy = -1;
                if (!editor->duplicateSubmesh(part, matrix, &copy))
                    failed("could not copy the part");

                const BlenderApplication::PartStyle style = styleArg(args);
                if ((!style.name.empty() || style.hasColor || style.hasRoughness || style.hasMetallic) &&
                    !editor->styleSubmesh(static_cast<u32>(copy), style))
                    failed("the copy was made but could not be restyled");
                return partAdded(*editor, copy);
            });
    }

    {
        Json properties = {{"part", partRefSchema()}};
        addStyleProperties(properties);
        add("style_part", "Changes a part's name, colour, roughness or metalness.",
            objectSchema(properties, {"part"}), false,
            [editor](const CommandArgs& args)
            {
                const u32 part = resolvePart(*editor, args);
                const BlenderApplication::PartStyle style = styleArg(args);
                if (style.name.empty() && !style.hasColor && !style.hasRoughness && !style.hasMetallic)
                    invalid("give at least one of name, color, roughness, metallic");
                if (!editor->styleSubmesh(part, style))
                    failed("could not restyle the part");
                return partAdded(*editor, static_cast<s32>(part));
            });
    }

    add("delete_part", "Removes a part and its triangles.",
        objectSchema({{"part", partRefSchema()}}, {"part"}), false,
        [editor](const CommandArgs& args)
        {
            const u32 part = resolvePart(*editor, args);
            const std::string name = partName(requireMesh(*editor), part);
            if (!editor->deleteSubmesh(part))
                failed("could not delete the part");
            return result({{"deleted", name}, {"remainingParts", editor->currentMeshData()->submeshes.size()}});
        });

    add("set_part_visible",
        "Hides or shows a part in the viewport and in screenshots (the geometry stays).",
        objectSchema({{"part", partRefSchema()}, {"visible", boolSchema("true to show, false to hide.")}},
                     {"part", "visible"}),
        false,
        [editor](const CommandArgs& args)
        {
            const u32 part = resolvePart(*editor, args);
            const bool visible = args.boolean("visible", true);
            if (!args.has("visible"))
                invalid("argument 'visible' is required");
            editor->setSubmeshVisible(part, visible);
            return result({{"part", part}, {"visible", visible}});
        });

    add("extract_part", "Keeps only one part, re-indexed to stand alone, and drops the rest.",
        objectSchema({{"part", partRefSchema()}}, {"part"}), false,
        [editor](const CommandArgs& args)
        {
            const u32 part = resolvePart(*editor, args);
            editor->setSelectedSubmesh(static_cast<s32>(part));
            if (!editor->extractSelectedSubmesh())
                failed("could not extract the part");
            return result(statusJson(*editor));
        });

    // -------------------------------------------------------- selection

    {
        Json box = objectSchema({{"min", vec3Schema("Lower corner.")}, {"max", vec3Schema("Upper corner.")}},
                                {"min", "max"});
        box["description"] = "Axis-aligned region in world space. Selects vertices inside it (vertex "
                             "mode), faces whose centre is inside it (face mode) or edges with both "
                             "ends inside it (edge mode).";
        Json properties = {
            {"mode", choiceSchema("Element type to select. Keeps the current mode when omitted.",
                                  {"vertex", "edge", "face"})},
            {"action", choiceSchema("set (default) replaces the selection; add / remove change it; "
                                    "clear, all, invert, grow, shrink and linked operate on what "
                                    "is selected.",
                                    {"set", "add", "remove", "clear", "all", "invert", "grow",
                                     "shrink", "linked"})},
            {"vertices", {{"type", "array"}, {"items", {{"type", "integer"}}}, {"description", "Vertex indices."}}},
            {"edges", {{"type", "array"},
                       {"items", {{"type", "array"},
                                  {"items", {{"type", "integer"}}},
                                  {"minItems", 2},
                                  {"maxItems", 2}}},
                       {"description", "Edges as [vertexA, vertexB] pairs; the two vertices must be "
                                       "joined by an edge (vertices standing at the same point count "
                                       "as one)."}}},
            {"faces", {{"type", "array"}, {"items", {{"type", "integer"}}}, {"description", "Face (triangle) indices."}}},
            {"part", partRefSchema()},
            {"box", box}};
        add("select",
            "Selects vertices, edges or faces so the edit commands (transform_selection, extrude, "
            "delete_selection, weld, smooth, ...) know what to act on. Pick by index, by part, or by "
            "a world-space box. Returns the resulting selection.",
            objectSchema(properties), false,
            [editor](const CommandArgs& args)
            {
                MeshData& mesh = requireMesh(*editor);
                BlenderSelection& selection = editor->selection();

                const std::string mode = args.choice("mode", {"vertex", "edge", "face"}, "");
                if (mode == "vertex")
                    selection.setMode(BlenderSelection::SelectionMode::Vertex);
                else if (mode == "edge")
                    selection.setMode(BlenderSelection::SelectionMode::Edge);
                else if (mode == "face")
                    selection.setMode(BlenderSelection::SelectionMode::Face);

                const std::string action = args.choice(
                    "action",
                    {"set", "add", "remove", "clear", "all", "invert", "grow", "shrink", "linked"},
                    "set");

                if (action == "clear")
                    selection.clearAll();
                else if (action == "all")
                    editor->selectAllElements();
                else if (action == "invert")
                    editor->invertElementSelection();
                else if (action == "grow")
                    editor->growSelection();
                else if (action == "shrink")
                    editor->shrinkSelection();
                else if (action == "linked")
                    editor->selectLinked();
                else
                {
                    const auto elementMode = selection.mode();
                    const bool faceMode = elementMode == BlenderSelection::SelectionMode::Face;
                    const bool edgeMode = elementMode == BlenderSelection::SelectionMode::Edge;
                    const char* modeName = faceMode ? "face" : edgeMode ? "edge" : "vertex";
                    const char* key = faceMode ? "faces" : edgeMode ? "edges" : "vertices";
                    const u32 vertexCount = static_cast<u32>(mesh.positions.size());
                    const u32 faceCount = static_cast<u32>(mesh.indices.size() / 3);
                    const MeshTopology& topology = editor->topology();

                    // What was asked for, as indices (vertex/face) or edge keys.
                    std::vector<u32> picked;
                    std::vector<u64> pickedEdges;

                    if (args.has("vertices") || args.has("faces") || args.has("edges"))
                    {
                        if (!args.has(key))
                            invalid(std::string("the selection mode is ") + modeName + ", so give '" +
                                    key + "'");
                        if (edgeMode)
                        {
                            const Json* pairs = args.raw("edges");
                            if (!pairs->is_array() || pairs->size() > kMaxSelectionIndices)
                                invalid("argument 'edges' must be an array of [a, b] pairs");
                            for (const Json& pair : *pairs)
                            {
                                if (!pair.is_array() || pair.size() != 2 || !pair[0].is_number_integer() ||
                                    !pair[1].is_number_integer())
                                    invalid("every edge must be [vertexA, vertexB]");
                                const long long a = pair[0].get<long long>();
                                const long long b = pair[1].get<long long>();
                                if (a < 0 || b < 0 || a >= vertexCount || b >= vertexCount)
                                    invalid("edge vertex index out of range (0.." +
                                            std::to_string(vertexCount - 1) + ")");
                                const u32 ca = topology.canonical(static_cast<u32>(a));
                                const u32 cb = topology.canonical(static_cast<u32>(b));
                                if (topology.findEdge(ca, cb) < 0)
                                    invalid("vertices " + std::to_string(a) + " and " + std::to_string(b) +
                                            " are not joined by an edge");
                                pickedEdges.push_back(MeshTopology::edgeKey(ca, cb));
                            }
                        }
                        else
                        {
                            picked = args.indices(key, kMaxSelectionIndices);
                            const u32 limit = faceMode ? faceCount : vertexCount;
                            for (const u32 index : picked)
                                if (index >= limit)
                                    invalid(std::string("index ") + std::to_string(index) + " is out of " +
                                            "range (0.." + std::to_string(limit - 1) + ")");
                        }
                    }
                    else if (args.has("part"))
                    {
                        const u32 part = resolvePart(*editor, args);
                        std::vector<u32> faces;
                        Assets().submeshFaces(mesh, part, faces);
                        if (faceMode)
                        {
                            picked = faces;
                        }
                        else if (edgeMode)
                        {
                            // Edges of the part's own triangles.
                            for (const u32 face : faces)
                                for (const s32 edge : topology.faceEdges(face))
                                    if (edge >= 0)
                                        pickedEdges.push_back(MeshTopology::edgeKey(
                                            topology.edges()[static_cast<usize>(edge)].a,
                                            topology.edges()[static_cast<usize>(edge)].b));
                        }
                        else
                        {
                            picked = editor->submeshVertices(part);
                        }
                    }
                    else if (args.has("box"))
                    {
                        const Json* boxJson = args.raw("box");
                        if (!boxJson || !boxJson->is_object())
                            invalid("argument 'box' must be an object with min and max");
                        const CommandArgs boxArgs(*boxJson);
                        if (!boxArgs.has("min") || !boxArgs.has("max"))
                            invalid("'box' needs both min and max");
                        const glm::vec3 low = vec3Arg(boxArgs, "min", glm::vec3(0.0f));
                        const glm::vec3 high = vec3Arg(boxArgs, "max", glm::vec3(0.0f));
                        const glm::vec3 lo = glm::min(low, high);
                        const glm::vec3 hi = glm::max(low, high);
                        auto inside = [&](const glm::vec3& p)
                        { return glm::all(glm::greaterThanEqual(p, lo)) && glm::all(glm::lessThanEqual(p, hi)); };

                        if (faceMode)
                        {
                            for (u32 face = 0; face < faceCount; ++face)
                            {
                                const glm::vec3 centre = (mesh.positions[mesh.indices[face * 3]] +
                                                          mesh.positions[mesh.indices[face * 3 + 1]] +
                                                          mesh.positions[mesh.indices[face * 3 + 2]]) /
                                                         3.0f;
                                if (inside(centre))
                                    picked.push_back(face);
                            }
                        }
                        else if (edgeMode)
                        {
                            for (const MeshTopology::Edge& edge : topology.edges())
                                if (inside(mesh.positions[edge.a]) && inside(mesh.positions[edge.b]))
                                    pickedEdges.push_back(MeshTopology::edgeKey(edge.a, edge.b));
                        }
                        else
                        {
                            for (u32 vertex = 0; vertex < vertexCount; ++vertex)
                                if (inside(mesh.positions[vertex]))
                                    picked.push_back(vertex);
                        }
                    }
                    else
                    {
                        invalid("give one of vertices, edges, faces, part or box (or a different action)");
                    }

                    if (action == "set")
                        selection.clearAll();
                    for (const u32 index : picked)
                    {
                        if (action == "remove")
                            faceMode ? selection.deselectFace(index) : selection.deselectVertex(index);
                        else
                            faceMode ? selection.selectFace(index) : selection.selectVertex(index);
                    }
                    for (const u64 edgeKey : pickedEdges)
                    {
                        if (action == "remove")
                            selection.deselectEdge(edgeKey);
                        else
                            selection.selectEdge(edgeKey);
                    }
                    editor->dropHiddenFromSelection();
                }
                return result(selectionJson(*editor));
            });
    }

    add("hide",
        "Hides the selection (or everything that is not selected) in the viewport and screenshots. "
        "Hidden triangles cannot be selected and are left alone by edits, which makes it easy to "
        "work on one area of a busy model. An edit that adds or removes triangles shows everything "
        "again. Vertices and edges hide every triangle that uses them.",
        objectSchema({{"what", choiceSchema("'selected' (default) or 'unselected'.", {"selected", "unselected"})}}),
        false,
        [editor](const CommandArgs& args)
        {
            requireMesh(*editor);
            const bool selected = args.choice("what", {"selected", "unselected"}, "selected") == "selected";
            const bool changed = selected ? editor->hideSelected() : editor->hideUnselected();
            if (!changed)
                failed(selected ? "nothing is selected to hide" : "nothing to hide - everything is selected or already hidden");
            return result({{"hiddenTriangles", editor->hiddenFaceCount()},
                           {"triangles", editor->currentMeshData()->indices.size() / 3}});
        });

    add("unhide", "Shows every hidden triangle again.", objectSchema(Json::object()), false,
        [editor](const CommandArgs&)
        {
            requireMesh(*editor);
            const usize was = editor->hiddenFaceCount();
            editor->unhideAll();
            return result({{"revealedTriangles", was}});
        });

    add("get_selection",
        "What is selected now: mode, counts, the bounds of the selected vertices and the first "
        "few indices.",
        objectSchema(Json::object()), true,
        [editor](const CommandArgs&) { return result(selectionJson(*editor)); });

    // ------------------------------------------------- edit the geometry

    {
        Json properties = {
            {"pivot", vec3Schema("Point rotation and scale act about. Default: the median of the "
                                 "selection.")}};
        addPlacementProperties(properties);
        properties["position"] = vec3Schema("Translation, [x,y,z]. Default [0,0,0].");
        add("transform_selection",
            "Moves, rotates and/or scales the selected vertices (or the corners of the selected "
            "faces). Fails when nothing is selected, rather than touching the whole mesh.",
            objectSchema(properties), false,
            [editor](const CommandArgs& args)
            {
                MeshData& mesh = requireMesh(*editor);
                const std::vector<u32> vertices = selectedVertexSet(*editor);
                if (vertices.empty())
                    failed("nothing is selected - use 'select' first");

                glm::vec3 pivot(0.0f);
                if (args.has("pivot"))
                {
                    pivot = vec3Arg(args, "pivot", glm::vec3(0.0f));
                }
                else
                {
                    glm::dvec3 sum(0.0);
                    for (const u32 index : vertices)
                        sum += glm::dvec3(mesh.positions[index]);
                    pivot = glm::vec3(sum / static_cast<double>(vertices.size()));
                }

                const glm::vec3 offset = vec3Arg(args, "position", glm::vec3(0.0f));
                const glm::mat4 about = composeTransform(glm::vec3(0.0f), vec3Arg(args, "rotation", glm::vec3(0.0f)),
                                                         scaleArg(args, "scale"));
                editor->recordUndo();
                Assets().transformVerticesAbout(mesh, glm::translate(glm::mat4(1.0f), offset) * about,
                                                pivot, vertices);
                editor->applyMeshEdit();
                return result({{"vertices", vertices.size()}, {"bounds", boundsJson(mesh.bounds)}});
            });
    }

    {
        Json properties = {
            {"pivot", vec3Schema("Point rotation and scale act about. Default the world origin.")}};
        addPlacementProperties(properties);
        properties["position"] = vec3Schema("Translation, [x,y,z]. Default [0,0,0].");
        add("transform_mesh", "Moves, rotates and/or scales the entire mesh (every part together).",
            objectSchema(properties), false,
            [editor](const CommandArgs& args)
            {
                MeshData& mesh = requireMesh(*editor);
                const glm::vec3 pivot = vec3Arg(args, "pivot", glm::vec3(0.0f));
                const glm::vec3 offset = vec3Arg(args, "position", glm::vec3(0.0f));
                const glm::mat4 about = composeTransform(glm::vec3(0.0f), vec3Arg(args, "rotation", glm::vec3(0.0f)),
                                                         scaleArg(args, "scale"));
                const glm::mat4 matrix = glm::translate(glm::mat4(1.0f), offset) * about;
                editor->recordUndo();
                Assets().transformVerticesAbout(mesh, matrix, pivot);
                if (glm::determinant(glm::mat3(matrix)) < 0.0f)
                    Assets().flipWinding(mesh);
                Assets().computeSubMeshBounds(mesh);
                editor->applyMeshEdit();
                return result({{"bounds", boundsJson(mesh.bounds)}});
            });
    }

    add("subdivide",
        "Subdivides the selected faces (every face when nothing is selected) 'levels' times. Flat "
        "splits each triangle into four without moving anything; 'smooth' uses Loop subdivision and "
        "rounds the surface - the way to turn a rough low-poly shape into a smooth one. Triangles "
        "next to the selection are cut just enough to stay watertight. Careful: triangle count "
        "grows 4x per level.",
        objectSchema({{"levels", integerSchema("1-4 (default 1).")},
                      {"smooth", boolSchema("Loop smoothing (default false = flat).")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const u32 levels = static_cast<u32>(args.integer("levels", 1, 1, 4));
            const usize before = mesh.indices.size() / 3;
            std::string why;
            if (!editor->subdivideSelection(levels, args.boolean("smooth", false), &why))
                failed(why.empty() ? "nothing was subdivided" : why);
            return result({{"trianglesBefore", before}, {"triangles", mesh.indices.size() / 3},
                           {"bounds", boundsJson(mesh.bounds)}});
        });

    add("turn_edge",
        "Flips the diagonal shared by two triangles (selected edges, mode 'edge'). Refused for an "
        "edge on a border, a seam, or where the two triangles form a concave quad.",
        objectSchema(Json::object()), false,
        [editor](const CommandArgs&)
        {
            MeshData& mesh = requireMesh(*editor);
            std::string why;
            const u32 turned = editor->turnSelectedEdges(&why);
            if (turned == 0)
                failed(why.empty() ? "no edge could be turned" : why);
            return result({{"turned", turned}, {"triangles", mesh.indices.size() / 3}});
        });

    add("split_edge",
        "Adds a vertex on each selected edge at 't' (0-1 from its lower-numbered end) and splits the "
        "triangles on it. The new vertices become the selection (vertex mode), ready to move or "
        "extrude.",
        objectSchema({{"t", numberSchema("Position along the edge, strictly between 0 and 1. Default 0.5.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 t = static_cast<f32>(args.number("t", 0.5, 0.001, 0.999));
            std::string why;
            const u32 split = editor->splitSelectedEdges(t, &why);
            if (split == 0)
                failed(why.empty() ? "nothing was split" : why);
            return result({{"split", split}, {"triangles", mesh.indices.size() / 3},
                           {"selection", selectionJson(*editor)}});
        });

    add("collapse_edge",
        "Merges the two ends of each selected edge at 't' along it and removes the triangles that "
        "collapse. Simplifies geometry by hand.",
        objectSchema({{"t", numberSchema("Where the ends meet, 0-1 from the lower-numbered end. Default 0.5.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 t = static_cast<f32>(args.number("t", 0.5, 0.0, 1.0));
            std::string why;
            const u32 collapsed = editor->collapseSelectedEdges(t, &why);
            if (collapsed == 0)
                failed(why.empty() ? "nothing was collapsed" : why);
            return result({{"collapsed", collapsed}, {"triangles", mesh.indices.size() / 3}});
        });

    add("knife",
        "Cuts the mesh along a plane, keeping all the geometry (unlike 'bisect'): every triangle the "
        "plane crosses is split along it, leaving a line of new edges that becomes the selection "
        "(edge mode). Follow with extrude, inset, bevel or a transform on that line. Give 'axis' "
        "and 'offset' for a plane perpendicular to x, y or z, or a free 'normal'.",
        objectSchema({{"axis", choiceSchema("Plane perpendicular to this axis.", {"x", "y", "z"})},
                      {"normal", vec3Schema("Free plane normal, instead of 'axis'.")},
                      {"offset", numberSchema("Plane position: the plane is dot(normal, p) = offset. Default 0.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            glm::vec3 normal(0.0f);
            if (args.has("normal"))
            {
                normal = vec3Arg(args, "normal", glm::vec3(0.0f));
            }
            else
            {
                const std::string axis = args.requireChoice("axis", {"x", "y", "z"});
                normal[axis == "x" ? 0 : axis == "y" ? 1 : 2] = 1.0f;
            }
            const f32 offset = static_cast<f32>(args.number("offset", 0.0, -kMaxCoordinate, kMaxCoordinate));
            std::string why;
            if (!editor->knifeCut(normal, offset, &why))
                failed(why.empty() ? "the knife did not cut anything" : why);
            return result({{"triangles", mesh.indices.size() / 3}, {"selection", selectionJson(*editor)}});
        });

    add("loop_cut",
        "Adds edge loops around a ring of quads: select ONE edge (mode 'edge') that runs across the "
        "strip to be cut - e.g. a vertical edge of a cylinder to cut a horizontal ring round it - and "
        "call this. The ring is followed through pairs of triangles that form quads until it closes "
        "or ends. The new loops become the selection.",
        objectSchema({{"cuts", integerSchema("Number of loops, 1-32 (default 1), evenly spaced.")},
                      {"edge", {{"type", "array"}, {"items", {{"type", "integer"}}}, {"minItems", 2}, {"maxItems", 2},
                                {"description", "Optional [vertexA, vertexB] of the edge; default the selected edge."}}}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const u32 cuts = static_cast<u32>(args.integer("cuts", 1, 1, 32));
            if (args.has("edge"))
            {
                const std::vector<unsigned> pair = args.indices("edge", 2);
                if (pair.size() != 2 || pair[0] >= mesh.positions.size() || pair[1] >= mesh.positions.size())
                    invalid("argument 'edge' must be two valid vertex indices");
                const MeshTopology& topology = editor->topology();
                const u32 a = topology.canonical(pair[0]);
                const u32 b = topology.canonical(pair[1]);
                if (topology.findEdge(a, b) < 0)
                    invalid("those two vertices are not joined by an edge");
                editor->selection().clearAll();
                editor->selection().setMode(BlenderSelection::SelectionMode::Edge);
                editor->selection().selectEdge(MeshTopology::edgeKey(a, b));
            }
            const usize before = mesh.indices.size() / 3;
            std::string why;
            if (!editor->loopCutSelected(cuts, &why))
                failed(why.empty() ? "no loop was cut" : why);
            return result({{"trianglesBefore", before}, {"triangles", mesh.indices.size() / 3},
                           {"selection", selectionJson(*editor)}});
        });

    add("inset",
        "Insets the selected faces as one region: the region shrinks away from its border by "
        "'thickness' (measured across the surface), leaving a ring of triangles; 'depth' then raises "
        "(+) or sinks (-) the inner region along its normal. Panels, windows, buttons. The shrunken "
        "region becomes the selection, so it can be inset or extruded again.",
        objectSchema({{"thickness", numberSchema("How far the border moves in, in world units.")},
                      {"depth", numberSchema("Move of the inner region along its normal. Default 0.")}},
                     {"thickness"}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 thickness = static_cast<f32>(args.requireNumber("thickness"));
            const f32 depth = static_cast<f32>(args.number("depth", 0.0, -kMaxCoordinate, kMaxCoordinate));
            if (thickness < 0.0f || thickness > kMaxCoordinate)
                invalid("argument 'thickness' must be zero or more");
            std::string why;
            if (!editor->insetSelection(thickness, depth, &why))
                failed(why.empty() ? "nothing was inset" : why);
            return result({{"triangles", mesh.indices.size() / 3}, {"selection", selectionJson(*editor)}});
        });

    add("bevel",
        "Chamfers the selected edges (mode 'edge'): each becomes a flat strip 'width' wide on both "
        "sides. The edges must have triangles on both sides and must not share a vertex - bevel "
        "edges that meet one after another. Refused when the width is larger than the faces next to "
        "an edge allow.",
        objectSchema({{"width", numberSchema("Distance from the old edge to each side of the new strip.")}},
                     {"width"}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 width = static_cast<f32>(args.requireNumber("width"));
            if (!(width > 0.0f) || width > kMaxCoordinate)
                invalid("argument 'width' must be greater than zero");
            std::string why;
            if (!editor->bevelSelectedEdges(width, &why))
                failed(why.empty() ? "nothing was bevelled" : why);
            return result({{"triangles", mesh.indices.size() / 3}, {"vertices", mesh.positions.size()}});
        });

    add("snap_to_grid",
        "Rounds the positions of the selected vertices (or of the whole mesh when nothing is "
        "selected) to multiples of 'step'. Cleans up hand-placed coordinates and makes parts line up.",
        objectSchema({{"step", numberSchema("Grid spacing in world units, e.g. 0.05.")}}, {"step"}), false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 step = static_cast<f32>(args.requireNumber("step"));
            if (!(step > 0.0f) || step > kMaxCoordinate)
                invalid("argument 'step' must be greater than 0");
            const u32 moved = editor->snapSelectionToGrid(step);
            return result({{"moved", moved}, {"bounds", boundsJson(mesh.bounds)}});
        });

    add("snap_to_vertex",
        "Moves each selected vertex onto the nearest vertex that is NOT selected, if one lies within "
        "'tolerance'. Closes small gaps between parts exactly; run weld_vertices afterwards to join "
        "them into one surface. Needs a selection.",
        objectSchema({{"tolerance", numberSchema("Largest distance to travel, in world units.")}}, {"tolerance"}),
        false,
        [editor](const CommandArgs& args)
        {
            requireMesh(*editor);
            const f32 tolerance = static_cast<f32>(args.requireNumber("tolerance"));
            if (!(tolerance > 0.0f) || tolerance > kMaxCoordinate)
                invalid("argument 'tolerance' must be greater than 0");
            if (editor->editVertices().empty())
                failed("nothing is selected - use 'select' first");
            const u32 moved = editor->snapSelectionToVertices(tolerance);
            if (moved == 0)
                failed("no selected vertex has an unselected vertex within the tolerance");
            return result({{"moved", moved}});
        });

    add("extrude",
        "Extrudes the selected faces along their normals by 'distance' and selects the new "
        "faces, so a second extrude continues from them. Needs a face selection.",
        objectSchema({{"distance", numberSchema("Distance along the face normals; negative sinks them.")}},
                     {"distance"}),
        false,
        [editor](const CommandArgs& args)
        {
            requireMesh(*editor);
            const f32 distance = static_cast<f32>(args.requireNumber("distance"));
            if (std::abs(distance) > kMaxCoordinate)
                invalid("argument 'distance' is out of range");
            if (editor->selection().selectedFaceCount() == 0)
                failed("no faces are selected - use 'select' with mode 'face' first");
            if (!editor->extrudeFaces(distance))
                failed("the extrusion produced nothing");
            return result({{"selection", selectionJson(*editor)},
                           {"triangles", editor->currentMeshData()->indices.size() / 3}});
        });

    add("delete_selection", "Deletes the selected vertices or faces (per the selection mode).",
        objectSchema(Json::object()), false,
        [editor](const CommandArgs&)
        {
            MeshData& mesh = requireMesh(*editor);
            BlenderSelection& selection = editor->selection();
            const auto mode = selection.mode();
            const u32 selected = mode == BlenderSelection::SelectionMode::Face   ? selection.selectedFaceCount()
                                 : mode == BlenderSelection::SelectionMode::Edge ? selection.selectedEdgeCount()
                                                                                 : selection.selectedVertexCount();
            if (selected == 0)
                failed("nothing is selected - use 'select' first");
            editor->deleteSelected();
            return result({{"vertices", mesh.positions.size()}, {"triangles", mesh.indices.size() / 3}});
        });

    add("weld_vertices",
        "Merges vertices closer than 'distance', removing cracks and duplicated seams.",
        objectSchema({{"distance", numberSchema("Merge radius. Default 0.0001.")},
                      {"selected_only", boolSchema("Only weld the selected vertices (default false).")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 distance = static_cast<f32>(args.number("distance", 0.0001, 0.0, 100.0));
            std::vector<u32> vertices;
            if (args.boolean("selected_only", false))
            {
                vertices = selectedVertexSet(*editor);
                if (vertices.empty())
                    failed("selected_only is set but nothing is selected");
            }
            const usize before = mesh.positions.size();
            editor->recordUndo();
            const u32 removed = Assets().weldVertices(mesh, distance, vertices);
            editor->selection().clearAll();
            editor->applyMeshEdit();
            return result({{"removed", removed}, {"verticesBefore", before}, {"vertices", mesh.positions.size()}});
        });

    add("smooth_vertices", "Relaxes vertex positions toward their neighbours (Laplacian smoothing).",
        objectSchema({{"strength", numberSchema("0-1 per pass. Default 0.5.")},
                      {"iterations", integerSchema("Passes, 1-50. Default 1.")},
                      {"selected_only", boolSchema("Only smooth the selection (default false).")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 strength = static_cast<f32>(args.number("strength", 0.5, 0.0, 1.0));
            const u32 iterations = static_cast<u32>(args.integer("iterations", 1, 1, 50));
            std::vector<u32> vertices;
            if (args.boolean("selected_only", false))
            {
                vertices = selectedVertexSet(*editor);
                if (vertices.empty())
                    failed("selected_only is set but nothing is selected");
            }
            editor->recordUndo();
            Assets().smoothVertices(mesh, strength, iterations, vertices);
            editor->applyMeshEdit();
            return result({{"bounds", boundsJson(mesh.bounds)}});
        });

    add("recalculate_normals", "Rebuilds the vertex normals from the triangles.",
        objectSchema({{"smooth", boolSchema("Average across shared vertices (default true); false = flat faces.")},
                      {"angle_weighted", boolSchema("Weight by corner angle (default false).")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            editor->recordUndo();
            Assets().recalculateNormals(mesh, args.boolean("smooth", true), args.boolean("angle_weighted", false));
            editor->applyMeshEdit();
            return result({{"vertices", mesh.positions.size()}});
        });

    add("flip_winding",
        "Turns triangles inside out - the fix for a part that renders dark or disappears from "
        "outside. Whole mesh unless 'part' is given.",
        objectSchema({{"part", partRefSchema()}}), false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const bool onePart = args.has("part");
            const u32 part = onePart ? resolvePart(*editor, args) : 0;
            editor->recordUndo();
            if (onePart)
                Assets().flipWinding(mesh, part);
            else
                Assets().flipWinding(mesh);
            Assets().recalculateNormals(mesh, true);
            editor->applyMeshEdit();
            return result({{"flipped", onePart ? partName(mesh, part) : std::string("all")}});
        });

    add("center_mesh",
        "Re-centres the whole mesh on the origin; 'ground' also drops the lowest point to y = 0 "
        "(what a model that stands on the floor wants).",
        objectSchema({{"ground", boolSchema("Centre on X/Z and rest on y = 0 (default false).")}}), false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            editor->recordUndo();
            if (args.boolean("ground", false))
                Assets().centerOnGround(mesh);
            else
                Assets().center(mesh);
            Assets().computeSubMeshBounds(mesh);
            editor->applyMeshEdit();
            return result({{"bounds", boundsJson(mesh.bounds)}});
        });

    add("bisect", "Cuts the mesh with an axis-aligned plane and keeps one side.",
        objectSchema({{"axis", choiceSchema("Plane normal axis.", {"x", "y", "z"})},
                      {"offset", numberSchema("Plane position along that axis. Default 0.")},
                      {"keep", choiceSchema("Which side survives: 'positive' (default) or 'negative'.",
                                            {"positive", "negative"})}},
                     {"axis"}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const std::string axis = args.requireChoice("axis", {"x", "y", "z"});
            const f32 offset = static_cast<f32>(args.number("offset", 0.0, -kMaxCoordinate, kMaxCoordinate));
            const bool keepPositive = args.choice("keep", {"positive", "negative"}, "positive") == "positive";
            if (!editor->bisectMesh(axis == "x" ? 0 : axis == "y" ? 1 : 2, offset, keepPositive))
                failed("the plane misses the mesh, or everything is on the discarded side");
            return result({{"triangles", mesh.indices.size() / 3}, {"bounds", boundsJson(mesh.bounds)}});
        });

    add("convex_hull", "Replaces the mesh with its convex hull (a collision proxy).",
        objectSchema(Json::object()), false,
        [editor](const CommandArgs&)
        {
            MeshData& mesh = requireMesh(*editor);
            if (!editor->makeConvexHull())
                failed("the convex hull could not be built (need at least 4 points)");
            return result({{"triangles", mesh.indices.size() / 3}});
        });

    add("generate_uv", "Generates texture coordinates for the whole mesh by projection.",
        objectSchema({{"mode", choiceSchema("Projection. Default planar.", {"planar", "cylindrical", "spherical"})},
                      {"u_tiles", numberSchema("Planar: texture repeat per unit; else repeats around. Default 1.")},
                      {"v_tiles", numberSchema("Cylindrical/spherical repeats along the height. Default 1.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const std::string mode = args.choice("mode", {"planar", "cylindrical", "spherical"}, "planar");
            const f32 u = static_cast<f32>(args.number("u_tiles", 1.0, 0.001, 1000.0));
            const f32 v = static_cast<f32>(args.number("v_tiles", 1.0, 0.001, 1000.0));
            editor->recordUndo();
            if (mode == "planar")
                Assets().makePlanarUV(mesh, u);
            else if (mode == "cylindrical")
                Assets().makeCylindricalUV(mesh, u, v);
            else
                Assets().makeSphericalUV(mesh, u, v);
            editor->applyMeshEdit();
            return result({{"mode", mode}});
        });

    add("unwrap_uv", "Unwraps non-overlapping UV islands (xatlas). Splits vertices at seams.",
        objectSchema({{"resolution", integerSchema("Atlas size in texels; 0 (default) = one page, automatic.")},
                      {"padding", integerSchema("Texels between islands. Default 4.")},
                      {"target", choiceSchema("Write to the texture UVs (default) or to the second "
                                              "set used for lightmaps.", {"uv", "uv2"})}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            BlenderApplication::UnwrapParams params;
            params.resolution = static_cast<u32>(args.integer("resolution", 0, 0, 16384));
            params.padding = static_cast<u32>(args.integer("padding", 4, 0, 256));
            params.target = args.choice("target", {"uv", "uv2"}, "uv") == "uv2" ? 1 : 0;
            if (!editor->unwrapUVs(params))
                failed("the unwrap failed");
            return result({{"vertices", mesh.positions.size()}});
        });

    add("simplify", "Reduces the triangle count while keeping the shape.",
        objectSchema({{"ratio", numberSchema("Fraction of triangles to keep, 0.05-1. Default 0.5.")},
                      {"error", numberSchema("Largest allowed deviation as a fraction of the mesh. Default 0.01.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 ratio = static_cast<f32>(args.number("ratio", 0.5, 0.05, 1.0));
            const f32 error = static_cast<f32>(args.number("error", 0.01, 0.0001, 0.5));
            const usize before = mesh.indices.size() / 3;
            editor->recordUndo();
            f32 reached = 0.0f;
            if (!Assets().simplifyMesh(mesh, ratio, error, &reached))
            {
                editor->discardUndo();
                failed("the mesh has no editable geometry");
            }
            Assets().optimizeVertexFetch(mesh);
            editor->applyMeshEdit();
            return result({{"trianglesBefore", before}, {"triangles", mesh.indices.size() / 3}, {"error", reached}});
        });

    add("optimize", "Welds duplicate vertices and reorders for GPU cache and overdraw efficiency.",
        objectSchema({{"overdraw_threshold", numberSchema("1-3. Default 1.05.")}}), false,
        [editor](const CommandArgs& args)
        {
            MeshData& mesh = requireMesh(*editor);
            const f32 threshold = static_cast<f32>(args.number("overdraw_threshold", 1.05, 1.0, 3.0));
            const usize before = mesh.positions.size();
            editor->recordUndo();
            const u32 welded = Assets().weldVertices(mesh);
            Assets().optimizeVertexCache(mesh);
            Assets().optimizeOverdraw(mesh, threshold);
            Assets().optimizeVertexFetch(mesh);
            editor->applyMeshEdit();
            return result({{"verticesBefore", before}, {"vertices", mesh.positions.size()}, {"welded", welded}});
        });

    // --------------------------------------------------------- animation

    add("set_animation",
        "Chooses the animation clip and frame shown, and starts or stops playback. Only "
        "meaningful for a rigged mesh with clips (see get_status).",
        objectSchema({{"clip", integerSchema("Index of the clip to activate.")},
                      {"frame", integerSchema("Frame to show (24 per second).")},
                      {"playing", boolSchema("Start (true) or stop (false) playback.")}}),
        false,
        [editor](const CommandArgs& args)
        {
            if (!editor->hasSkeleton())
                failed("the mesh has no skeleton");
            if (args.has("clip"))
            {
                const long long clip = args.requireInteger("clip");
                if (clip < 0 || static_cast<usize>(clip) >= editor->animationClipCount())
                    invalid("argument 'clip' is out of range");
                editor->setActiveAnimationClip(static_cast<s32>(clip));
            }
            if (args.has("frame"))
            {
                const long long frame = args.requireInteger("frame");
                if (frame < 0 || frame >= static_cast<long long>(editor->totalFrames()))
                    invalid("argument 'frame' is out of range (0.." +
                            std::to_string(editor->totalFrames() - 1) + ")");
                editor->setCurrentFrame(static_cast<u32>(frame));
            }
            if (args.has("playing"))
            {
                if (args.boolean("playing", false))
                    editor->play();
                else
                    editor->stop();
            }
            return result(statusJson(*editor)["animation"]);
        });
}

} // namespace Radion::BlenderApi

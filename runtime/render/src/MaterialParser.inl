#include "FileSystem.h"
#include "MaterialParserInternal.h"

#include <cstdlib>
#include <cmath>

namespace Radion
{

namespace
{

enum class TokenKind : u8
{
    End,
    Word,
    String,
    Number,
    LeftBrace,
    RightBrace,
    LeftBracket,
    RightBracket,
    LeftParen,
    RightParen,
    Comma
};

struct Token
{
    TokenKind kind = TokenKind::End;
    std::string text;
    u32 line = 1;
    u32 column = 1;
};

bool equals(const std::string& a, const char* b)
{
    usize i = 0;
    for (; i < a.size() && b[i]; ++i)
    {
        char ca = a[i];
        char cb = b[i];
        if (ca >= 'A' && ca <= 'Z')
            ca = static_cast<char>(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z')
            cb = static_cast<char>(cb - 'A' + 'a');
        if (ca != cb)
            return false;
    }
    return i == a.size() && b[i] == '\0';
}

u32 hashName(const std::string& name)
{
    u32 hash = 2166136261u;
    for (char c : name)
    {
        hash ^= static_cast<u8>(c);
        hash *= 16777619u;
    }
    return hash;
}

class Parser
{
public:
    Parser(const std::string& text, MaterialParseError* error) : mText(text), mError(error)
    {
        next();
    }

    bool run(std::vector<MaterialDefinition>& materials)
    {
        materials.clear();
        while (mToken.kind != TokenKind::End)
        {
            MaterialDefinition definition;
            if (!parseMaterial(definition))
            {
                materials.clear();
                return false;
            }
            // A duplicate name used to fail the whole file - one bad entry
            // discarding every other material in it, thousands strong on a
            // Bistro-sized export, is a worse outcome than the file itself.
            // Keep the first occurrence and skip the rest instead.
            bool duplicate = false;
            for (const MaterialDefinition& existing : materials)
            {
                if (existing.name == definition.name)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate)
                Log::warning("MaterialParser: duplicate material name '%s' at line %d - keeping "
                            "the first, skipping this one",
                            definition.name.c_str(), mToken.line);
            else
                materials.push_back(definition);
        }
        return true;
    }

private:
    bool parseMaterial(MaterialDefinition& out)
    {
        if (!word("material"))
            return fail("expected 'material'");
        next();
        if (!takeText(out.name))
            return fail("expected material name");
        out.material.name = out.name;
        out.material.nameHash = hashName(out.name);
        if (!take(TokenKind::LeftBrace, "expected '{' after material name"))
            return false;

        while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
        {
            if (word("blendMode"))
            {
                next();
                if (!parseBlend(out.material.blend))
                    return false;
            }
            else if (word("cullMode"))
            {
                next();
                if (!parseCull(out.material.cull))
                    return false;
            }
            else if (word("depthWrite"))
            {
                next();
                bool enabled;
                if (!parseBool(enabled))
                    return false;
                if (enabled)
                    out.material.flags &= ~MaterialNoDepthWrite;
                else
                    out.material.flags |= MaterialNoDepthWrite;
            }
            else if (word("flags"))
            {
                next();
                if (!parseFlags(out.material.flags))
                    return false;
            }
            else if (word("properties"))
            {
                next();
                if (!parseProperties(out.material))
                    return false;
            }
            else if (word("textures"))
            {
                next();
                if (!parseTextures(out))
                    return false;
            }
            else if (word("animations"))
            {
                next();
                if (!parseAnimations(out.material))
                    return false;
            }
            else
                return fail("unknown material field '" + mToken.text + "'");
        }
        return take(TokenKind::RightBrace, "expected '}' after material");
    }

    bool parseProperties(Material& material)
    {
        if (!take(TokenKind::LeftBrace, "expected '{' after properties"))
            return false;
        while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
        {
            const std::string name = mToken.text;
            if (mToken.kind != TokenKind::Word)
                return fail("expected property name");
            next();

            if (equals(name, "baseColor") || equals(name, "albedoColor"))
            {
                if (!parseVector(material.params.baseColor, 4))
                    return false;
            }
            else if (equals(name, "emissive") || equals(name, "emissiveColor"))
            {
                if (!parseVector(material.params.emissive, 3))
                    return false;
            }
            else if (equals(name, "surface"))
            {
                if (!parseVector(material.params.surface, 4))
                    return false;
            }
            else if (equals(name, "uvTransform"))
            {
                if (!parseVector(material.params.uvTransform, 4))
                    return false;
            }
            else if (equals(name, "uvAnim"))
            {
                if (!parseVector(material.params.uvAnim, 3))
                    return false;
            }
            else if (equals(name, "sequence"))
            {
                if (!parseVector(material.params.sequence, 4))
                    return false;
            }
            else if (equals(name, "custom0"))
            {
                if (!parseVector(material.params.custom0, 4))
                    return false;
            }
            else if (equals(name, "custom1"))
            {
                if (!parseVector(material.params.custom1, 4))
                    return false;
            }
            else
            {
                f32 value;
                if (!parseNumber(value))
                    return false;
                if (equals(name, "roughness"))
                    material.params.surface.x = value;
                else if (equals(name, "metalness"))
                    material.params.surface.y = value;
                else if (equals(name, "alphaCut"))
                    material.params.surface.z = value;
                else if (equals(name, "normalScale"))
                    material.params.surface.w = value;
                else if (equals(name, "distortion"))
                    material.params.custom0.x = value;
                // unlit.frag's flat-ambient path (200-233); 0 keeps the
                // shader's own default instead of forcing it to zero.
                else if (equals(name, "ambientStrength"))
                    material.params.custom1.x = value;
                else if (equals(name, "diffuseStrength"))
                    material.params.custom1.y = value;
                else if (equals(name, "flatLighting"))
                    material.params.custom1.z = value;
                else if (equals(name, "detailTiling"))
                    material.params.custom0.x = value;
                else if (equals(name, "detailStrength"))
                    material.params.custom0.y = value;
                else
                    return fail("unknown material property '" + name + "'");
            }
        }
        material.paramsDirty = true;
        return take(TokenKind::RightBrace, "expected '}' after properties");
    }

    bool parseTextures(MaterialDefinition& definition)
    {
        if (!take(TokenKind::LeftBrace, "expected '{' after textures"))
            return false;
        while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
        {
            if (!word("texture"))
                return fail("expected 'texture'");
            next();

            MaterialTextureSource texture;
            if (!takeText(texture.name))
                return fail("expected texture name");
            texture.slot = inferSlot(texture.name, definition.textures);
            if (!take(TokenKind::LeftBrace, "expected '{' after texture name"))
                return false;

            while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
            {
                if (word("mode") || word("type"))
                {
                    next();
                    if (!parseTextureSource(texture.source))
                        return false;
                }
                else if (word("slot"))
                {
                    next();
                    if (!parseSlot(texture.slot))
                        return false;
                }
                else if (word("file"))
                {
                    next();
                    if (!takeText(texture.file))
                        return fail("expected texture filename");
                }
                else if (word("source"))
                {
                    next();
                    if (!takeText(texture.target))
                        return fail("expected render target name");
                }
                else if (word("filter"))
                {
                    next();
                    if (!parseFilter(texture.filter))
                        return false;
                }
                else if (word("wrap"))
                {
                    next();
                    if (!parseWrap(texture.wrap))
                        return false;
                }
                else if (word("srgb"))
                {
                    next();
                    bool srgb = false;
                    if (!parseBool(srgb))
                        return false;
                    texture.colorSpace =
                        srgb ? ColorSpaceOverride::sRGB : ColorSpaceOverride::Linear;
                }
                else if (word("generateMips"))
                {
                    next();
                    if (!parseBool(texture.generateMips))
                        return false;
                }
                else if (word("fps"))
                {
                    next();
                    if (!parseNumber(definition.material.params.sequence.y))
                        return false;
                }
                else if (word("loop"))
                {
                    next();
                    bool loop;
                    if (!parseBool(loop))
                        return false;
                    definition.material.params.sequence.z = loop ? 1.0f : 0.0f;
                }
                else if (word("frames"))
                {
                    next();
                    if (!parseFrames(texture.frames))
                        return false;
                    texture.source = TextureSource::Sequence;
                    definition.material.params.sequence.x = static_cast<f32>(texture.frames.size());
                }
                else if (word("uvAnimation"))
                {
                    next();
                    if (!parseUvAnimation(texture))
                        return false;
                }
                else
                    return fail("unknown texture field '" + mToken.text + "'");
            }
            if (!take(TokenKind::RightBrace, "expected '}' after texture"))
                return false;
            if (texture.source == TextureSource::None && !texture.file.empty())
                texture.source = TextureSource::Static;
            if (texture.source == TextureSource::None)
                return fail("texture '" + texture.name + "' has no type or file");
            if (texture.source == TextureSource::Static && texture.file.empty())
                return fail("static texture '" + texture.name + "' has no file");
            if (texture.source == TextureSource::Sequence && texture.frames.empty())
                return fail("sequence texture '" + texture.name + "' has no frames");
            if (texture.source == TextureSource::RenderTarget && texture.target.empty())
                return fail("render target texture '" + texture.name + "' has no source");
            if (texture.slot >= MaterialSlotCount)
                return fail("material has no free texture slots");

            for (const MaterialTextureSource& existing : definition.textures)
            {
                if (existing.slot == texture.slot)
                    return fail("texture slot is already in use");
            }

            MaterialTexture& runtimeTexture = definition.material.textures[texture.slot];
            runtimeTexture.source = texture.source;
            runtimeTexture.layers = static_cast<u16>(texture.frames.size());
            runtimeTexture.targetName = hashName(texture.target);
            if (texture.scrollSpeed != glm::vec2(0.0f) || texture.rotateSpeed != 0.0f)
            {
                definition.material.params.uvAnim =
                    glm::vec4(texture.scrollSpeed, texture.rotateSpeed, 0.0f);
                definition.material.flags |= MaterialAnimated;
            }
            definition.textures.push_back(texture);
        }
        return take(TokenKind::RightBrace, "expected '}' after textures");
    }

    bool parseFrames(std::vector<std::string>& frames)
    {
        if (!take(TokenKind::LeftBracket, "expected '[' after frames"))
            return false;
        while (mToken.kind != TokenKind::RightBracket && mToken.kind != TokenKind::End)
        {
            std::string frame;
            if (!takeText(frame))
                return fail("expected frame filename");
            frames.push_back(frame);
            if (mToken.kind == TokenKind::Comma)
                next();
            else if (mToken.kind != TokenKind::RightBracket)
                return fail("expected ',' or ']' after frame");
        }
        return take(TokenKind::RightBracket, "expected ']' after frames");
    }

    bool parseUvAnimation(MaterialTextureSource& texture)
    {
        if (!take(TokenKind::LeftBrace, "expected '{' after uvAnimation"))
            return false;
        while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
        {
            if (word("scrollSpeed"))
            {
                next();
                glm::vec4 value(0.0f);
                if (!parseVector(value, 2))
                    return false;
                texture.scrollSpeed = glm::vec2(value);
            }
            else if (word("rotateSpeed"))
            {
                next();
                if (!parseNumber(texture.rotateSpeed))
                    return false;
            }
            else
                return fail("unknown uvAnimation field '" + mToken.text + "'");
        }
        return take(TokenKind::RightBrace, "expected '}' after uvAnimation");
    }

    bool parseAnimations(Material& material)
    {
        if (!take(TokenKind::LeftBrace, "expected '{' after animations"))
            return false;
        while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
        {
            if (mToken.kind != TokenKind::Word && mToken.kind != TokenKind::String)
                return fail("expected animation name");
            next();
            if (material.animCount >= Material::MaxAnims)
                return fail("too many material animations");
            if (!take(TokenKind::LeftBrace, "expected '{' after animation name"))
                return false;

            MaterialAnim& animation = material.anims[material.animCount];
            bool propertySeen = false;
            while (mToken.kind != TokenKind::RightBrace && mToken.kind != TokenKind::End)
            {
                if (word("property"))
                {
                    next();
                    std::string property;
                    if (!takeText(property) || !animationField(property, animation))
                        return fail("unknown animated property '" + property + "'");
                    propertySeen = true;
                }
                else if (word("type"))
                {
                    next();
                    if (!parseCurve(animation.curve))
                        return false;
                }
                else if (word("speed"))
                {
                    next();
                    if (!parseNumber(animation.speed))
                        return false;
                }
                else if (word("phase"))
                {
                    next();
                    if (!parseNumber(animation.phase))
                        return false;
                }
                else if (word("min"))
                {
                    next();
                    if (!parseVector(animation.min, 1))
                        return false;
                }
                else if (word("max"))
                {
                    next();
                    if (!parseVector(animation.max, 1))
                        return false;
                }
                else
                    return fail("unknown animation field '" + mToken.text + "'");
            }
            if (!take(TokenKind::RightBrace, "expected '}' after animation"))
                return false;
            if (!propertySeen)
                return fail("material animation has no property");
            ++material.animCount;
            material.flags |= MaterialAnimated;
        }
        return take(TokenKind::RightBrace, "expected '}' after animations");
    }

    bool animationField(const std::string& name, MaterialAnim& animation)
    {
        if (equals(name, "baseColor") || equals(name, "albedoColor"))
        {
            animation.field = 0;
            animation.mask = 0xF;
        }
        else if (equals(name, "emissive") || equals(name, "emissiveColor"))
        {
            animation.field = 1;
            animation.mask = 0x7;
        }
        else if (equals(name, "roughness"))
        {
            animation.field = 2;
            animation.mask = 0x1;
        }
        else if (equals(name, "metalness"))
        {
            animation.field = 2;
            animation.mask = 0x2;
        }
        else if (equals(name, "alphaCut"))
        {
            animation.field = 2;
            animation.mask = 0x4;
        }
        else if (equals(name, "distortion"))
        {
            animation.field = 6;
            animation.mask = 0x1;
        }
        else if (equals(name, "surface"))
        {
            animation.field = 2;
            animation.mask = 0xF;
        }
        else if (equals(name, "uvTransform"))
        {
            animation.field = 3;
            animation.mask = 0xF;
        }
        else if (equals(name, "uvAnim"))
        {
            animation.field = 4;
            animation.mask = 0x7;
        }
        else if (equals(name, "sequence"))
        {
            animation.field = 5;
            animation.mask = 0xF;
        }
        else if (equals(name, "custom0"))
        {
            animation.field = 6;
            animation.mask = 0xF;
        }
        else if (equals(name, "custom1"))
        {
            animation.field = 7;
            animation.mask = 0xF;
        }
        else
            return false;
        return true;
    }

    u8 inferSlot(const std::string& name, const std::vector<MaterialTextureSource>& used) const
    {
        if (equals(name, "albedo") || equals(name, "albedoMap") || equals(name, "baseColor"))
            return SlotAlbedo;
        if (equals(name, "normal") || equals(name, "normalMap") || equals(name, "waterNormal") ||
            equals(name, "normalFlowMap"))
            return SlotNormal;
        if (equals(name, "surface") || equals(name, "surfaceMap"))
            return SlotSurface;
        if (equals(name, "emissive") || equals(name, "emissiveMap"))
            return SlotEmissive;
        if (equals(name, "detail") || equals(name, "detailMap"))
            return SlotDetail;
        if (equals(name, "colorMap") || equals(name, "colourMap"))
            return SlotColorMap;
        if (equals(name, "lightmap") || equals(name, "lightMap"))
            return SlotLightmap;
        if (equals(name, "height") || equals(name, "heightMap") || equals(name, "parallax") ||
            equals(name, "parallaxMap"))
            return SlotHeight;

        for (u8 slot = SlotDetail; slot < MaterialSlotCount; ++slot) // Detail..Height
        {
            bool occupied = false;
            for (const MaterialTextureSource& texture : used)
                occupied |= texture.slot == slot;
            if (!occupied)
                return slot;
        }
        return MaterialSlotCount;
    }

    bool parseFlags(u32& flags)
    {
        if (!take(TokenKind::LeftBracket, "expected '[' after flags"))
            return false;

        flags = 0;
        while (mToken.kind != TokenKind::RightBracket && mToken.kind != TokenKind::End)
        {
            if (mToken.kind != TokenKind::Word)
                return fail("expected material flag");

            const u32 bit = MaterialManager::flagBit(mToken.text.c_str());
            if (bit == 0)
                return fail("invalid material flag '" + mToken.text + "'");
            flags |= bit;

            next();
            if (mToken.kind == TokenKind::Comma)
                next();
            else if (mToken.kind != TokenKind::RightBracket)
                return fail("expected ',' or ']' after material flag");
        }
        return take(TokenKind::RightBracket, "expected ']' after flags");
    }

    bool parseSlot(u8& slot)
    {
        if (word("Albedo"))
            slot = SlotAlbedo;
        else if (word("Normal"))
            slot = SlotNormal;
        else if (word("Surface"))
            slot = SlotSurface;
        else if (word("Emissive"))
            slot = SlotEmissive;
        else if (word("Detail"))
            slot = SlotDetail;
        else if (word("ColorMap"))
            slot = SlotColorMap;
        else if (word("Lightmap"))
            slot = SlotLightmap;
        else if (word("Height"))
            slot = SlotHeight;
        else
            return fail("invalid material texture slot '" + mToken.text + "'");
        next();
        return true;
    }

    bool parseVector(glm::vec4& value, u8 minimumComponents)
    {
        if (mToken.kind != TokenKind::LeftParen)
        {
            f32 scalar;
            if (!parseNumber(scalar))
                return false;
            value = glm::vec4(scalar);
            return true;
        }

        next();
        f32 components[4] = {0.0f, 0.0f, 0.0f, value.w};
        u8 count = 0;
        while (mToken.kind != TokenKind::RightParen && count < 4)
        {
            if (!parseNumber(components[count++]))
                return false;
            if (mToken.kind == TokenKind::Comma)
                next();
            else if (mToken.kind != TokenKind::RightParen)
                return fail("expected ',' or ')' in vector");
        }
        if (count < minimumComponents)
            return fail("not enough vector components");
        if (!take(TokenKind::RightParen, "expected ')' after vector"))
            return false;
        value = glm::vec4(components[0], components[1], components[2], components[3]);
        return true;
    }

    bool parseNumber(f32& value)
    {
        if (mToken.kind != TokenKind::Number)
            return fail("expected number");
        char* end = nullptr;
        value = std::strtof(mToken.text.c_str(), &end);
        if (end == mToken.text.c_str() || *end != '\0' || !std::isfinite(value))
            return fail("invalid number '" + mToken.text + "'");
        next();
        return true;
    }

    bool parseBool(bool& value)
    {
        if (word("true"))
            value = true;
        else if (word("false"))
            value = false;
        else
            return fail("expected true or false");
        next();
        return true;
    }

#define PARSE_ENUM(functionName, type, ...)                                                        \
    bool functionName(type& value)                                                                 \
    {                                                                                              \
        if (mToken.kind != TokenKind::Word)                                                        \
            return fail("expected " #type);                                                        \
        const struct                                                                               \
        {                                                                                          \
            const char* name;                                                                      \
            type value;                                                                            \
        } entries[] = {__VA_ARGS__};                                                               \
        for (const auto& entry : entries)                                                          \
        {                                                                                          \
            if (equals(mToken.text, entry.name))                                                   \
            {                                                                                      \
                value = entry.value;                                                               \
                next();                                                                            \
                return true;                                                                       \
            }                                                                                      \
        }                                                                                          \
        return fail("invalid " #type " '" + mToken.text + "'");                                    \
    }

    PARSE_ENUM(parseBlend, BlendMode, {"Opaque", BlendMode::Opaque}, {"Alpha", BlendMode::Alpha},
               {"Additive", BlendMode::Additive}, {"Multiply", BlendMode::Multiply},
               {"PremultipliedAlpha", BlendMode::PremultipliedAlpha},
               {"AddColors", BlendMode::AddColors}, {"SubtractColors", BlendMode::SubtractColors})
    PARSE_ENUM(parseCull, CullMode, {"None", CullMode::None}, {"Back", CullMode::Back},
               {"Front", CullMode::Front})
    PARSE_ENUM(parseTextureSource, TextureSource, {"Static", TextureSource::Static},
               {"Sequence", TextureSource::Sequence}, {"UVScroll", TextureSource::Static},
               {"RenderTarget", TextureSource::RenderTarget})
    PARSE_ENUM(parseFilter, Filter, {"Point", Filter::Point}, {"Linear", Filter::Linear},
               {"Trilinear", Filter::Trilinear}, {"Anisotropic", Filter::Anisotropic})
    PARSE_ENUM(parseWrap, Wrap, {"Repeat", Wrap::Repeat}, {"Mirror", Wrap::Mirror},
               {"Clamp", Wrap::Clamp}, {"Border", Wrap::Border})
    PARSE_ENUM(parseCurve, Curve, {"Linear", Curve::Linear}, {"SineWave", Curve::SineWave},
               {"PingPong", Curve::PingPong}, {"Noise", Curve::Noise})

#undef PARSE_ENUM

    bool takeText(std::string& value)
    {
        if (mToken.kind != TokenKind::String && mToken.kind != TokenKind::Word)
            return false;
        value = mToken.text;
        next();
        return true;
    }

    bool take(TokenKind kind, const char* message)
    {
        if (mToken.kind != kind)
            return fail(message);
        next();
        return true;
    }

    bool word(const char* value) const
    {
        return mToken.kind == TokenKind::Word && equals(mToken.text, value);
    }

    bool fail(const std::string& message)
    {
        if (mError)
        {
            mError->message = message;
            mError->line = mToken.line;
            mError->column = mToken.column;
        }
        return false;
    }

    void next()
    {
        skipWhitespace();
        mToken = Token();
        mToken.line = mLine;
        mToken.column = mColumn;
        if (mPos >= mText.size())
            return;

        const char c = advance();
        switch (c)
        {
        case '{':
            mToken.kind = TokenKind::LeftBrace;
            return;
        case '}':
            mToken.kind = TokenKind::RightBrace;
            return;
        case '[':
            mToken.kind = TokenKind::LeftBracket;
            return;
        case ']':
            mToken.kind = TokenKind::RightBracket;
            return;
        case '(':
            mToken.kind = TokenKind::LeftParen;
            return;
        case ')':
            mToken.kind = TokenKind::RightParen;
            return;
        case ',':
            mToken.kind = TokenKind::Comma;
            return;
        case '"':
            mToken.kind = TokenKind::String;
            while (mPos < mText.size() && mText[mPos] != '"')
            {
                const char value = advance();
                if (value == '\\' && mPos < mText.size())
                    mToken.text += advance();
                else
                    mToken.text += value;
            }
            if (mPos >= mText.size())
            {
                // Do not accept a truncated path/name as if EOF closed it.
                // Keeping a non-text token makes the surrounding production
                // fail at the exact unterminated value.
                mToken.kind = TokenKind::End;
                return;
            }
            advance();
            return;
        default:
            break;
        }

        const bool number = (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
        mToken.kind = number ? TokenKind::Number : TokenKind::Word;
        mToken.text += c;
        while (mPos < mText.size())
        {
            const char value = mText[mPos];
            const bool numeric = (value >= '0' && value <= '9') || value == '.' || value == 'e' ||
                                 value == 'E' || value == '-' || value == '+';
            const bool identifier = (value >= 'a' && value <= 'z') ||
                                    (value >= 'A' && value <= 'Z') || value == '_' ||
                                    (value >= '0' && value <= '9');
            if ((number && !numeric) || (!number && !identifier))
                break;
            mToken.text += advance();
        }
    }

    void skipWhitespace()
    {
        while (mPos < mText.size())
        {
            const char c = mText[mPos];
            if (c == '/' && mPos + 1 < mText.size() && mText[mPos + 1] == '/')
            {
                while (mPos < mText.size() && mText[mPos] != '\n')
                    advance();
            }
            else if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
                advance();
            else
                break;
        }
    }

    char advance()
    {
        const char c = mText[mPos++];
        if (c == '\n')
        {
            ++mLine;
            mColumn = 1;
        }
        else
            ++mColumn;
        return c;
    }

    const std::string& mText;
    MaterialParseError* mError;
    Token mToken;
    usize mPos = 0;
    u32 mLine = 1;
    u32 mColumn = 1;
};

} // namespace

bool MaterialParser::parse(const std::string& text, std::vector<MaterialDefinition>& materials,
                           MaterialParseError* error)
{
    if (error)
        *error = MaterialParseError();
    Parser parser(text, error);
    return parser.run(materials);
}

bool MaterialParser::parseFile(const std::string& filename,
                               std::vector<MaterialDefinition>& materials,
                               MaterialParseError* error)
{
    const std::string text = FileSystem::getSingleton().readText(filename);
    if (text.empty())
    {
        materials.clear();
        if (error)
            error->message = "could not read material file '" + filename + "'";
        return false;
    }
    return parse(text, materials, error);
}

} // namespace Radion

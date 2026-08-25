#include "PCH.h"

#include "ScriptProperty.h"

#include <cstdlib>
#include <cstring>

namespace Radion
{

static bool isIdentStart(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

static bool isIdentChar(char c)
{
    return isIdentStart(c) || (c >= '0' && c <= '9');
}

static const char* skipBlanks(const char* p, const char* end)
{
    while (p < end && (*p == ' ' || *p == '\t'))
        ++p;
    return p;
}

static int indentOf(const char* p, const char* end)
{
    int indent = 0;
    while (p < end && (*p == ' ' || *p == '\t'))
        indent += (*p++ == '\t') ? 4 : 1;
    return indent;
}

static const char* parseIdent(const char* p, const char* end, std::string& out)
{
    if (p >= end || !isIdentStart(*p))
        return nullptr;
    const char* start = p;
    while (p < end && isIdentChar(*p))
        ++p;
    out.assign(start, (usize)(p - start));
    return p;
}

static const char* parseString(const char* p, const char* end, std::string& out)
{
    const char quote = *p++;
    out.clear();
    while (p < end && *p != quote)
    {
        if (*p == '\\' && p + 1 < end)
        {
            ++p;
            switch (*p)
            {
            case 'n': out.push_back('\n'); break;
            case 't': out.push_back('\t'); break;
            case 'r': out.push_back('\r'); break;
            default: out.push_back(*p); break;
            }
            ++p;
            continue;
        }
        out.push_back(*p++);
    }
    if (p >= end)
        return nullptr;
    return p + 1;
}

static const char* parseLiteral(const char* p, const char* end, ScriptProperty& out)
{
    p = skipBlanks(p, end);
    if (p >= end)
        return nullptr;

    if (*p == '"' || *p == '\'')
    {
        const char* after = parseString(p, end, out.text);
        if (!after)
            return nullptr;
        out.kind = ScriptProperty::Kind::String;
        return after;
    }

    if (isIdentStart(*p))
    {
        std::string word;
        const char* after = parseIdent(p, end, word);
        if (!after || (word != "True" && word != "False"))
            return nullptr;
        out.kind = ScriptProperty::Kind::Bool;
        out.flag = word == "True";
        return after;
    }

    char buffer[64];
    const usize span = (usize)(end - p) < sizeof(buffer) - 1 ? (usize)(end - p)
                                                             : sizeof(buffer) - 1;
    std::memcpy(buffer, p, span);
    buffer[span] = '\0';

    char* stop = nullptr;
    const f64 value = std::strtod(buffer, &stop);
    if (!stop || stop == buffer)
        return nullptr;

    out.kind = ScriptProperty::Kind::Number;
    out.number = value;
    out.integer = std::memchr(buffer, '.', (usize)(stop - buffer)) == nullptr &&
                  std::memchr(buffer, 'e', (usize)(stop - buffer)) == nullptr &&
                  std::memchr(buffer, 'E', (usize)(stop - buffer)) == nullptr;
    return p + (stop - buffer);
}

static bool endsStatement(const char* p, const char* end)
{
    p = skipBlanks(p, end);
    return p >= end || *p == '#';
}

static const ScriptProperty* findNamed(const std::vector<ScriptProperty>& list,
                                       const std::string& name)
{
    for (usize i = 0; i < list.size(); ++i)
        if (list[i].name == name)
            return &list[i];
    return nullptr;
}

static bool isAssignment(const char* p, const char* end)
{
    return p < end && *p == '=' && (p + 1 >= end || p[1] != '=');
}

usize ScriptProperties::scan(const char* source, std::vector<ScriptProperty>& out)
{
    out.clear();
    if (!source)
        return 0;

    std::vector<ScriptProperty> constants;
    bool inInit = false;
    int initIndent = 0;

    const char* cursor = source;
    while (*cursor)
    {
        const char* lineEnd = std::strchr(cursor, '\n');
        if (!lineEnd)
            lineEnd = cursor + std::strlen(cursor);
        const char* next = *lineEnd ? lineEnd + 1 : lineEnd;
        if (lineEnd > cursor && lineEnd[-1] == '\r')
            --lineEnd;

        const int indent = indentOf(cursor, lineEnd);
        const char* p = skipBlanks(cursor, lineEnd);

        if (p >= lineEnd || *p == '#')
        {
            cursor = next;
            continue;
        }

        // Dedenting out of __init__ ends the scan: anything past it is
        // another method, and its locals are not properties.
        if (inInit && indent <= initIndent)
            break;

        std::string name;
        if (inInit)
        {
            if (std::strncmp(p, "self.", 5) == 0)
            {
                const char* after = parseIdent(p + 5, lineEnd, name);
                if (after)
                {
                    after = skipBlanks(after, lineEnd);
                    if (isAssignment(after, lineEnd) && name[0] != '_' && !findNamed(out, name))
                    {
                        ScriptProperty prop;
                        prop.name = name;
                        const char* value = parseLiteral(after + 1, lineEnd, prop);
                        if (value && endsStatement(value, lineEnd))
                        {
                            out.push_back(prop);
                        }
                        else
                        {
                            std::string constant;
                            const char* ident =
                                parseIdent(skipBlanks(after + 1, lineEnd), lineEnd, constant);
                            const ScriptProperty* known =
                                ident ? findNamed(constants, constant) : nullptr;
                            if (known && endsStatement(ident, lineEnd))
                            {
                                prop.kind = known->kind;
                                prop.number = known->number;
                                prop.text = known->text;
                                prop.flag = known->flag;
                                prop.integer = known->integer;
                                out.push_back(prop);
                            }
                        }
                    }
                }
            }
            cursor = next;
            continue;
        }

        if (std::strncmp(p, "def", 3) == 0 && (p + 3 < lineEnd) && (p[3] == ' ' || p[3] == '\t'))
        {
            std::string method;
            if (parseIdent(skipBlanks(p + 3, lineEnd), lineEnd, method) && method == "__init__")
            {
                inInit = true;
                initIndent = indent;
            }
            cursor = next;
            continue;
        }

        if (indent == 0)
        {
            const char* after = parseIdent(p, lineEnd, name);
            if (after)
            {
                after = skipBlanks(after, lineEnd);
                if (isAssignment(after, lineEnd))
                {
                    ScriptProperty constant;
                    constant.name = name;
                    const char* value = parseLiteral(after + 1, lineEnd, constant);
                    if (value && endsStatement(value, lineEnd))
                        constants.push_back(constant);
                }
            }
        }

        cursor = next;
    }

    return out.size();
}

} // namespace Radion

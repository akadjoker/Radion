"""Turns the editor's `/api/commands` listing into OpenAI-style tool specs.

Nothing about the commands is duplicated here: a command added to the editor shows up as
a tool the next time the listing is fetched.
"""

import copy

# Schema-level keywords that some OpenAI-compatible servers reject or mis-handle when they
# turn the schema into a grammar (constrained decoding). Dropped in "simplified" mode.
_UNSUPPORTED_KEYWORDS = frozenset({
    "$schema", "$id", "$ref", "$defs", "definitions", "additionalProperties",
    "minItems", "maxItems", "uniqueItems", "minimum", "maximum", "exclusiveMinimum",
    "exclusiveMaximum", "multipleOf", "minLength", "maxLength", "pattern", "format",
    "default", "examples", "title", "const", "not", "allOf", "if", "then", "else",
})

_ARTICLES = {"integer": "an integer", "array": "an array", "object": "an object"}


def build_tools(commands, simplify=False):
    """OpenAI `tools` list, one function per command."""
    tools = []
    for command in commands:
        schema = command.get("inputSchema") or {}
        parameters = simplify_schema(schema) if simplify else copy.deepcopy(schema)
        # Servers differ on whether a function without parameters may omit them.
        parameters.setdefault("type", "object")
        parameters.setdefault("properties", {})
        tools.append({
            "type": "function",
            "function": {
                "name": command["name"],
                "description": command.get("description", ""),
                "parameters": parameters,
            },
        })
    return tools


def simplify_schema(schema):
    """A permissive copy of a JSON Schema for picky servers.

    `oneOf`/`anyOf` is dropped (together with the node's own `type`) and described in the
    `description` instead, so the model still learns e.g. that `scale` takes a number or
    three numbers. Array size limits are also moved into the description, and keywords
    in _UNSUPPORTED_KEYWORDS are removed. `type`, `enum`, `properties`, `items` and
    `required` survive.
    """
    if not isinstance(schema, dict):
        return schema
    simple = {}
    notes = []
    for key, value in schema.items():
        if key in ("oneOf", "anyOf") or key in _UNSUPPORTED_KEYWORDS:
            continue
        if key == "properties":
            simple[key] = {name: simplify_schema(sub) for name, sub in value.items()}
        elif key == "items":
            simple[key] = simplify_schema(value)
        else:
            simple[key] = copy.deepcopy(value)

    alternatives = schema.get("oneOf") or schema.get("anyOf")
    if alternatives:
        simple.pop("type", None)
        notes.append("Accepts " + " or ".join(_describe(alt) for alt in alternatives) + ".")
    elif schema.get("type") == "array":
        size = _size_phrase(schema, schema.get("items", {}).get("type", "value"))
        if size:
            notes.append(size[0].upper() + size[1:] + ".")

    if notes:
        simple["description"] = " ".join(filter(None, [schema.get("description", ""), *notes]))
    return simple


def _describe(schema):
    """Short English phrase for one alternative of a union."""
    if "enum" in schema:
        return "one of " + ", ".join(_literal(v) for v in schema["enum"])
    kind = schema.get("type", "any value")
    if kind == "array":
        item = schema.get("items", {}).get("type", "value")
        return f"an array of {_size_phrase(schema, item) or item + 's'}"
    return _ARTICLES.get(kind, f"a {kind}")


def _size_phrase(schema, item="value"):
    low, high = schema.get("minItems"), schema.get("maxItems")
    if low is None and high is None:
        return ""
    if low == high:
        return f"exactly {low} {item}s"
    if low is None:
        return f"at most {high} {item}s"
    if high is None:
        return f"at least {low} {item}s"
    return f"{low} to {high} {item}s"


def _literal(value):
    return f'"{value}"' if isinstance(value, str) else str(value)

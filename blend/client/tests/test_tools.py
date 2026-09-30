import copy
import json

from radion_chat.tools import build_tools, simplify_schema


def walk(node):
    """Every dict inside a schema, however deep."""
    if isinstance(node, dict):
        yield node
        for value in node.values():
            yield from walk(value)
    elif isinstance(node, list):
        for value in node:
            yield from walk(value)


def command(commands, name):
    return next(c for c in commands if c["name"] == name)


def test_one_function_tool_per_command(commands):
    tools = build_tools(commands)
    assert len(tools) == len(commands) == 39
    first = tools[0]
    assert first["type"] == "function"
    assert set(first["function"]) == {"name", "description", "parameters"}
    assert {t["function"]["name"] for t in tools} == {c["name"] for c in commands}


def test_default_mode_keeps_schema_untouched_and_does_not_alias(commands):
    original = command(commands, "add_loft")["inputSchema"]
    tool = next(t for t in build_tools(commands) if t["function"]["name"] == "add_loft")
    assert tool["function"]["parameters"] == original
    tool["function"]["parameters"]["properties"]["axis"]["enum"].append("w")
    assert "w" not in original["properties"]["axis"]["enum"]


def test_commands_without_properties_still_get_an_object_schema():
    tools = build_tools([{"name": "center", "description": "d", "inputSchema": {}}])
    assert tools[0]["function"]["parameters"] == {"type": "object", "properties": {}}


def test_real_schemas_contain_the_keywords_we_simplify(commands):
    keywords = {key for c in commands for node in walk(c["inputSchema"]) for key in node}
    assert {"oneOf", "minItems", "maxItems"} <= keywords  # otherwise these tests prove nothing


def test_simplified_real_schemas_have_no_unions_or_limits(commands):
    for tool in build_tools(commands, simplify=True):
        for node in walk(tool["function"]["parameters"]):
            banned = {"oneOf", "anyOf", "minItems", "maxItems", "additionalProperties"}
            assert not banned & set(node), tool["function"]["name"]


def test_union_integer_or_string_becomes_described_and_untyped(commands):
    part = simplify_schema(command(commands, "transform_part")["inputSchema"])["properties"]["part"]
    assert "type" not in part
    assert "oneOf" not in part
    assert "an integer or a string" in part["description"]
    assert part["description"].startswith("A part, by its index")  # original text kept


def test_union_number_or_array_describes_the_array(commands):
    scale = simplify_schema(command(commands, "add_primitive")["inputSchema"])["properties"]["scale"]
    assert "type" not in scale
    assert "a number or an array of exactly 3 numbers" in scale["description"]


def test_colour_union_with_size_range(commands):
    color = simplify_schema(command(commands, "add_primitive")["inputSchema"])["properties"]["color"]
    assert "a string or an array of 3 to 4 numbers" in color["description"]


def test_plain_array_limits_move_into_description(commands):
    position = simplify_schema(command(commands, "add_primitive")["inputSchema"])["properties"]["position"]
    assert position["type"] == "array"
    assert position["items"] == {"type": "number"}
    assert "Exactly 3 numbers." in position["description"]


def test_nested_objects_are_simplified(commands):
    sections = simplify_schema(command(commands, "add_loft")["inputSchema"])["properties"]["sections"]
    section = sections["items"]
    assert section["type"] == "object"
    assert "at" in section["properties"]
    assert "required" in section
    assert not {"minItems", "maxItems", "oneOf"} & {k for n in walk(sections) for k in n}


def test_enums_types_and_required_survive(commands):
    schema = command(commands, "screenshot")["inputSchema"]
    simple = simplify_schema(schema)
    assert simple["properties"]["view"]["enum"] == schema["properties"]["view"]["enum"]
    assert simple["properties"]["width"]["type"] == "integer"
    select = simplify_schema(command(commands, "select")["inputSchema"])
    assert select["properties"]["box"]["required"] == ["min", "max"]


def test_property_names_that_look_like_keywords_are_kept():
    schema = {"type": "object", "properties": {
        "format": {"type": "string", "default": "x", "pattern": "a"},
        "default": {"type": "integer", "minimum": 0}}}
    simple = simplify_schema(schema)
    assert simple == {"type": "object", "properties": {"format": {"type": "string"}, "default": {"type": "integer"}}}


def test_simplify_does_not_mutate_its_input_and_is_idempotent(commands):
    for c in commands:
        before = copy.deepcopy(c["inputSchema"])
        once = simplify_schema(c["inputSchema"])
        assert c["inputSchema"] == before
        assert simplify_schema(once) == once


def test_simplified_tools_are_json_serialisable(commands):
    json.dumps(build_tools(commands, simplify=True))

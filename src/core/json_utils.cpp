#include "robot_process_platform/core/json_utils.h"

#include <cctype>
#include <sstream>
#include <stdexcept>

namespace robot_process_platform::core::json
{

JsonParser::JsonParser(const std::string& input_text)
    : text(input_text) {}

JsonValue JsonParser::Parse()
{
    SkipWhitespace();
    JsonValue value = ParseValue();
    SkipWhitespace();
    if (position != text.size())
    {
        throw std::runtime_error("json_has_trailing_content");
    }

    return value;
}

JsonValue JsonParser::ParseValue()
{
    SkipWhitespace();
    if (position >= text.size())
    {
        throw std::runtime_error("unexpected_end_of_json");
    }

    if (text[position] == '"')
    {
        JsonValue value;
        value.type = JsonValueType::String;
        value.string_value = ParseString();
        return value;
    }

    if (text[position] == '{')
    {
        return ParseObject();
    }

    if (text[position] == '[')
    {
        return ParseArray();
    }

    if (text[position] == 't')
    {
        return ParseTrue();
    }

    if (text[position] == 'f')
    {
        return ParseFalse();
    }

    if (text[position] == 'n')
    {
        return ParseNull();
    }

    if (text[position] == '-' || std::isdigit(static_cast<unsigned char>(text[position])))
    {
        JsonValue value;
        value.type = JsonValueType::Number;
        value.number_value = ParseNumber();
        return value;
    }

    throw std::runtime_error("unsupported_json_value_type");
}

JsonValue JsonParser::ParseObject()
{
    Expect('{');

    JsonValue value;
    value.type = JsonValueType::Object;

    SkipWhitespace();
    if (Consume('}'))
    {
        return value;
    }

    while (true)
    {
        const std::string key = ParseString();
        SkipWhitespace();
        Expect(':');
        value.object_value.emplace(key, ParseValue());

        SkipWhitespace();
        if (Consume('}'))
        {
            return value;
        }

        Expect(',');
    }
}

JsonValue JsonParser::ParseArray()
{
    Expect('[');

    JsonValue value;
    value.type = JsonValueType::Array;

    SkipWhitespace();
    if (Consume(']'))
    {
        return value;
    }

    while (true)
    {
        value.array_value.push_back(ParseValue());

        SkipWhitespace();
        if (Consume(']'))
        {
            return value;
        }

        Expect(',');
    }
}

JsonValue JsonParser::ParseTrue()
{
    ExpectKeyword("true");
    JsonValue value;
    value.type = JsonValueType::Bool;
    value.bool_value = true;
    return value;
}

JsonValue JsonParser::ParseFalse()
{
    ExpectKeyword("false");
    JsonValue value;
    value.type = JsonValueType::Bool;
    value.bool_value = false;
    return value;
}

JsonValue JsonParser::ParseNull()
{
    ExpectKeyword("null");
    JsonValue value;
    value.type = JsonValueType::Null;
    return value;
}

std::string JsonParser::ParseString()
{
    Expect('"');

    std::string output_text;
    while (position < text.size())
    {
        const char character = text[position++];
        if (character == '"')
        {
            return output_text;
        }

        if (character == '\\')
        {
            if (position >= text.size())
            {
                throw std::runtime_error("invalid_json_escape");
            }

            output_text.push_back(text[position++]);
            continue;
        }

        output_text.push_back(character);
    }

    throw std::runtime_error("unterminated_json_string");
}

std::string JsonParser::ParseNumber()
{
    const std::size_t number_start = position;
    if (text[position] == '-')
    {
        ++position;
    }

    while (position < text.size() &&
           std::isdigit(static_cast<unsigned char>(text[position])))
    {
        ++position;
    }

    if (position < text.size() && text[position] == '.')
    {
        ++position;
        while (position < text.size() &&
               std::isdigit(static_cast<unsigned char>(text[position])))
        {
            ++position;
        }
    }

    return text.substr(number_start, position - number_start);
}

void JsonParser::SkipWhitespace()
{
    while (position < text.size() &&
           std::isspace(static_cast<unsigned char>(text[position])))
    {
        ++position;
    }
}

bool JsonParser::Consume(char expected_character)
{
    SkipWhitespace();
    if (position < text.size() && text[position] == expected_character)
    {
        ++position;
        return true;
    }

    return false;
}

void JsonParser::Expect(char expected_character)
{
    SkipWhitespace();
    if (position >= text.size() || text[position] != expected_character)
    {
        throw std::runtime_error("unexpected_json_character");
    }

    ++position;
}

void JsonParser::ExpectKeyword(const std::string& keyword)
{
    if (text.substr(position, keyword.size()) != keyword)
    {
        throw std::runtime_error("unexpected_json_keyword");
    }

    position += keyword.size();
}

std::string EscapeJsonString(const std::string& raw_text)
{
    std::string escaped_text;
    for (const char character : raw_text)
    {
        if (character == '"' || character == '\\')
        {
            escaped_text.push_back('\\');
        }

        escaped_text.push_back(character);
    }

    return escaped_text;
}

std::string SerializeJsonValue(const JsonValue& json_value)
{
    std::ostringstream output_stream;

    switch (json_value.type)
    {
        case JsonValueType::Null:
            output_stream << "null";
            break;
        case JsonValueType::Bool:
            output_stream << (json_value.bool_value ? "true" : "false");
            break;
        case JsonValueType::String:
            output_stream << "\"" << EscapeJsonString(json_value.string_value) << "\"";
            break;
        case JsonValueType::Number:
            output_stream << json_value.number_value;
            break;
        case JsonValueType::Object:
        {
            output_stream << "{";
            std::size_t field_index = 0;
            for (const auto& field_entry : json_value.object_value)
            {
                if (field_index > 0)
                {
                    output_stream << ", ";
                }

                output_stream << "\"" << EscapeJsonString(field_entry.first) << "\": "
                              << SerializeJsonValue(field_entry.second);
                ++field_index;
            }
            output_stream << "}";
            break;
        }
        case JsonValueType::Array:
        {
            output_stream << "[";
            for (std::size_t index = 0; index < json_value.array_value.size(); ++index)
            {
                if (index > 0)
                {
                    output_stream << ", ";
                }

                output_stream << SerializeJsonValue(json_value.array_value[index]);
            }
            output_stream << "]";
            break;
        }
    }

    return output_stream.str();
}

const JsonValue& GetObjectField(const JsonValue& object_value, const std::string& field_name)
{
    if (object_value.type != JsonValueType::Object)
    {
        throw std::runtime_error("json_value_is_not_object");
    }

    const auto field_it = object_value.object_value.find(field_name);
    if (field_it == object_value.object_value.end())
    {
        throw std::runtime_error("missing_json_field: " + field_name);
    }

    return field_it->second;
}

std::string GetStringField(const JsonValue& object_value, const std::string& field_name)
{
    const JsonValue& field_value = GetObjectField(object_value, field_name);
    if (field_value.type != JsonValueType::String)
    {
        throw std::runtime_error("json_field_is_not_string: " + field_name);
    }

    return field_value.string_value;
}

std::string GetNumberFieldAsString(const JsonValue& object_value, const std::string& field_name)
{
    const JsonValue& field_value = GetObjectField(object_value, field_name);
    if (field_value.type != JsonValueType::Number)
    {
        throw std::runtime_error("json_field_is_not_number: " + field_name);
    }

    return field_value.number_value;
}

int GetIntField(const JsonValue& object_value, const std::string& field_name)
{
    return std::stoi(GetNumberFieldAsString(object_value, field_name));
}

const std::vector<JsonValue>& GetArrayField(const JsonValue& object_value, const std::string& field_name)
{
    const JsonValue& field_value = GetObjectField(object_value, field_name);
    if (field_value.type != JsonValueType::Array)
    {
        throw std::runtime_error("json_field_is_not_array: " + field_name);
    }

    return field_value.array_value;
}

std::map<std::string, std::string> GetStringMapField(const JsonValue& object_value,
                                                     const std::string& field_name)
{
    const JsonValue& field_value = GetObjectField(object_value, field_name);
    if (field_value.type != JsonValueType::Object)
    {
        throw std::runtime_error("json_field_is_not_object: " + field_name);
    }

    std::map<std::string, std::string> output_map;
    for (const auto& field_entry : field_value.object_value)
    {
        if (field_entry.second.type != JsonValueType::String)
        {
            throw std::runtime_error("json_map_value_is_not_string: " + field_name);
        }

        output_map[field_entry.first] = field_entry.second.string_value;
    }

    return output_map;
}

}  // namespace robot_process_platform::core::json

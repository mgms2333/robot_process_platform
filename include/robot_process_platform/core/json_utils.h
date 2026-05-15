#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot_process_platform::core::json
{

enum class JsonValueType
{
    Null,
    Bool,
    String,
    Number,
    Object,
    Array
};

struct JsonValue
{
    JsonValueType type = JsonValueType::Object;
    bool bool_value = false;
    std::string string_value;
    std::string number_value;
    std::map<std::string, JsonValue> object_value;
    std::vector<JsonValue> array_value;
};

class JsonParser
{
public:
    explicit JsonParser(const std::string& input_text);

    JsonValue Parse();

private:
    JsonValue ParseValue();
    JsonValue ParseObject();
    JsonValue ParseArray();
    JsonValue ParseTrue();
    JsonValue ParseFalse();
    JsonValue ParseNull();
    std::string ParseString();
    std::string ParseNumber();
    void SkipWhitespace();
    bool Consume(char expected_character);
    void Expect(char expected_character);
    void ExpectKeyword(const std::string& keyword);

    const std::string& text;
    std::size_t position = 0;
};

std::string EscapeJsonString(const std::string& raw_text);
std::string SerializeJsonValue(const JsonValue& json_value);

const JsonValue& GetObjectField(const JsonValue& object_value, const std::string& field_name);
std::string GetStringField(const JsonValue& object_value, const std::string& field_name);
std::string GetNumberFieldAsString(const JsonValue& object_value, const std::string& field_name);
int GetIntField(const JsonValue& object_value, const std::string& field_name);
const std::vector<JsonValue>& GetArrayField(const JsonValue& object_value, const std::string& field_name);
std::map<std::string, std::string> GetStringMapField(const JsonValue& object_value,
                                                     const std::string& field_name);

}  // namespace robot_process_platform::core::json

#include "FlightPlanJson.hpp"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
// Strict parser for the deliberately small route schema: objects, arrays,
// ASCII field names and JSON numbers. It rejects unsupported JSON constructs.
class RouteJsonParser {
public:
    explicit RouteJsonParser(const std::string& input) : input_(input) {}

    std::vector<Segment> parse(double simulation_end_time_s) {
        consume('{');
        std::vector<Segment> segments;
        bool found_segments = false;
        do {
            const std::string field = readString();
            consume(':');
            if (field == "segments" && !found_segments) {
                consume('[');
                if (!peek(']')) {
                    do {
                        segments.push_back(readSegment());
                    } while (consumeIf(','));
                }
                consume(']');
                found_segments = true;
            } else {
                fail("Unknown or duplicate top-level field '" + field + "'");
            }
        } while (consumeIf(','));
        consume('}');
        if (!found_segments) fail("Expected 'segments'");
        for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
            segments[segment_index].end_time_s = segment_index + 1 < segments.size() ?
                segments[segment_index + 1].start_time_s : simulation_end_time_s;
        }
        skipWhitespace();
        if (cursor_ != input_.size()) fail("Unexpected content after flight plan");
        return segments;
    }

private:
    const std::string& input_;
    std::size_t cursor_ = 0;

    void skipWhitespace() {
        while (cursor_ < input_.size() &&
               (input_[cursor_] == ' ' || input_[cursor_] == '\n' ||
                input_[cursor_] == '\r' || input_[cursor_] == '\t')) ++cursor_;
    }

    bool peek(char expected) {
        skipWhitespace();
        return cursor_ < input_.size() && input_[cursor_] == expected;
    }

    bool consumeIf(char expected) {
        if (!peek(expected)) return false;
        ++cursor_;
        return true;
    }

    void consume(char expected) {
        if (!consumeIf(expected)) fail(std::string("Expected '") + expected + "'");
    }

    std::string readString() {
        consume('"');
        std::string value;
        while (cursor_ < input_.size()) {
            const char character = input_[cursor_++];
            if (character == '"') return value;
            if (character == '\\') {
                if (cursor_ >= input_.size()) fail("Incomplete string escape");
                const char escaped = input_[cursor_++];
                switch (escaped) {
                    case '"': case '\\': case '/': value += escaped; break;
                    case 'b': value += '\b'; break;
                    case 'f': value += '\f'; break;
                    case 'n': value += '\n'; break;
                    case 'r': value += '\r'; break;
                    case 't': value += '\t'; break;
                    default: fail("Unsupported string escape");
                }
            } else {
                if (static_cast<unsigned char>(character) < 0x20) fail("Control character in string");
                value += character;
            }
        }
        fail("Unterminated string");
        return {};
    }

    double readNumber() {
        skipWhitespace();
        const std::size_t start = cursor_;
        if (cursor_ < input_.size() && input_[cursor_] == '-') ++cursor_;
        if (cursor_ >= input_.size()) fail("Expected number");
        if (input_[cursor_] == '0') {
            ++cursor_;
            if (cursor_ < input_.size() && input_[cursor_] >= '0' && input_[cursor_] <= '9')
                fail("Leading zero in number");
        } else {
            if (input_[cursor_] < '1' || input_[cursor_] > '9') fail("Expected number");
            while (cursor_ < input_.size() && input_[cursor_] >= '0' && input_[cursor_] <= '9') ++cursor_;
        }
        if (cursor_ < input_.size() && input_[cursor_] == '.') {
            ++cursor_;
            const std::size_t first_digit = cursor_;
            while (cursor_ < input_.size() && input_[cursor_] >= '0' && input_[cursor_] <= '9') ++cursor_;
            if (first_digit == cursor_) fail("Expected digit after decimal point");
        }
        if (cursor_ < input_.size() && (input_[cursor_] == 'e' || input_[cursor_] == 'E')) {
            ++cursor_;
            if (cursor_ < input_.size() && (input_[cursor_] == '+' || input_[cursor_] == '-')) ++cursor_;
            const std::size_t first_digit = cursor_;
            while (cursor_ < input_.size() && input_[cursor_] >= '0' && input_[cursor_] <= '9') ++cursor_;
            if (first_digit == cursor_) fail("Expected exponent digit");
        }
        const std::string token = input_.substr(start, cursor_ - start);
        errno = 0;
        const double result = std::strtod(token.c_str(), nullptr);
        if (errno == ERANGE || !std::isfinite(result)) fail("Number is outside finite range");
        return result;
    }

    Segment readSegment() {
        consume('{');
        std::unordered_map<std::string, double> values;
        if (!peek('}')) {
            do {
                const std::string name = readString();
                consume(':');
                if (!values.emplace(name, readNumber()).second) fail("Duplicate field '" + name + "'");
            } while (consumeIf(','));
        }
        consume('}');
        static const char* required[] = {
            "start_s", "target_bank_deg", "roll_rate_deg_s",
            "target_speed_mps", "acceleration_mps2", "start_altitude_m",
            "end_altitude_m", "pitch_rate_deg_s"
        };
        for (const char* name : required) {
            if (!values.count(name)) fail(std::string("Missing field '") + name + "'");
        }
        if (values.size() != sizeof(required) / sizeof(required[0]))
            fail("Segment contains an unknown field");
        return {values.at("start_s"), 0.0, values.at("target_bank_deg"),
                values.at("roll_rate_deg_s"), values.at("target_speed_mps"),
                values.at("acceleration_mps2"), values.at("start_altitude_m"),
                values.at("end_altitude_m"), values.at("pitch_rate_deg_s")};
    }

    [[noreturn]] void fail(const std::string& message) const {
        throw std::invalid_argument("Route JSON at byte " + std::to_string(cursor_) + ": " + message);
    }
};
} // namespace

FlightPlan loadFlightPlanJson(const std::string& path, double simulation_end_time_s) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open flight plan: " + path);
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (!file.eof() && file.bad()) throw std::runtime_error("Cannot read flight plan: " + path);
    return FlightPlan(RouteJsonParser(content).parse(simulation_end_time_s));
}

// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_GENERATOR_UTIL_H_
#define INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_GENERATOR_UTIL_H_

#include <cstddef>
#include <optional>

#include "absl/log/check.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

constexpr size_t kMaxXMLElementNesting = 512;
constexpr absl::string_view kSpecialXMLElementNameCharacters = "-_.";
constexpr absl::string_view kXMLQuoteCharacters = "\"'";

// Represents a string range inside of a larger string. This is useful when the
// surroundings of the string range are also needed or the position in the
// string is relevant.
// The string is stored internally only as a stringview. Therefore, the
// stringview must outlive the StringRange.
class StringRange {
 public:
  StringRange() = default;
  // Create a new StringRange spanning over the whole `str`. `str` must outlive
  // this StringRange.
  explicit StringRange(absl::string_view str)
      : StringRange(str, 0, str.size()) {}
  // Create a new StringRange starting at `pos` with length `size`. `str` must
  // outlive this StringRange.
  StringRange(absl::string_view str, size_t pos, size_t size)
      : str_(str), pos_(pos), size_(size) {}
  // Create a new StringRange starting at `pos` to the end of `str`. `str` must
  // outlive this StringRange.
  StringRange(absl::string_view str, size_t pos)
      : str_(str), pos_(pos), size_(str.size() - pos) {}
  // Constructs a StringRange from the existing `range`. `pos` is relative to
  // the `pos` of `range`.
  StringRange(const StringRange& range, size_t pos, size_t size)
      : str_(range.str_), pos_(range.pos_ + pos), size_(size) {}
  // Constructs a StringRange from the existing `range`. `pos` is relative to
  // the `pos` of `range`. The end of the new StringRange will be the end of
  // `range`.
  StringRange(const StringRange& range, size_t pos)
      : str_(range.str_), pos_(range.pos_ + pos), size_(range.size_ - pos) {}

  // Create a new StringRange with a new absolute start and end position using
  // the underlying stringview. The position `end` is not included in the
  // string.
  StringRange WithNewStartEnd(size_t start, size_t end) {
    QCHECK_LE(start, end);
    return StringRange(str_, start, end - start);
  }

  // Get the string represented by this StringRange.
  absl::string_view String() const { return str_.substr(pos_, size_); }

  // Create a sub-StringRange of this StringRange relative to the same
  // underlying stringview.
  // `pos` is relative to the position of the `this` StringRange.
  // If `size` is not given, the new StringRange will go to the end of `this`
  // StringRange.
  StringRange SubStr(size_t pos,
                     std::optional<size_t> size = std::nullopt) const {
    return StringRange(str_, this->pos_ + pos,
                       size.value_or(this->size_ - pos));
  }

  // Size of this StringRange().
  size_t Size() const { return size_; }

  // Position of this StringRange in the underlying string_view.
  size_t Position() const { return pos_; }

  // Returns the full source string view of `this` StringRange.
  absl::string_view SourceString() const { return str_; }

 private:
  absl::string_view str_;
  size_t pos_ = 0;
  size_t size_ = 0;
};

// Contains StringRanges for the 3 different parts of an XML element:
// - the full range of the XML element
// - only the range of the opening tag
// - the range of the text of an XML element (unparsed, i.e. including all
// sub-elements if any)
struct ElementScopes {
  StringRange full_scope;
  StringRange opening_scope;
  StringRange text;
};

// Returns a sub string around `center` with size `radius` in both directions.
// Truncates the string if center+-radius is out of bounds.
absl::string_view SafeSubStrAroundPosition(absl::string_view str, size_t center,
                                           size_t radius);

// Trims a string if longer than `Size`. In this case, appends 3 dots to signal
// that it was trimmed.
// Returns a copy.
template <size_t Size>
icon::FixedString<Size> GetLimitedString(absl::string_view string) {
  if (string.size() > Size) {
    return icon::FixedStrCat<Size>(string.substr(0, Size - 3), "...");
  }
  return icon::FixedString<Size>(string);
}

// Checks if the given string is a valid XML element name.
bool IsValidXMLElementName(absl::string_view element_name);

// Remainder overload for the variadic template below. Checks efficiently if
// `str` starts with `part`.
bool StartsWithConcatenation(absl::string_view str, bool ignore_case,
                             absl::string_view part);

// Checks efficiently if `str` starts with a concatenation of `parts`.
template <typename... Parts>
bool StartsWithConcatenation(absl::string_view str, bool ignore_case,
                             absl::string_view part, Parts... parts) {
  if (!StartsWithConcatenation(str, ignore_case, part)) {
    return false;
  }
  return StartsWithConcatenation(str.substr(part.size()), ignore_case,
                                 parts...);
}

// Find the next quoted string (with quotes types given with `quote_characters`)
// in `str`. Returns the string without the quotes.
icon::RealtimeStatusOr<StringRange> FindQuotedString(
    absl::string_view str, absl::string_view quote_characters);

// Finds the next full-scope of an XML element named `element_name`, e.g.
// `<other_element/><element>text or sub-elements</element>` on the top-level of
// the given XML string.
//
// Set `fill_not_found_message` to false to keep the
// notfound-error message empty for better performance.
//
// The returned scopes for the example above when `element_name=="elemenmt"`
// are:
//
// Full scope: <other_element/><element>text or sub-elements</element>
//                             ^─────────────────────────────────────^
// Opening tag: <other_element/><element>text or sub-elements</element>
//                              ^───────^
// Text: <other_element/><element>text or sub-elements</element>
//                                ^──────────────────^
icon::RealtimeStatusOr<ElementScopes> FindElementTagScope(
    absl::string_view str, absl::string_view element_name,
    bool fill_not_found_message);

// Copies `value` to `buffer` starting at `position`. `position` will be
// increased by `value.size()`. Remainder overload for variadic template.
icon::RealtimeStatus CopyToBuffer(absl::Span<char>& buffer, size_t& position,
                                  absl::string_view value);

// Copies `value` and `args` to `buffer` starting at `position`. `position` will
// be increased by the sum of the lengths of all copied strings. `args` need to
// be implicitly convertible to `absl::string_view`.
//
// Returns ResourceExhausted error if the given `buffer` is too small. There
// might be some part of the buffer already overwritten in this case, but
// `position` remains at the initial value.
template <typename... Args>
icon::RealtimeStatus CopyToBuffer(absl::Span<char>& buffer, size_t& position,
                                  absl::string_view value,
                                  const Args&... args) {
  size_t temp_position = position;
  INTRINSIC_RT_RETURN_IF_ERROR(CopyToBuffer(buffer, temp_position, value));
  icon::RealtimeStatus result = CopyToBuffer(buffer, temp_position, args...);
  if (result.ok()) {
    position = temp_position;
  }
  return result;
}

// Escapes a string for usage in an XML text element or XML attribute.
// Returns ResourceExhaustedError if `ResultSize` cannot fit the escaped string.
template <size_t ResultSize>
icon::RealtimeStatusOr<icon::FixedString<ResultSize>> EscapeStringForXML(
    absl::string_view str) INTRINSIC_CHECK_REALTIME_SAFE {
  icon::FixedString<ResultSize> escaped_string;
  auto check_size = [&escaped_string, str](size_t additional_size) {
    if (escaped_string.size() + additional_size > ResultSize) {
      return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
          "String of size ", ResultSize,
          " too small to escape string for XML: ", GetLimitedString<20>(str)));
    }
    return icon::OkStatus();
  };
  for (size_t i = 0; i < str.size(); ++i) {
    switch (str[i]) {
      case '&':
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(5));
        escaped_string.append("&amp;");
        break;
      case '"':
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(6));
        escaped_string.append("&quot;");
        break;
      case '\'':
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(6));
        escaped_string.append("&apos;");
        break;
      case '<':
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(4));
        escaped_string.append("&lt;");
        break;
      case '>':
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(4));
        escaped_string.append("&gt;");
        break;
      default:
        INTRINSIC_RT_RETURN_IF_ERROR(check_size(1));
        escaped_string.append(icon::SingleCharacterString(str[i]));
        break;
    }
  }
  return escaped_string;
}

}  // namespace intrinsic

#endif  // INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_GENERATOR_UTIL_H_

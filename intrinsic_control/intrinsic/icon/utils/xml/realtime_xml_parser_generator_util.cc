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

#include "intrinsic/icon/utils/xml/realtime_xml_parser_generator_util.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <optional>
#include <string_view>

#include "absl/status/status.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/invalid_until_set.h"

namespace intrinsic {

bool StartsWithConcatenation(absl::string_view str, bool ignore_case,
                             absl::string_view part) {
  if (ignore_case) {
    return absl::StartsWithIgnoreCase(str, part);
  } else {
    return absl::StartsWith(str, part);
  }
  return true;
}

absl::string_view SafeSubStrAroundPosition(absl::string_view str, size_t center,
                                           size_t radius) {
  // center = std::min(center, str.size());
  size_t left_pos = radius > center ? 0 : center - radius;
  left_pos = std::min(left_pos, str.size());
  size_t size =
      center + radius > str.size()
          ? absl::string_view::npos
          : center + radius - left_pos + 1;  // + 1 for center element.

  return str.substr(left_pos, size);
}

namespace {
// Checks if the given character `ch` is any of the special characters, i.e. not
// the alpha-numeric characters, that are allowed in an XML element name
// (hyphen, dot and underscore).
bool IsSpecialXMLElementNameCharacter(char ch) {
  return absl::StrContains(kSpecialXMLElementNameCharacters, ch);
}

// Checks if the given character `ch` is any of the characters that are allowed
// in an XML element name (Alpha-numeric characters and hyphen, dot and
// underscore).
bool IsElementNameBodyCharacter(char ch) {
  return IsSpecialXMLElementNameCharacter(ch) || absl::ascii_isalnum(ch);
}

// Returns the size of the closing tag or error if the closing tag was not
// found.
icon::RealtimeStatusOr<size_t> StartsWithElementClosing(
    absl::string_view str, absl::string_view element_name) {
  if (StartsWithConcatenation(str, false, "</", element_name)) {
    // White spaces after </element_name and before > are allowed.
    for (size_t i = 2 + element_name.size(); i < str.size(); ++i) {
      if (str.at(i) == '>') {
        return i + 1;
      }
      if (!absl::ascii_isspace(str.at(i))) {
        return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
            "Invalid characters found in closing tag: ",
            SafeSubStrAroundPosition(str, i, 10)));
      }
    }
  }
  return icon::NotFoundError("");
}

// Parse the XML element name starting at the current position. Will stop at
// first whitespace or when an XML tag closes. Checks the returned value to be a
// valid XML element name.
icon::RealtimeStatusOr<absl::string_view> ParseElementName(
    absl::string_view str) {
  if (str.empty()) {
    return icon::InvalidArgumentError(
        "Empty string is not a valid XML element name.");
  }
  if (str.front() != '_' && !absl::ascii_isalpha(str.front())) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "First character of string '", GetLimitedString<20>(str),
        "' is not a valid XML element start character."));
  }
  size_t i;
  for (i = 0; i < str.size(); i++) {
    char c = str.at(i);
    if (IsElementNameBodyCharacter(c)) {
      continue;
    }
    if (absl::ascii_isspace(c) || c == '/' || c == '>') {
      break;
    }
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Found invalid XML element character '", icon::SingleCharacterString(c),
        "' in string '", SafeSubStrAroundPosition(str, i, 7), "'."));
  }
  auto result = str.substr(0, i);
  if (StartsWithConcatenation(result, true, "xml")) {
    return icon::InvalidArgumentError(
        "An XML element name must not start with any form of the string "
        "'xml'.");
  }
  return result;
}

// Finds the next occurrence of `>` and returns a StringRange from the
// beginning of `str` up to and including this `>`.
icon::RealtimeStatusOr<StringRange> FindClosingAngleBracket(
    absl::string_view str) {
  // Escaping of > with backslash is not allowed according to XML spec.
  constexpr char closing_brace = '>';
  const size_t pos = str.find_first_of(closing_brace);
  if (pos == absl::string_view::npos) {
    return icon::NotFoundError("Could not find closing brace.");
  }
  return StringRange{str, 0, pos + 1};
}

icon::RealtimeStatusOr<ElementScopes> FindElementTagScopeRecursive(
    StringRange str, absl::string_view element_name,
    bool fill_not_found_message, size_t recursion_depth);

// Attempts to find the XML closing tag (e.g. `</root>`) for an XML element
// (e.g. `<root>text</root>`) called `element_name` starting at the beginning of
// `str`. The function goes character by character through the string:
// - If a closing tag starts at the current position, it's the requested closing
// tag.
// - If a new tag starts at the current position, the scope of that element
// (i.e. start and end position of a string like `<root>text</root>`) will be
// searched and the current position will be incremented by the size of this
// element.
//
// For example for a StringRange `str` like the following with
// `element_name=="element"`:
//
// <root><element><subelement attribute="some text"/></element></root>
//                ^───────────────────────────────────────────^
// The returned StringRange will be as shown below:
//
// <root><element attribute="some text">text</element></root>
//                                          ^────────^
// It's important to note that `str` needs to start directly after the opening
// tag to correctly process sub-elements.
icon::RealtimeStatusOr<StringRange> FindElementClosingTag(
    StringRange str, absl::string_view element_name, size_t recursion_depth) {
  for (size_t i = 0; i < str.Size(); ++i) {
    StringRange current_remaining_str(str, i);
    icon::RealtimeStatusOr<size_t> size =
        StartsWithElementClosing(current_remaining_str.String(), element_name);
    if (size.ok()) {
      return StringRange(str, i, *size);
    }
    if (size.status().code() != absl::StatusCode::kNotFound) {
      return size.status();
    }
    if (current_remaining_str.String().at(0) == '<') {
      // A new XML element is starting if we reach this. So, get the full scope
      // of it.
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto found_element_name,
          ParseElementName(current_remaining_str.String().substr(1)));
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto scope, FindElementTagScopeRecursive(
                          current_remaining_str, found_element_name,
                          /*fill_not_found_message=*/false, recursion_depth));
      // Skip the element we just found since the closing tag will come
      // afterwards.
      i += scope.full_scope.Size() - 1;
    }
  }
  return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
      "Could not find element closing tag for '", element_name, "'."));
}

// Attempts to find an element scope by iterating through the string. If an XML
// element with a different name is found, the scope of this element is searched
// recursively and skipped for the currently requested element.
icon::RealtimeStatusOr<ElementScopes> FindElementTagScopeRecursive(
    StringRange str, absl::string_view element_name,
    bool fill_not_found_message, size_t recursion_depth) {
  if (recursion_depth > kMaxXMLElementNesting) {
    return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
        "XML parser recursion limit reached: ", kMaxXMLElementNesting));
  }
  for (size_t i = 0; i < str.Size(); ++i) {
    StringRange current_remaining_range(str, i);
    // If there is a `<` in the string, it is an XML tag. No backslash escaping
    // is allowed in XML. If not, nothing of interest starts at this character.
    if (current_remaining_range.String().at(0) != '<') {
      continue;
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto found_element_name,
        ParseElementName(current_remaining_range.String().substr(1)));
    // If the element has the requested name, we look for the closing bracket
    // and then the closing tag.
    if (found_element_name == element_name) {
      const size_t tag_opening_size = found_element_name.size() + 1;
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          StringRange element_opening_range,
          FindClosingAngleBracket(
              current_remaining_range.String().substr(tag_opening_size)));
      const bool sub_element_directly_closed =
          absl::EndsWith(element_opening_range.String(), "/>");
      // If the element was directly closed again, we have all the data we
      // need to describe it.
      if (sub_element_directly_closed) {
        StringRange opening_scope(
            current_remaining_range, 0,
            tag_opening_size + element_opening_range.Size());
        return ElementScopes{
            .full_scope = opening_scope,
            .opening_scope = opening_scope,
            .text = StringRange(current_remaining_range,
                                tag_opening_size + element_opening_range.Size(),
                                0)};
      } else {  // Otherwise, we need to find the closing tag.
        auto element_remainder = current_remaining_range.SubStr(
            tag_opening_size + element_opening_range.Size());
        INTRINSIC_RT_ASSIGN_OR_RETURN(
            auto closing_tag_range,
            FindElementClosingTag(element_remainder, element_name,
                                  recursion_depth + 1));
        return ElementScopes{
            .full_scope = current_remaining_range.WithNewStartEnd(
                current_remaining_range.Position(),
                closing_tag_range.Position() + closing_tag_range.Size()),
            .opening_scope =
                StringRange(current_remaining_range, 0,
                            tag_opening_size + element_opening_range.Size()),
            .text = current_remaining_range.WithNewStartEnd(
                element_remainder.Position(), closing_tag_range.Position())};
      }
    } else {
      // Otherwise, we do a recursive descent with the found element name. We
      // need to find the full scope of this unrelated element. Since
      // otherwise we could find an element with the requested name as a
      // sub-element of this element.
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto scope, FindElementTagScopeRecursive(current_remaining_range,
                                                   found_element_name, false,
                                                   recursion_depth + 1));
      // Skip the element we just found to not confuse elements with the same
      // name but on a deeper level with the requested element.
      i += scope.full_scope.Size() - 1;
    }
  }

  if (fill_not_found_message) {
    return icon::NotFoundError(icon::RealtimeStatus::StrCat(
        "Could not find element scope for '", element_name, "'."));
  }
  return icon::NotFoundError("");
}

}  // namespace

bool IsValidXMLElementName(absl::string_view element_name) {
  if (element_name.empty()) {
    return false;
  }
  if (element_name.front() != '_' &&
      !absl::ascii_isalpha(element_name.front())) {
    return false;
  }
  for (size_t i = 1; i < element_name.size(); ++i) {
    const char ch = element_name.at(i);
    if (!IsElementNameBodyCharacter(ch)) {
      return false;
    }
  }
  if (StartsWithConcatenation(element_name, true, "xml")) {
    return false;
  }
  return true;
}

icon::RealtimeStatusOr<StringRange> FindQuotedString(
    absl::string_view str, absl::string_view quote_characters) {
  InvalidUntilSet<size_t> string_start_pos;
  InvalidUntilSet<size_t> string_end_pos;
  InvalidUntilSet<char> found_quote_type;
  for (size_t i = 0; i < str.size(); ++i) {
    const char character = str.at(i);
    if (!string_start_pos.has_value() &&
        absl::StrContains(quote_characters, character)) {
      string_start_pos = i + 1;
      found_quote_type = character;
    } else if (character == found_quote_type) {
      string_end_pos = i;
      break;
    }
  }

  if (!string_start_pos.has_value() || string_start_pos >= str.size()) {
    return icon::NotFoundError(icon::RealtimeStatus::StrCat(
        "Could not find quoted string in '", GetLimitedString<30>(str), "'."));
  }
  if (!string_end_pos.has_value()) {
    return icon::NotFoundError(icon::RealtimeStatus::StrCat(
        "Could not find second quote of type ",
        icon::SingleCharacterString(*found_quote_type), " in '",
        GetLimitedString<30>(str), "'."));
  }
  return StringRange{str, *string_start_pos,
                     *string_end_pos - *string_start_pos};
}

icon::RealtimeStatusOr<ElementScopes> FindElementTagScope(
    absl::string_view str, absl::string_view element_name,
    bool fill_not_found_message) {
  return FindElementTagScopeRecursive(StringRange(str, 0, str.size()),
                                      element_name, fill_not_found_message,
                                      /*recursion_depth=*/1);
}

icon::RealtimeStatus CopyToBuffer(absl::Span<char>& buffer, size_t& position,
                                  absl::string_view value) {
  if (position + value.size() > buffer.size()) {
    return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
        "String buffer exhausted (Capacity: ", buffer.size(),
        "). Starts with: ",
        GetLimitedString<30>(absl::string_view(buffer.data(), buffer.size()))));
  }
  (void)std::memcpy(buffer.data() + position, value.data(), value.size());
  position += value.size();
  return icon::OkStatus();
}

}  // namespace intrinsic

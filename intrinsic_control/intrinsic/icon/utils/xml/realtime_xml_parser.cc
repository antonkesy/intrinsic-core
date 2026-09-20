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

#include "intrinsic/icon/utils/xml/realtime_xml_parser.h"

#include <cstddef>

#include "absl/base/optimization.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser_generator_util.h"

namespace intrinsic {

icon::RealtimeStatusOr<RealtimeXmlParserElement>
RealtimeXmlParserElement::ParseDoc(absl::string_view xml_string,
                                   absl::string_view root_element_name) {
  absl::string_view trimmed_xml_string = absl::StripAsciiWhitespace(xml_string);
  constexpr absl::string_view kXmlDeclarationTag = "<?xml";
  if (absl::StartsWith(trimmed_xml_string, kXmlDeclarationTag)) {
    bool found_end = false;
    for (size_t i = kXmlDeclarationTag.size();  // skip <?xml
         i < trimmed_xml_string.size(); ++i) {
      auto current_substr = trimmed_xml_string.substr(i);
      if (absl::StartsWith(current_substr, "?>")) {
        trimmed_xml_string = current_substr.substr(2);
        found_end = true;
        break;
      }
      if (current_substr[0] == '<' || current_substr[0] == '>') {
        return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
            "Found `<?xml` in XML string, but no corresponding `?>`. XML: ",
            GetLimitedString<30>(trimmed_xml_string)));
      }
    }
    if (!found_end) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "Found `<?xml` in XML string, but no corresponding `?>`. XML: ",
          GetLimitedString<30>(trimmed_xml_string)));
    }
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto element_scopes,
      FindElementTagScope(trimmed_xml_string, root_element_name, true));
  return RealtimeXmlParserElement(element_scopes, root_element_name);
}

icon::RealtimeStatusOr<RealtimeXmlParserElement>
RealtimeXmlParserElement::FirstElement(
    absl::string_view element_name, bool fill_not_found_error_message) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto elements, FirstNElements<1>(element_name));
  if (elements.empty()) {
    if (fill_not_found_error_message) {
      return icon::NotFoundError(icon::RealtimeStatus::StrCat(
          "Could not find an element named '", element_name, "'"));
    }
    return icon::NotFoundError("");
  }
  return elements[0];
}

icon::RealtimeStatusOr<absl::string_view> RealtimeXmlParserElement::Attribute(
    absl::string_view attribute_name, bool fill_not_found_error_message) const {
  if (!IsValidXMLElementName(attribute_name)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Not a valid XML attribute name: ", attribute_name));
  }
  auto str = element_scopes_.opening_scope.String();
  for (size_t i = 0; i < str.size(); ++i) {
    if (ABSL_PREDICT_FALSE(absl::StrContains(kXMLQuoteCharacters, str.at(i)))) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          auto quoted_string_range,
          FindQuotedString(str.substr(i), kXMLQuoteCharacters));
      i +=
          quoted_string_range.Size() + 1;  // +1 for the closing quote character
      continue;
    }
    if (absl::ascii_isspace(
            str.at(i))  // Before the attribute starts there must be a white
                        // space, otherwise an attribute with just a different
                        // prefix might be confused with the target attribute.
        &&
        StartsWithConcatenation(str.substr(i + 1), true, attribute_name, "=")) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          StringRange attribute_value_range,
          FindQuotedString(str.substr(i + 1 + attribute_name.size()),
                           kXMLQuoteCharacters));
      return attribute_value_range.String();
    }
  }
  if (fill_not_found_error_message) {
    return icon::NotFoundError(icon ::RealtimeStatus::StrCat(
        "Could not find attribute '", attribute_name, "' in '", element_name_,
        "'"));
  }
  return icon::NotFoundError("");
}

icon::RealtimeStatusOr<RealtimeXmlParserElement>
RealtimeXmlParserElement::NextElement(absl::string_view element_name,
                                      size_t& offset,
                                      bool fill_not_found_error_message) const {
  if (offset >= element_scopes_.text.Size()) {
    if (fill_not_found_error_message) {
      return icon::NotFoundError(icon::RealtimeStatus::StrCat(
          "Could not find an element named '", element_name, "'"));
    }
    return icon::NotFoundError("");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      ElementScopes element_scopes,
      FindElementTagScope(element_scopes_.text.String().substr(offset),
                          element_name, false));
  offset +=
      element_scopes.full_scope.Position() + element_scopes.full_scope.Size();
  return RealtimeXmlParserElement(element_scopes, element_name);
}

icon::RealtimeStatusOr<double> RealtimeXmlParserElement::TextAsDouble() const {
  double value = 0;
  if (!absl::SimpleAtod(element_scopes_.text.String(), &value)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Could not convert text to double: '",
        GetLimitedString<25>(element_scopes_.text.String()), "' from '",
        element_name_, "'"));
  }
  return value;
}

icon::RealtimeStatusOr<double> RealtimeXmlParserElement::AttributeAsDouble(
    absl::string_view attribute_name, bool fill_not_found_error_message) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto attribute_value_string,
      Attribute(attribute_name, fill_not_found_error_message));
  double value = 0;
  if (!absl::SimpleAtod(attribute_value_string, &value)) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Could not convert attribute to double: '",
        GetLimitedString<25>(attribute_value_string), "' for attribute '",
        attribute_name, "'"));
  }
  return value;
}

}  // namespace intrinsic

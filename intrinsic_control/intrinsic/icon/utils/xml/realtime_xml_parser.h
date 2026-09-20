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

#ifndef INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_H_
#define INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_H_

#include <sys/types.h>

#include <cstddef>
#include <type_traits>

#include "absl/status/status.h"
#include "absl/strings/numbers.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser_generator_util.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Parses a string of XML in a real-time safe manner.
//
// Limitations:
// - The XML error handling is very limited. For example an
//   unescaped < or > might lead to misleading error messages (note: correct
//   escaping of `<` is `&lt;` not with a backslash).
// - No XML comments supported. This means the XML string must not contain any
//   XML comments.
// - `>` must be escaped (as `&gt;`) if not part of an xml-tag definition.
//
//
// Time complexity of each class method call is up to O(n), where n is the
// string size of the current element.
// Space complexity is O(1).
//
// Note: The RT XML parser and generator do not round-trip perfectly in all
// cases.
//
// Example usage:
//   INTRINSIC_RT_ASSIGN_OR_RETURN(const RealtimeXmlParserElement root_element,
//                            RealtimeXmlParserElement::ParseDoc(buffer,
//                            "Rob"));
//   INTRINSIC_RT_ASSIGN_OR_RETURN(auto position_element,
//                            root_element.FirstElement("Pos"));
//   INTRINSIC_RT_ASSIGN_OR_RETURN(
//       double position,
//       position_element.AttributeAsDouble("Joint1"));
//
class RealtimeXmlParserElement {
 public:
  RealtimeXmlParserElement() = default;
  RealtimeXmlParserElement(const RealtimeXmlParserElement& other) = default;
  RealtimeXmlParserElement(RealtimeXmlParserElement&& other) = default;

  // Function to parse the root element from an XML string.
  //
  // Returns NotFoundError if there is not root element or if the name is
  // different.
  //
  // Note: `xml_string` must outlive any `RealtimeXmlParserElement`
  // or `string_view` that are returned by any of the methods of
  // `RealtimeXmlParserElement`.
  static icon::RealtimeStatusOr<RealtimeXmlParserElement> ParseDoc(
      absl::string_view xml_string,
      absl::string_view root_element_name) INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns the full XML string of this element including the opening tag, the
  // text field and the closing tag. Given an XML document
  // `<root><element>text</element></root>`, if this element is named `element`
  // this function returns `<element>text</element>`.
  absl::string_view XmlString() const INTRINSIC_CHECK_REALTIME_SAFE {
    return element_scopes_.full_scope.String();
  }

  // Returns the text field value. This includes the XML representation of any
  // sub-elements.
  // Given an XML document `<root><element>text</element></root>`, if this
  // element is named `element` this function returns `text`
  absl::string_view Text() const INTRINSIC_CHECK_REALTIME_SAFE {
    return element_scopes_.text.String();
  }

  // Queries the text field value and attempts to convert it to `IntegerType`.
  // Otherwise returns InvalidArgumentError.
  template <typename IntegerType>
  icon::RealtimeStatusOr<IntegerType> TextAsInteger() const
      INTRINSIC_CHECK_REALTIME_SAFE {
    static_assert(std::is_integral<IntegerType>::value,
                  "The given type must be an integer type");
    IntegerType value = 0;
    if (!absl::SimpleAtoi(element_scopes_.text.String(), &value)) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "Could not convert text to integer: '",
          GetLimitedString<25>(element_scopes_.text.String()), "' from '",
          element_name_, "'"));
    }
    return value;
  }

  // Queries the text field value and attempts to convert it to double.
  // Otherwise returns InvalidArgumentError.
  icon::RealtimeStatusOr<double> TextAsDouble() const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Queries the first `N` subelements of the XML element with name
  // `element_name`. If less than `N` elements are found, only those are
  // returned. If none are found, returns NotFoundError.
  //
  // Can return various errors from parsing.
  template <size_t N>
  icon::RealtimeStatusOr<FixedVector<RealtimeXmlParserElement, N>>
  FirstNElements(absl::string_view element_name,
                 bool fill_not_found_error_message = true) const
      INTRINSIC_CHECK_REALTIME_SAFE {
    static_assert(sizeof(RealtimeXmlParserElement) * N < 1024 * 100,
                  "FirstNElements allocates on the stack and thus is "
                  "limited to 100KB worth of memory");
    FixedVector<RealtimeXmlParserElement, N> elements;
    size_t offset = 0;
    for (size_t i = 0; i < N; ++i) {
      icon::RealtimeStatusOr<RealtimeXmlParserElement> element =
          NextElement(element_name, offset);
      if (element.status().code() == absl::StatusCode::kNotFound) {
        if (elements.empty()) {
          if (fill_not_found_error_message) {
            return icon::NotFoundError(icon::RealtimeStatus::StrCat(
                "Could not find any element with name '", element_name,
                "' in '", GetLimitedString<25>(element_scopes_.text.String()),
                "'"));
          }
          return icon::NotFoundError("");
        } else {
          break;
        }
      } else if (element.ok()) {
        elements.push_back(element.value());
      } else {
        return element.status();
      }
    }
    return elements;
  }

  // Queries the first element named `element_name`.
  //
  // Returns NotFoundError if the element does not exist. Can return various
  // errors from parsing.
  icon::RealtimeStatusOr<RealtimeXmlParserElement> FirstElement(
      absl::string_view element_name, bool fill_not_found_error_message = true)
      const INTRINSIC_CHECK_REALTIME_SAFE;

  // Queries next element starting in the XML string of this XML element from
  // position `offset` with name `element_name`.
  //
  // Returns the element and increments `offset` to the position behind the
  // returned element.
  //
  // Returns NotFound error if no element with name `element_name` was found
  // behind `offset`.
  // Usage:
  //   size_t offset = 0;
  //   while (true) {
  //     auto element = root_element.NextElement("element", offset);
  //     if (element.status().code() == absl::StatusCode::kNotFound) {
  //       break;
  //     }
  //     do stuff...
  //   }
  //
  // Returns various errors from parsing.
  icon::RealtimeStatusOr<RealtimeXmlParserElement> NextElement(
      absl::string_view element_name, size_t& offset,
      bool fill_not_found_error_message = true) const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Queries the attribute `attribute_name` from this XML element. Returns
  // NotFoundError if the attribute does not exist.
  // Set `fill_not_found_error_message` to false for better performance
  // to return an empty NotFoundError without any error string.
  //
  // Can return various errors from parsing.
  icon::RealtimeStatusOr<absl::string_view> Attribute(
      absl::string_view attribute_name,
      bool fill_not_found_error_message = true) const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Queries the attribute `attribute_name` from this XML element and attempts
  // to convert it to `IntegerType`. `fill_not_found_error_message` can be set
  // to false for better performance to return an empty NotFoundError without
  // any error string.
  //
  // Returns NotFoundError, if the attribute does not exist.
  // Returns InvalidArgumentError if the conversion fails.
  // Can return various errors from parsing.
  template <typename IntegerType>
  icon::RealtimeStatusOr<IntegerType> AttributeAsInteger(
      absl::string_view attribute_name,
      bool fill_not_found_error_message = true) const
      INTRINSIC_CHECK_REALTIME_SAFE {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto attribute_value_string,
        Attribute(attribute_name, fill_not_found_error_message));
    IntegerType value = 0;
    if (!absl::SimpleAtoi(attribute_value_string, &value)) {
      return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
          "Could not convert attribute to integer: '",
          GetLimitedString<25>(attribute_value_string), "' for attribute '",
          attribute_name, "'"));
    }
    return value;
  }

  // Queries the attribute `attribute_name` from this XML element and attempts
  // to convert it to double. `fill_not_found_error_message` can be set
  // to false for better performance to return an empty NotFoundError without
  // any error string.
  //
  // Returns NotFoundError, if the attribute does not exist.
  // Returns InvalidArgumentError if the conversion fails.
  // Can return various errors from parsing.
  icon::RealtimeStatusOr<double> AttributeAsDouble(
      absl::string_view attribute_name,
      bool fill_not_found_error_message = true) const
      INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  explicit RealtimeXmlParserElement(const ElementScopes& element_scopes,
                                    absl::string_view element_name)
      : element_scopes_(element_scopes), element_name_(element_name) {}
  ElementScopes element_scopes_;
  absl::string_view element_name_;
};
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_UTILS_XML_REALTIME_XML_PARSER_H_

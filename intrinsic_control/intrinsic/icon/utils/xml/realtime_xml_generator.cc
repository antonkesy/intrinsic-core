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

#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <memory>
#include <variant>

#include "absl/algorithm/container.h"
#include "absl/container/fixed_array.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser_generator_util.h"

namespace intrinsic {

namespace {
class VariantToStringVisitor {
 public:
  absl::string_view operator()(absl::string_view string_view) {
    return string_view;
  }

  absl::string_view operator()(
      const icon::FixedString<kFloatingPointNumberStringSize>& string) {
    return string;
  }
};

}  // namespace

icon::RealtimeStatusOr<RtXmlElement*> RtXmlElement::AddElement(
    absl::string_view name) {
  if (!IsValidXMLElementName(name)) {
    return icon::InvalidArgumentError(
        icon::RealtimeStatus::StrCat("Invalid element name '", name, "'"));
  }
  if (generator_->current_elements_span_.size() + 1 >
      generator_->elements_.size()) {
    return icon::ResourceExhaustedError(
        icon::RealtimeStatus::StrCat("XML element vector exhausted. Capacity: ",
                                     generator_->elements_.size()));
  }
  std::unique_ptr<RtXmlElement>& new_element =
      generator_->elements_.at(generator_->current_elements_span_.size());
  *new_element = RtXmlElement(generator_, name, this);
  generator_->current_elements_span_ =
      absl::MakeSpan(generator_->elements_.data(),
                     generator_->current_elements_span_.size() + 1);
  return new_element.get();
}

icon::RealtimeStatusOr<RtXmlAttribute*> RtXmlElement::AddAttribute(
    absl::string_view name, bool check_for_duplicate) {
  if (!IsValidXMLElementName(name)) {
    return icon::InvalidArgumentError(
        icon::RealtimeStatus::StrCat("Invalid attribute name '", name, "'"));
  }
  if (check_for_duplicate) {
    for (const auto& attribute : generator_->current_attributes_span_) {
      if (attribute->Parent() == this && attribute->name_ == name) {
        return icon::AlreadyExistsError(
            icon::RealtimeStatus::StrCat("Duplicate attribute name '", name,
                                         "' in element '", this->name_, "'"));
      }
    }
  }
  if (generator_->current_attributes_span_.size() >=
      generator_->attributes_.size()) {
    return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
        "XML attributes vector exhausted. Capacity: ",
        generator_->attributes_.size()));
  }
  std::unique_ptr<RtXmlAttribute>& new_attribute =
      generator_->attributes_.at(generator_->current_attributes_span_.size());
  *new_attribute = RtXmlAttribute(this, name);
  generator_->current_attributes_span_ =
      absl::MakeSpan(generator_->attributes_.data(),
                     generator_->current_attributes_span_.size() + 1);
  return new_attribute.get();
}

absl::string_view RtXmlAttribute::ValueToString()
    INTRINSIC_CHECK_REALTIME_SAFE {
  return std::visit(VariantToStringVisitor(), string_);
}

size_t RtXmlAttribute::NextId() {
  static std::atomic_uint64_t next_id = 1;
  return next_id++;
}

absl::string_view RtXmlElement::Text() const INTRINSIC_CHECK_REALTIME_SAFE {
  return std::visit(VariantToStringVisitor(), text_);
}

size_t RtXmlElement::NextId() {
  static std::atomic_uint64_t next_id = 1;
  return next_id++;
}

RealtimeXmlGenerator::RealtimeXmlGenerator(size_t max_element_count,
                                           size_t max_attribute_count)
    INTRINSIC_NON_REALTIME_ONLY : elements_(max_element_count),
                                  attributes_(max_attribute_count) {
  absl::c_generate(elements_, [] { return std::make_unique<RtXmlElement>(); });
  absl::c_generate(attributes_,
                   [] { return std::make_unique<RtXmlAttribute>(); });
}

void RealtimeXmlGenerator::SortContainers() {
  auto comp = [](const auto& a, const auto& b) {
    if (a->parent_ != b->parent_) return a->parent_ < b->parent_;
    // Now check the secondary sort criteria.
    return a->unique_id_ < b->unique_id_;
  };
  absl::c_sort(current_elements_span_, comp);

  absl::c_sort(current_attributes_span_, comp);
}

namespace {

template <typename T>
auto FindFirstChildInSortedVector(T& container, const RtXmlElement* parent) {
  // Returns the first element, i.e. most-left in the sorted container, where
  // element.parent is >= parent.
  auto it = std::lower_bound(
      container.begin(), container.end(), parent,
      [](const auto& a, const RtXmlElement* b) { return a->Parent() < b; });
  // It is possible that the search returns an element with a parent that
  // is not the requested parent (i.e. greater than the requested parent), if no
  // element has the requested parent. In this case return end of list.
  if (it != container.end() && (*it)->Parent() != parent) {
    return container.end();
  }
  return it;
}

}  // namespace

absl::Span<std::unique_ptr<RtXmlAttribute>>::const_iterator
RealtimeXmlGenerator::FindFirstAttribute(const RtXmlElement* parent) const
    INTRINSIC_CHECK_REALTIME_SAFE {
  return FindFirstChildInSortedVector(current_attributes_span_, parent);
}

absl::Span<std::unique_ptr<RtXmlElement>>::const_iterator
RealtimeXmlGenerator::FindFirstChildElement(const RtXmlElement* parent) const
    INTRINSIC_CHECK_REALTIME_SAFE {
  return FindFirstChildInSortedVector(current_elements_span_, parent);
}

icon::RealtimeStatusOr<size_t> RealtimeXmlGenerator::GenerateXMLString(
    absl::Span<char> buffer) INTRINSIC_CHECK_REALTIME_SAFE {
  SortContainers();
  auto root_it = FindFirstChildElement(/*parent=*/nullptr);
  if (root_it == current_elements_span_.end()) {
    return icon::NotFoundError("Could not find root element");
  }
  size_t size = 0;
  INTRINSIC_RT_RETURN_IF_ERROR(GenerateElements(buffer, size, root_it->get()));
  return size;
}

icon::RealtimeStatusOr<RtXmlElement*> RealtimeXmlGenerator::AddRootElement(
    absl::string_view name) INTRINSIC_CHECK_REALTIME_SAFE {
  if (!current_elements_span_.empty()) {
    icon::RealtimeStatusOr<RtXmlElement*> element = RootElement();
    if (!element.ok()) {
      return icon::InternalError(
          "Generator has elements, but none of them is the root. This is a "
          "bug.");
    }
    return icon::AlreadyExistsError(icon::RealtimeStatus::StrCat(
        "Already got a root element (name: '", (*element)->name_, "')"));
  }
  if (elements_.empty()) {
    return icon::ResourceExhaustedError(
        "The XML element vector has size zero. Cannot add the root element.");
  }
  *elements_.at(current_elements_span_.size()) =
      RtXmlElement(this, name, nullptr);
  current_elements_span_ =
      absl::MakeSpan(elements_.data(), current_elements_span_.size() + 1);
  return current_elements_span_.rbegin()->get();
}

icon::RealtimeStatusOr<RtXmlElement*> RealtimeXmlGenerator::RootElement() const
    INTRINSIC_CHECK_REALTIME_SAFE {
  auto it = FindFirstChildElement(nullptr);
  if (it == current_elements_span_.end()) {
    return icon::NotFoundError("Could not find root element");
  }
  return it->get();
}

icon::RealtimeStatus RealtimeXmlGenerator::GenerateElements(
    absl::Span<char> buffer, size_t& position,
    const RtXmlElement* element) INTRINSIC_CHECK_REALTIME_SAFE {
  auto child_it = FindFirstChildElement(element);
  const bool has_child_elements = child_it != current_elements_span_.end();
  const bool has_text_value = !element->Text().empty();
  size_t new_position = position;
  INTRINSIC_RT_RETURN_IF_ERROR(
      CopyToBuffer(buffer, new_position, "<", element->name_));
  INTRINSIC_RT_RETURN_IF_ERROR(
      GenerateAttributes(buffer, new_position, element));
  if (!has_child_elements && !has_text_value) {
    // We neither have sub-elements nor a text value. So the node must be
    // directly closed again with the opening tag.
    INTRINSIC_RT_RETURN_IF_ERROR(CopyToBuffer(buffer, new_position, "/>"));

  } else {
    // We have some text and/or sub-elements.
    INTRINSIC_RT_RETURN_IF_ERROR(CopyToBuffer(buffer, new_position, ">"));

    // The elements vector is sorted by parent, so we can just iterate until the
    // parent is different.
    while (child_it != elements_.end()) {
      if (*child_it == nullptr) {
        return icon::InternalError(
            "An element in the XML element buffer is null. This should not "
            "happen.");
      }
      if ((*child_it)->parent_ != element) {
        break;
      }
      INTRINSIC_RT_RETURN_IF_ERROR(
          GenerateElements(buffer, new_position, child_it->get()));
      child_it++;
    }

    if (!element->Text().empty()) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          CopyToBuffer(buffer, new_position, element->Text()));
    }

    // Close the XML element with a closing tag.
    INTRINSIC_RT_RETURN_IF_ERROR(
        CopyToBuffer(buffer, new_position, "</", element->name_, ">"));
  }
  position = new_position;
  return icon::OkStatus();
}

icon::RealtimeStatus RealtimeXmlGenerator::GenerateAttributes(
    absl::Span<char> buffer, size_t& position, const RtXmlElement* parent) {
  size_t new_position = position;
  auto it = FindFirstAttribute(parent);
  while (it != current_attributes_span_.end() && (*it) != nullptr &&
         (*it)->parent_ == parent) {
    INTRINSIC_RT_RETURN_IF_ERROR(CopyToBuffer(buffer, new_position, " ",
                                              (*it)->name_, "=\"",
                                              (*it)->ValueToString(), "\""));
    it++;
  }
  position = new_position;
  return icon::OkStatus();
}

void RealtimeXmlGenerator::Clear() INTRINSIC_CHECK_REALTIME_SAFE {
  for (auto& element : current_elements_span_) {
    *element = RtXmlElement();
  }
  current_elements_span_ = {};

  for (auto& attribute : current_attributes_span_) {
    *attribute = RtXmlAttribute();
  }
  current_attributes_span_ = {};
}

icon::RealtimeStatus RtXmlAttribute::SetValue(absl::string_view text) {
  string_ = text;
  return icon::OkStatus();
}

}  // namespace intrinsic

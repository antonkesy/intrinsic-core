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

#ifndef INTRINSIC_PERCEPTION_CORE_IMAGE_H_
#define INTRINSIC_PERCEPTION_CORE_IMAGE_H_

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/perception/core/coordinate.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

/**
 * @brief Our basic image type, intended for everything from fast embedded
 * processing on up. This implementation strives for explicit, obvious behavior
 * when possible.
 *
 * An image is a regular type, thus has value semantics. The image is the data
 * owner (allocating and managing its memory internally). It provides direct
 * data access to support fast implementations.
 *
 * Images are expected to be used with a pixel type (or, in rare cases, with a
 * structure type). See image_traits.h for details.
 */
template <typename MyImageTrait>
class Image {
 public:
  using ImageTrait = MyImageTrait;
  using PixelType = typename ImageTrait::PixelType;
  using ScalarType = typename ImageTrait::ScalarType;
  static constexpr uint8_t kNumChannels = ImageTrait::kNumChannels;

  static_assert(ImageTrait::kPaddingBytes == 0,
                "padding bytes not supported (yet)");
  static_assert(sizeof(PixelType) == kNumChannels * sizeof(ScalarType),
                "Pixel data must be packed");
  static_assert(std::is_standard_layout<PixelType>::value,
                "PixelType must have standard layout");
  static_assert(std::is_default_constructible<PixelType>::value,
                "PixelType must be default constructible");
  static_assert(std::is_trivially_destructible<PixelType>::value,
                "PixelType must be trivially destructible");

  // "All pointer types are also valid random-access iterators."
  // http://www.cplusplus.com/reference/iterator/RandomAccessIterator/
  using iterator = PixelType*;
  using const_iterator = const PixelType*;

  // Construct an empty image.
  Image() : dimensions_(), owning_ptr_(nullptr) {}

  // Construct an (cols x rows) image, and default constructs Pixels.
  explicit Image(Dimensions dimensions) : dimensions_(dimensions) {
    if (dimensions_.area() > 0) {
      owning_ptr_ = OwningPointer(new PixelType[dimensions_.area()],
                                  std::default_delete<PixelType[]>{});
    }
  }

  // Construct an (cols x rows) image, and default constructs Pixels.
  Image(int32_t cols, int32_t rows)
      : Image<ImageTrait>(Dimensions(cols, rows)) {}

  // Construct an (cols x rows) image, where all pixels are initialized
  // to a common value.
  Image(Dimensions dimensions, const PixelType& val)
      : Image<ImageTrait>(dimensions) {
    std::fill(begin(), end(), val);
  }

  // Construct an (cols x rows) image, where all pixels are initialized
  // to a common value.
  Image(int32_t cols, int32_t rows, const PixelType& val)
      : Image<ImageTrait>(Dimensions(cols, rows), val) {}

  // Construct an image from a pointer and a custom deleter.
  Image(Dimensions dimensions, PixelType* data,
        std::function<void(PixelType*)> deleter)
      : dimensions_(dimensions), owning_ptr_(data, std::move(deleter)) {
    ABSL_CHECK(dimensions.area() > 0) << "Dimensions must positive.";
  }

  // Creates an image by copying pixel data.
  Image(Dimensions dimensions, const PixelType* data)
      : Image<ImageTrait>(dimensions) {
    std::copy(data, data + dimensions.area(), begin());
  }

  // Creates an image by copying pixel data from an absl::Cord
  static absl::StatusOr<Image<ImageTrait>> CreateImage(
      const Dimensions& dimensions, absl::Cord data);

  ~Image() = default;

  // move constructor & assignment
  Image(Image&& rhs) noexcept
      : dimensions_(rhs.dimensions()),
        owning_ptr_(std::move(rhs.owning_ptr_)) {}

  Image& operator=(Image&& rhs) noexcept {
    if (this == &rhs) {
      return *this;
    }
    dimensions_ = rhs.dimensions_;
    owning_ptr_ = std::move(rhs.owning_ptr_);
    rhs.dimensions_ = {};
    return *this;
  }

  // copy constructor, performs deep copy
  Image(const Image& rhs) : Image<ImageTrait>(rhs.dimensions()) {
    std::copy(rhs.begin(), rhs.end(), begin());
  }

  // copy assignment, performs deep copy
  Image<ImageTrait>& operator=(const Image<ImageTrait>& rhs) {
    if (this == &rhs) {
      return *this;
    }
    if (area() != rhs.area()) {
      Reshape(rhs.dimensions());
    }
    std::copy(rhs.begin(), rhs.end(), begin());
    dimensions_ = rhs.dimensions();
    return *this;
  }

  // Reshape image and potentially performs deallocation + re-allocation.
  // Deallocation and re-allocator happens if the image area changes.
  void Reshape(Dimensions new_dimensions) {
    // Reset pointer if area is 0 to satisfy class invariant.
    if (new_dimensions.area() == 0) {
      *this = {};
      return;
    }
    // reallocate data if size is different
    if (area() != new_dimensions.area()) {
      owning_ptr_ = OwningPointer(new PixelType[new_dimensions.area()],
                                  std::default_delete<PixelType[]>{});
    }
    dimensions_ = new_dimensions;
  }

  // Check if coordinate is within the image
  bool IsInBounds(int32_t col, int32_t row) const {
    return col >= 0 && col < cols() && row >= 0 && row < rows();
  }
  bool IsInBounds(const Coordinate& coordinate) const {
    return IsInBounds(coordinate.col, coordinate.row);
  }

  // accessors/mutators:

  // Row-wise access.
  const PixelType* datarow(int32_t row) const {
    return data() + (row * cols());
  }
  PixelType* datarow(int32_t row) { return data() + (row * cols()); }
  // random pixel accessor
  const PixelType& operator()(int32_t col, int32_t row) const {
    return data()[(row * cols()) + col];
  }
  const PixelType& operator()(const Coordinate& coordinate) const {
    return (*this)(coordinate.col, coordinate.row);
  }
  // random pixel mutator
  PixelType& operator()(int32_t col, int32_t row) {
    return data()[(row * cols()) + col];
  }
  PixelType& operator()(const Coordinate& coordinate) {
    return (*this)(coordinate.col, coordinate.row);
  }

  // returns if image is empty
  bool empty() const { return !owning_ptr_; }

  // getters for image size, dimensions and area
  int32_t cols() const { return dimensions_.cols; }
  int32_t rows() const { return dimensions_.rows; }
  Dimensions dimensions() const { return dimensions_; }
  int32_t area() const { return dimensions_.area(); }

  // The size of the data in bytes.
  int32_t data_size() const { return area() * sizeof(PixelType); }

  // Direct access; none of these are bounds-checked.
  PixelType* data() { return owning_ptr_.get(); }
  const PixelType* data() const { return owning_ptr_.get(); }
  // Syntactic sugar for image.data()[index];
  PixelType& data(int32_t index) { return data()[index]; }
  const PixelType& data(int32_t index) const { return data()[index]; }

  // STL-style iterators
  iterator begin() { return data(); }
  iterator end() { return data() + area(); }
  const_iterator begin() const { return data(); }
  const_iterator end() const { return data() + area(); }

 private:
  // Unique pointer with custom deleter support.
  using OwningPointer =
      std::unique_ptr<PixelType, std::function<void(PixelType*)>>;

  // image dimensions, cols and rows
  Dimensions dimensions_;

  // Pointer to data.
  OwningPointer owning_ptr_;

  // Class invariant: m_owning_ptr == nullptr iff m_dimensions == {0, 0}.
};

template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> Image<ImageTrait>::CreateImage(
    const Dimensions& dimensions, absl::Cord data) {
  const size_t image_data_size = dimensions.area() * sizeof(PixelType);
  if (data.size() != image_data_size) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Data size must be equal to the image area times the pixel size: ",
        data.size(), " vs ", image_data_size));
  }
  std::string data_string;
  absl::CopyCordToString(data, &data_string);
  return CreateImageFromMemory<ImageTrait>(dimensions, std::move(data_string));
}

// Creates an image from existing memory. Please note that we
// can't guarantee proper memory alignment here. The DataType must have a data()
// method to access the underlying data pointer.
template <typename ImageTrait, typename DataType = std::string>
Image<ImageTrait> CreateImageFromMemory(const Dimensions& dimensions,
                                        DataType&& data) {
  // We move the data type into a raw pointer type to prolong the lifetime
  // of the underlying data.
  auto* data_owner = new DataType(std::forward<DataType>(data));
  return Image<ImageTrait>(
      dimensions,
      reinterpret_cast<typename ImageTrait::PixelType*>(data_owner->data()),
      [data_owner](typename ImageTrait::PixelType*) { delete data_owner; });
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_IMAGE_H_

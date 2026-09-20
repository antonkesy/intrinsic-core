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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_ARRAYS_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_ARRAYS_H_

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include "intrinsic/hardware/opcua/opcua_type_traits.h"
#include "open62541/types.h"
#include "open62541/types_generated_handling.h"

namespace intrinsic::opcua {

// Returns a std::vector corresponding to the given opcua array.
template <class T>
std::vector<typename OpcuaToC<T>::type> ToStdVector(const T* data,
                                                    size_t size) {
  std::vector<typename OpcuaToC<T>::type> vec;
  vec.reserve(size);
  for (int i = 0; i < size; ++i) {
    vec.push_back(*(data + i));
  }
  return vec;
}

// Returns a pair of a pointer to the opcua array and its size corresponding to
// the given std::vector.
// NOTE: the caller owns the returned pointer and must manage its lifetime.
// Returns a pair of nullptr and 0 if the array could not be created.
template <class T>
std::pair<typename CToOpcua<T>::type*, size_t> ToOpcuaArray(
    const std::vector<T>& vec) {
  const UA_DataType* dataType = CToOpcua<T>::value;

  void* array_ptr = nullptr;

  if constexpr (std::is_same_v<std::remove_const_t<T>, bool>) {
    // std::vector<bool> does not have a data() method. We need to copy
    // elements one by one.
    array_ptr = UA_Array_new(vec.size(), dataType);
    UA_Boolean* bool_array = static_cast<UA_Boolean*>(array_ptr);
    for (size_t i = 0; i < vec.size(); ++i) {
      bool_array[i] = static_cast<UA_Boolean>(vec[i]);
    }
  } else if constexpr (std::is_same_v<std::remove_const_t<T>, const char*> ||
                       std::is_same_v<std::remove_const_t<T>, char*>) {
    std::vector<UA_String> ua_strings;
    ua_strings.reserve(vec.size());
    for (const auto& s : vec) {
      ua_strings.push_back(UA_String_fromChars(s));
    }
    static_assert(CToOpcua<T>::value == &UA_TYPES[UA_TYPES_STRING]);
    auto ret = UA_Array_copy(ua_strings.data(), ua_strings.size(), &array_ptr,
                             dataType);
    for (auto& ua_str : ua_strings) {
      UA_String_clear(&ua_str);
    }
    if (ret != UA_STATUSCODE_GOOD) {
      return std::make_pair(nullptr, 0);
    }
  } else {
    // For other types, use UA_Array_copy with the data() pointer. It also works
    // for complex types like UA_String.
    auto ret = UA_Array_copy(vec.data(), vec.size(), &array_ptr, dataType);
    if (ret != UA_STATUSCODE_GOOD) {
      return std::make_pair(nullptr, 0);
    }
  }
  return std::make_pair(static_cast<typename CToOpcua<T>::type*>(array_ptr),
                        vec.size());
}

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_ARRAYS_H_

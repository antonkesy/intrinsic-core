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

#ifndef INTRINSIC_HARDWARE_OPCUA_OPCUA_VARIANT_H_
#define INTRINSIC_HARDWARE_OPCUA_OPCUA_VARIANT_H_

#include <vector>

#include "intrinsic/hardware/opcua/opcua_arrays.h"
#include "intrinsic/hardware/opcua/opcua_type_traits.h"
#include "open62541/types.h"
#include "open62541/types_generated_handling.h"

namespace intrinsic::opcua {

// Returns a UA_Variant corresponding to the given std::vector.
//
// NOTE: the caller owns the returned object and must manage its lifetime.
// Usage:
// ```
//    UA_Variant v = MakeArrayVariant<bool>({true, false, true});
//    .... // use variant
//
//    // Delete when done.
//    UA_Variant_clear(&v);
// ```
template <class T>
UA_Variant MakeArrayVariant(const std::vector<T>& vec) {
  UA_Variant vr;
  UA_Variant_init(&vr);

  typename OpcuaToC<T>::type* array_ptr;
  UA_UInt32 array_size;
  std::tie(array_ptr, array_size) = ToOpcuaArray(vec);
  UA_Variant_setArray(&vr, array_ptr, array_size, CToOpcua<T>::value);

  // Should be done after UA_Variant_setArray() to avoid getting overwritten.
  UA_UInt32* dims = UA_UInt32_new();
  *dims = array_size;
  vr.arrayDimensions = dims;
  vr.arrayDimensionsSize = 1;

  return vr;
}

// Returns a UA_Variant corresponding to the given std value.
//
// NOTE: the caller owns the returned object and must manage its lifetime.
// Usage:
// ```
//    UA_Variant v = MakeScalarVariant<bool>(true);
//    .... // use variant
//
//    // Delete when done.
//    UA_Variant_clear(&v);
// ```

template <class T>
UA_Variant MakeScalarVariant(const T& value) {
  UA_Variant vr;
  UA_Variant_init(&vr);

  typename CToOpcua<T>::type* data = CToOpcua<T>::kCreate();
  *data = value;

  UA_Variant_setScalar(&vr, data, CToOpcua<T>::value);
  return vr;
}

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_OPCUA_OPCUA_VARIANT_H_

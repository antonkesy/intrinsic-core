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

#ifndef INTRINSIC_WORLD_COLLISION_CHECK_CACHE_H_
#define INTRINSIC_WORLD_COLLISION_CHECK_CACHE_H_

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/hash/hash.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "third_party/simple_lru_cache/simple_lru_cache_inl.h"

namespace intrinsic {

// Type used to cache collision checks between objects with a relative pose.
using CachedCheckKey = std::tuple<PhysicalEntityId, PhysicalEntityId, Pose3d>;

// Helper that uses exact equality when comparing two tuples within a cache map.
// Note that using isApprox can cause issues as it does not generate a proper
// equality relationship.
struct CachedCheckEq {
  bool operator()(const CachedCheckKey& t, const CachedCheckKey& u) const {
    return std::get<0>(t) == std::get<0>(u) &&
           std::get<1>(t) == std::get<1>(u) &&
           std::get<2>(t).translation() == std::get<2>(u).translation() &&
           std::get<2>(t).quaternion().w() == std::get<2>(u).quaternion().w() &&
           std::get<2>(t).quaternion().x() == std::get<2>(u).quaternion().x() &&
           std::get<2>(t).quaternion().y() == std::get<2>(u).quaternion().y() &&
           std::get<2>(t).quaternion().z() == std::get<2>(u).quaternion().z();
  }
};

// The value is a set of pairs of [margin, is_in_collision].
using CollisionCheckCacheValue = WorldHashSet<std::pair<double, bool>>;

// Thin wrapper around google::simple_lru_cache::SimpleLRUCache that clears the
// cache on destruction.
class CollisionCheckCache {
 public:
  explicit CollisionCheckCache(int64_t total_units);
  ~CollisionCheckCache();

  CollisionCheckCacheValue* Lookup(const CachedCheckKey& key)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void Insert(const CachedCheckKey& key, CollisionCheckCacheValue* value,
              size_t size) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void Release(const CachedCheckKey& key, CollisionCheckCacheValue* value)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  void Clear() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  absl::Mutex mutex_;

 private:
  google::simple_lru_cache::SimpleLRUCache<
      CachedCheckKey, CollisionCheckCacheValue, absl::Hash<CachedCheckKey>,
      CachedCheckEq>
      cache_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COLLISION_CHECK_CACHE_H_

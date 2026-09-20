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

#ifndef INTRINSIC_SCENE_SERVICE_IMPORT_CACHE_H_
#define INTRINSIC_SCENE_SERVICE_IMPORT_CACHE_H_

#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/scene/proto/v1/imported_scene.pb.h"
#include "intrinsic/scene/proto/v1/scene_object.pb.h"
#include "intrinsic/util/lru_cache.h"

namespace intrinsic {

// A simple LRU cache based cache for both single SceneObject and ImportedScene
// imports keyed by hash of the import file data.
struct ImportCache {
  ImportCache();
  ~ImportCache();

  std::optional<intrinsic_proto::scene_object::v1::SceneObject> GetSceneObject(
      const std::string& hash);

  std::optional<intrinsic_proto::scene_object::v1::ImportedScene>
  GetImportedScene(const std::string& hash);

  absl::Status StoreSceneObject(
      const std::string& hash,
      intrinsic_proto::scene_object::v1::SceneObject scene_object);

  absl::Status StoreImportedScene(
      const std::string& hash,
      intrinsic_proto::scene_object::v1::ImportedScene imported_scene);

 private:
  // Adaptors for the difference between g3 and insrc. Remove after kiruna is
  // all finished.
  std::pair<std::optional<intrinsic_proto::scene_object::v1::SceneObject>,
            std::optional<intrinsic_proto::scene_object::v1::ImportedScene>>*
  Lookup(const std::string& hash) ABSL_SHARED_LOCKS_REQUIRED(lru_cache_mutex);

  void Release(
      const std::string& hash,
      std::pair<
          std::optional<intrinsic_proto::scene_object::v1::SceneObject>,
          std::optional<intrinsic_proto::scene_object::v1::ImportedScene>>*
          value) ABSL_EXCLUSIVE_LOCKS_REQUIRED(lru_cache_mutex);

  void Insert(
      const std::string& hash,
      std::pair<
          std::optional<intrinsic_proto::scene_object::v1::SceneObject>,
          std::optional<intrinsic_proto::scene_object::v1::ImportedScene>>*
          value) ABSL_EXCLUSIVE_LOCKS_REQUIRED(lru_cache_mutex);

  static constexpr int kRawImportCacheSize = 100;
  mutable absl::Mutex lru_cache_mutex;
  intrinsic::LruCache<
      std::string,
      std::pair<
          std::optional<intrinsic_proto::scene_object::v1::SceneObject>,
          std::optional<intrinsic_proto::scene_object::v1::ImportedScene>>>
      lru_cache ABSL_GUARDED_BY(lru_cache_mutex);
};

}  // namespace intrinsic
#endif  // INTRINSIC_SCENE_SERVICE_IMPORT_CACHE_H_

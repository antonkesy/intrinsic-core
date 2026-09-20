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

#include "intrinsic/scene/service/import_cache.h"

#include <utility>

#include "intrinsic/scene/validate/scene_object_validation.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

using ::intrinsic_proto::scene_object::v1::ImportedScene;
using ::intrinsic_proto::scene_object::v1::SceneObject;

ImportCache::ImportCache() : lru_cache(kRawImportCacheSize) {}
ImportCache::~ImportCache() {
  absl::MutexLock lock(&lru_cache_mutex);
  lru_cache.removeAll();
  lru_cache.clear();
}

std::optional<SceneObject> ImportCache::GetSceneObject(
    const std::string& hash) {
  // TODO(b/446969475): Trace metrics for cache hit/miss rates.
  absl::MutexLock lock(&lru_cache_mutex);
  auto* value = Lookup(hash);
  if (value == nullptr) return std::nullopt;
  auto result = value->first;
  Release(hash, value);
  return result;
}

std::optional<ImportedScene> ImportCache::GetImportedScene(
    const std::string& hash) {
  absl::MutexLock lock(&lru_cache_mutex);
  auto* value = Lookup(hash);
  if (value == nullptr) return std::nullopt;
  auto result = value->second;
  Release(hash, value);
  return result;
}

absl::Status ImportCache::StoreSceneObject(const std::string& hash,
                                           SceneObject scene_object) {
  INTR_RETURN_IF_ERROR(scene_object::ValidateSceneObject(scene_object));
  absl::MutexLock lock(&lru_cache_mutex);
  if (auto* cached_import = Lookup(hash); cached_import != nullptr) {
    cached_import->first = scene_object;
    Release(hash, cached_import);
  } else {
    Insert(
        hash,
        new std::pair<std::optional<SceneObject>, std::optional<ImportedScene>>(
            scene_object, std::nullopt));
  }
  return absl::OkStatus();
}

absl::Status ImportCache::StoreImportedScene(const std::string& hash,
                                             ImportedScene imported_scene) {
  // Validate the imported scene objects.
  for (const auto& [_, scene_object] :
       imported_scene.scene_objects().objects()) {
    INTR_RETURN_IF_ERROR(scene_object::ValidateSceneObject(scene_object));
  }

  absl::MutexLock lock(&lru_cache_mutex);
  if (auto* cached_import = Lookup(hash); cached_import != nullptr) {
    cached_import->second = imported_scene;
    Release(hash, cached_import);
  } else {
    Insert(
        hash,
        new std::pair<std::optional<SceneObject>, std::optional<ImportedScene>>(
            std::nullopt, imported_scene));
  }

  return absl::OkStatus();
}

std::pair<std::optional<SceneObject>, std::optional<ImportedScene>>*
ImportCache::Lookup(const std::string& hash)
    ABSL_SHARED_LOCKS_REQUIRED(lru_cache_mutex) {
  return lru_cache.lookup(hash);
}

void ImportCache::Release(
    const std::string& hash,
    std::pair<std::optional<SceneObject>, std::optional<ImportedScene>>* value)
    ABSL_EXCLUSIVE_LOCKS_REQUIRED(lru_cache_mutex) {
  lru_cache.release(hash, value);
}

void ImportCache::Insert(
    const std::string& hash,
    std::pair<std::optional<SceneObject>, std::optional<ImportedScene>>* value)
    ABSL_EXCLUSIVE_LOCKS_REQUIRED(lru_cache_mutex) {
  lru_cache.insert(hash, value, 1);
}

}  // namespace intrinsic

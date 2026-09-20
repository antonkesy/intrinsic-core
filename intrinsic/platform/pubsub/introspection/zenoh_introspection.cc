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

#include <atomic>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_handle.h"
#include "intrinsic/production/external/googleinit/googleinit.h"
#include "intrinsic/stats/opencensus.h"
#include "nlohmann/json.hpp"
#include "opencensus/stats/stats.h"
#include "opencensus/tags/tag_key.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

ABSL_FLAG(std::string, zenoh_config, "",
          "JSON config file for Zenoh. If not provided, defaults to the config "
          "provided by GetZenohPeerConfig()");

namespace {

std::atomic<bool> g_shutdown_requested = false;
static_assert(std::atomic<bool>::is_always_lock_free);

}  // namespace

namespace intrinsic {

namespace {

using ::nlohmann::json;
using ::opencensus::stats::Aggregation;
using ::opencensus::stats::MeasureInt64;
using ::opencensus::stats::ViewDescriptor;
using ::opencensus::tags::TagKey;

constexpr char kPubSubTopicBytesName[] = "intrinsic/pubsub/topic_bytes";
constexpr char kPubSubTopicBytesDesc[] = "Total number of bytes";

constexpr char kPubSubTopicMessagesName[] = "intrinsic/pubsub/topic_messages";
constexpr char kPubSubTopicMessagesDesc[] = "Total number of messages";

constexpr char kNumPublishersName[] = "intrinsic/pubsub/num_publishers";
constexpr char kNumPublishersDesc[] =
    "Number of publishers in each Zenoh session";

constexpr char kNumSubscribersName[] = "intrinsic/pubsub/num_subscribers";
constexpr char kNumSubscribersDesc[] =
    "Number of subscribers in each Zenoh session";

MeasureInt64 PubSubTopicBytes() {
  static const auto measure =
      MeasureInt64::Register(kPubSubTopicBytesName, kPubSubTopicBytesDesc, "1");
  return measure;
}

MeasureInt64 PubSubTopicMessages() {
  static const auto measure = MeasureInt64::Register(
      kPubSubTopicMessagesName, kPubSubTopicMessagesDesc, "1");
  return measure;
}

MeasureInt64 PubSubNumPublishers() {
  static const auto measure =
      MeasureInt64::Register(kNumPublishersName, kNumPublishersDesc, "1");
  return measure;
}

MeasureInt64 PubSubNumSubscribers() {
  static const auto measure =
      MeasureInt64::Register(kNumSubscribersName, kNumSubscribersDesc, "1");
  return measure;
}

TagKey PubSubDirectionNameKey() {
  static const auto key = opencensus::tags::TagKey::Register("direction");
  return key;
}

TagKey PubSubTopicNameKey() {
  static const auto key = opencensus::tags::TagKey::Register("topic_name");
  return key;
}

TagKey PubSubHostNameKey() {
  static const auto key = opencensus::tags::TagKey::Register("host_name");
  return key;
}

REGISTER_MODULE_INITIALIZER(pubsub_metrics, {
  PubSubTopicBytes();
  PubSubTopicMessages();
  PubSubNumPublishers();
  PubSubNumSubscribers();

  ViewDescriptor()
      .set_name(kPubSubTopicBytesName)
      .set_measure(kPubSubTopicBytesName)
      .set_description(kPubSubTopicBytesDesc)
      .set_aggregation(Aggregation::LastValue())
      .add_column(PubSubDirectionNameKey())
      .add_column(PubSubTopicNameKey())
      .add_column(PubSubHostNameKey())
      .RegisterForExport();

  ViewDescriptor()
      .set_name(kPubSubTopicMessagesName)
      .set_measure(kPubSubTopicMessagesName)
      .set_description(kPubSubTopicMessagesDesc)
      .set_aggregation(Aggregation::LastValue())
      .add_column(PubSubDirectionNameKey())
      .add_column(PubSubTopicNameKey())
      .add_column(PubSubHostNameKey())
      .RegisterForExport();

  ViewDescriptor()
      .set_name(kNumPublishersName)
      .set_measure(kNumPublishersName)
      .set_description(kNumPublishersDesc)
      .set_aggregation(Aggregation::LastValue())
      .add_column(PubSubHostNameKey())
      .RegisterForExport();

  ViewDescriptor()
      .set_name(kNumSubscribersName)
      .set_measure(kNumSubscribersName)
      .set_description(kNumSubscribersDesc)
      .set_aggregation(Aggregation::LastValue())
      .add_column(PubSubHostNameKey())
      .RegisterForExport();
});

class ZenohIntrospector {
 public:
  ZenohIntrospector() {
    introspection_callback_fptr_ = std::make_unique<imw_callback_functor_t>(
        [this](const char* keyexpr, const void* blob, const size_t blob_len) {
          this->introspection_callback(keyexpr, blob, blob_len);
        });
  }
  ZenohIntrospector(const ZenohIntrospector&) = delete;
  ZenohIntrospector& operator=(const ZenohIntrospector&) = delete;
  ZenohIntrospector(ZenohIntrospector&&) = delete;
  ZenohIntrospector& operator=(ZenohIntrospector&&) = delete;

  ~ZenohIntrospector() {
    if (zenoh_init_complete_) {
      Zenoh().imw_fini();
    }
  }

  bool init() {
    std::string config_flag = absl::GetFlag(FLAGS_zenoh_config);
    std::string config;
    if (config_flag.empty()) {
      config = intrinsic::GetZenohPeerConfig();
    } else {
      absl::Status status =
          file::GetContents(config_flag, &config, file::Defaults());
      if (!status.ok())
        LOG(FATAL) << "Couldn't read the config file: " << config_flag;
    }

    if (config.empty()) {
      LOG(FATAL) << "Empty Zenoh config";
    }
    const imw_ret_t init_ret = Zenoh().imw_init(config.c_str());
    if (init_ret != IMW_OK) {
      LOG(FATAL) << "Error from imw_init with config " << config;
    }
    zenoh_init_complete_ = true;

    const imw_ret_t sub_ret = Zenoh().imw_create_subscription(
        "in/_introspection/sessions/*", introspection_callback_s, "{}",
        introspection_callback_fptr_.get());
    if (sub_ret != IMW_OK) {
      LOG(ERROR) << "Could not create Zenoh introspection subscriber";
      return false;
    }
    return true;
  }

 private:
  bool zenoh_init_complete_ = false;
  std::unique_ptr<imw_callback_functor_t> introspection_callback_fptr_;

  template <typename ValueType>
  ValueType extract_value(json& container, absl::string_view key,
                          ValueType default_value) {
    if (container.contains(key)) {
      return container[key].get<ValueType>();
    }

    return default_value;
  }

  void introspection_callback(const char* keyexpr, const void* blob,
                              const size_t blob_len) {
    const std::string json_payload(reinterpret_cast<const char*>(blob),
                                   blob_len);
    json j = json::parse(json_payload, nullptr, false);
    if (j.is_discarded()) {
      LOG(ERROR) << "JSON parse error";
      return;
    }

    std::string j_hostname = extract_value<std::string>(j, "hostname", "");
    std::string j_zid = extract_value<std::string>(j, "zenoh_id", "");
    uint64_t num_publishers = 0;
    uint64_t num_subscribers = 0;

    if (j.contains("publishers") && !j["publishers"].is_null()) {
      json j_pubs = j["publishers"];
      num_publishers = j_pubs.size();
      for (json::iterator it = j_pubs.begin(); it != j_pubs.end(); ++it) {
        std::string j_pub_name = extract_value<std::string>(*it, "name", "");
        uint64_t j_pub_messages = extract_value<uint64_t>(*it, "messages", 0);
        uint64_t j_pub_bytes = extract_value<uint64_t>(*it, "bytes", 0);

        opencensus::stats::Record(
            {
                {PubSubTopicBytes(), j_pub_bytes},
            },
            {
                {PubSubDirectionNameKey(), "pub"},
                {PubSubTopicNameKey(), j_pub_name},
                {PubSubHostNameKey(), j_hostname},
            });

        opencensus::stats::Record(
            {
                {PubSubTopicMessages(), j_pub_messages},
            },
            {
                {PubSubDirectionNameKey(), "pub"},
                {PubSubTopicNameKey(), j_pub_name},
                {PubSubHostNameKey(), j_hostname},
            });
      }
    }

    // Recording the number of publishers outside of the `if` statement above to
    // make sure that we record clients with zero publishers.
    opencensus::stats::Record(
        {
            {PubSubNumPublishers(), num_publishers},
        },
        {
            {PubSubHostNameKey(), j_hostname},
        });

    if (j.contains("subscriptions") && !j["subscriptions"].is_null()) {
      json j_subs = j["subscriptions"];
      num_subscribers = j_subs.size();
      for (json::iterator it = j_subs.begin(); it != j_subs.end(); ++it) {
        std::string j_sub_name = extract_value<std::string>(*it, "name", "");
        uint64_t j_sub_messages = extract_value<uint64_t>(*it, "messages", 0);
        uint64_t j_sub_bytes = extract_value<uint64_t>(*it, "bytes", 0);

        opencensus::stats::Record(
            {
                {PubSubTopicBytes(), j_sub_bytes},
            },
            {
                {PubSubDirectionNameKey(), "sub"},
                {PubSubTopicNameKey(), j_sub_name},
                {PubSubHostNameKey(), j_hostname},
            });

        opencensus::stats::Record(
            {
                {PubSubTopicMessages(), j_sub_messages},
            },
            {
                {PubSubDirectionNameKey(), "sub"},
                {PubSubTopicNameKey(), j_sub_name},
                {PubSubHostNameKey(), j_hostname},
            });
      }
    }

    // Recording the number of subscribers outside of the `if` statement above
    // to make sure that we record clients with zero subscribers.
    opencensus::stats::Record(
        {
            {PubSubNumSubscribers(), num_subscribers},
        },
        {
            {PubSubHostNameKey(), j_hostname},
        });
  }
  static void introspection_callback_s(const char* keyexpr, const void* blob,
                                       size_t blob_len, void* fptr) {
    (*static_cast<imw_callback_functor_t*>(fptr))(keyexpr, blob, blob_len);
  }
};

absl::Status MainImpl() {
  ZenohIntrospector zenoh_introspector;
  if (!zenoh_introspector.init()) {
    return absl::FailedPreconditionError("Init failed");
  }

  std::signal(SIGINT, [](int) {
    g_shutdown_requested = true;
    g_shutdown_requested.notify_all();
  });

  std::signal(SIGTERM, [](int) {
    g_shutdown_requested = true;
    g_shutdown_requested.notify_all();
  });

  g_shutdown_requested.wait(false);

  LOG(INFO) << "zenoh introspection is cleanly shutting down";
  return absl::OkStatus();
}

}  // namespace

}  // namespace intrinsic

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  intrinsic::OpenCensusPlugin open_census;
  QCHECK_OK(intrinsic::MainImpl());
  return 0;
}

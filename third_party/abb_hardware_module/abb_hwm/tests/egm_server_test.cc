#include "third_party/abb_hardware_module/abb_hwm/egm_server.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/text_format.h"
#include "intrinsic/util/thread/thread_options.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_serialization.h"
#include "third_party/abb_hardware_module/abb_hwm/udp_server_interface.h"

using namespace abb_hardware_module;

namespace {

constexpr absl::string_view kEgmRobot = R"(
header: {
    seqno: 1
    tm: 2
    mtype: MSGTYPE_DATA
    }
feedBack: {
    joints: {
        joints: 1.1
        joints: 2.1
        joints: 3.1
        joints: 4.1
        joints: 5.1
        joints: 6.1
    }
    cartesian: {
        orient: {
            u0: 1.1
            u1: 1.2
            u2: 1.3
            u3: 1.4
        }
        pos: {
            x: 1
            y: 2
            z: 3
        }
        euler: {
            x: 1
            y: 2
            z: 3
        }
    }
    externalJoints: {
        joints: 1.7
    }
    time: {
        sec: 1
        usec: 42
    }
}
planned: {
    joints: {
        joints: 1.2
        joints: 2.2
        joints: 3.2
        joints: 4.2
        joints: 5.2
        joints: 6.2
    }
    cartesian: {
        orient: {
            u0: 2.1
            u1: 2.2
            u2: 2.3
            u3: 2.4
        }
        pos: {
            x: 1.1
            y: 2.2
            z: 3.3
        }
        euler: {
            x: 1.1
            y: 2.2
            z: 3.3
        }
    }
    externalJoints: {
        joints: 1.66
    }
    time: {
        sec: 2
        usec: 43
    }
}
motorState: {
    state: MOTORS_ON
}
mciState: {
    state: MCI_RUNNING
}
mciConvergenceMet: false
testSignals: {
    signals: 1.99
}
rapidExecState: {
    state: RAPID_RUNNING
}
measuredForce: {
    fcActive: true
    force: 1
    force: 2
    force: 3
    force: 4
    force: 5
    force: 6
}
utilizationRate: 1.0
moveIndex: 2
CollisionInfo: {
    collsionTriggered: false
    collDetQuota: 0.1
    collDetQuota: 0.2
}
RAPIDfromRobot: {
    digVal: false
    dnum: 1.1
    dnum: 1.2
}
)";

constexpr absl::string_view kEgmSensor = R"(
header: {
    seqno: 2
    tm: 3
    mtype: MSGTYPE_CORRECTION
    }
planned: {
    joints: {
        joints: 1.3
        joints: 2.3
        joints: 3.3
        joints: 4.3
        joints: 5.3
        joints: 6.3
    }
    cartesian: {
        orient: {
            u0: 2.2
            u1: 2.4
            u2: 2.5
            u3: 2.6
        }
        pos: {
            x: 1.2
            y: 2.3
            z: 3.4
        }
        euler: {
            x: 1.2
            y: 2.3
            z: 3.4
        }
    }
    externalJoints: {
        joints: 1.67
    }
    time: {
        sec: 3
        usec: 47
    }
}
speedRef: {
    joints: {
        joints: 1.4
        joints: 2.4
        joints: 3.4
        joints: 4.4
        joints: 5.4
        joints: 6.4
    }
    cartesians:{
        value: 1.2
        value: 2.3
        value: 3.4
        value: 4.5
        value: 5.6
        value: 6.7
    }
    externalJoints:{
        joints: 1.69
    }
}
RAPIDtoRobot: {
    digVal: true
    dnum: 1.3
    dnum: 1.4
}

)";

std::string CreateSerializedEgmRobot(size_t seqno) {
  abb::egm::EgmRobot egm_robot;
  google::protobuf::TextFormat::ParseFromString(kEgmRobot, &egm_robot);
  egm_robot.mutable_header()->set_seqno(seqno);
  std::string egm_robot_string;
  egm_robot.SerializeToString(&egm_robot_string);
  return egm_robot_string;
}

egm_data::EgmSensor CreateEgmSensor(size_t seqno) {
  abb::egm::EgmSensor egm_sensor_proto;
  google::protobuf::TextFormat::ParseFromString(kEgmSensor, &egm_sensor_proto);
  egm_data::EgmSensor egm_sensor;
  FromProto(egm_sensor_proto, &egm_sensor);
  egm_sensor.header->seqno = seqno;
  return egm_sensor;
}

constexpr size_t kCommunicationPeriodMs = 100;

class FakeUdpServer : public UDPServerInterface {
 public:
  std::string last_response;

  absl::Status Init(MessageHandler handler) override {
    handler_ = handler;
    return absl::OkStatus();
  }

  void run() override {
    int i = 0;
    while (!bShouldDie) {
      last_response = handler_(CreateSerializedEgmRobot(i++));
      // // sleep 0.1 seconds
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kCommunicationPeriodMs));
    }
  }

  void Stop() override { bShouldDie = true; }

 private:
  MessageHandler handler_;
  bool bShouldDie = false;
};

TEST(EgmServerTest, Init) {
  std::unique_ptr<FakeUdpServer> udp_server = std::make_unique<FakeUdpServer>();
  // Use default (non-realtime) thread options for testing
  intrinsic::ThreadOptions test_thread_options;
  EgmServer egm_server(std::move(udp_server), test_thread_options);
  auto status = egm_server.Init();
  if (!status.ok()) {
    std::cerr << "Init failed: " << status.ToString() << std::endl;
  }
  ASSERT_TRUE(status.ok()) << status.message();
}

TEST(EgmServerTest, BasicUsage) {
  std::unique_ptr<FakeUdpServer> udp_server = std::make_unique<FakeUdpServer>();
  // Borrow a pointer to the FakeUdpServer before transferring ownership
  FakeUdpServer* udp_server_ptr = udp_server.get();
  intrinsic::ThreadOptions test_thread_options;
  EgmServer egm_server(std::move(udp_server), test_thread_options);
  ASSERT_TRUE(egm_server.Init().ok());
  // Wait for a period corresponding to 10 cycles of the UDP communication loop
  // to make sure that data is available.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(10 * kCommunicationPeriodMs));

  for (int i = 0; i < 5; i++) {
    auto egm_robot_or_status = egm_server.GetDataFromRobot();
    ASSERT_TRUE(egm_robot_or_status.ok());
    egm_data::EgmSensor egm_sensor = CreateEgmSensor(i);
    egm_server.SetDataToRobot(egm_sensor);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kCommunicationPeriodMs));
    // make sure that the data received on the UDP interface was updated
    // correctly
    abb::egm::EgmSensor egm_sensor_proto;
    ToProto(egm_sensor, &egm_sensor_proto);
    ASSERT_EQ(egm_sensor_proto.SerializeAsString(),
              udp_server_ptr->last_response);
  }
}

TEST(EgmSequenceCheckerTest, CorrectSequence) {
  EgmSequenceChecker seq_checker(3);
  seq_checker.Init(0);
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(2).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(3).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(4).ok());
}

TEST(EgmSequenceCheckerTest, DecrementingSequence) {
  EgmSequenceChecker seq_checker(3);
  seq_checker.Init(0);
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(2).ok());
  ASSERT_FALSE(seq_checker.CheckSequenceNumber(1).ok());
}

TEST(EgmSequenceCheckerTest, RepeatingSequence) {
  EgmSequenceChecker seq_checker(3);
  seq_checker.Init(0);
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  // Not ok, more than tolerance repeats
  ASSERT_FALSE(seq_checker.CheckSequenceNumber(1).ok());
}

TEST(EgmSequenceCheckerTest, MissedSequenceNumber) {
  EgmSequenceChecker seq_checker(3);
  seq_checker.Init(0);
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  // This is ok, within tolerance
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(3).ok());
  // also ok, within tolerance
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(4).ok());
  // not ok, beyond tolerance
  ASSERT_FALSE(seq_checker.CheckSequenceNumber(8).ok());
}

TEST(EgmSequenceCheckerTest, AcceptableSequence) {
  EgmSequenceChecker seq_checker(3);
  seq_checker.Init(0);
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(1).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(2).ok());
  // a couple of repeats is ok
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(3).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(3).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(3).ok());
  // reset the repeat counter by again getting incrementing number
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(4).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(5).ok());
  // repeats should be acceptable again
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(5).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(5).ok());
  // continuing with a stretch of healthy sequence
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(6).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(7).ok());
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(8).ok());
  // skip a tolerable amount of messages
  ASSERT_TRUE(seq_checker.CheckSequenceNumber(10).ok());
}
}  // namespace

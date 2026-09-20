#include "third_party/abb_hardware_module/abb_hwm/egm_serialization.h"

#include <gtest/gtest.h>

#include <string>

#include "absl/strings/string_view.h"
#include "google/protobuf/text_format.h"
#include "third_party/abb_egm/egm.pb.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"

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

abb::egm::EgmRobot ParseEgmRobotProto() {
  abb::egm::EgmRobot egm_robot;
  google::protobuf::TextFormat::ParseFromString(kEgmRobot, &egm_robot);
  return egm_robot;
}

abb::egm::EgmSensor ParseEgmSensorProto() {
  abb::egm::EgmSensor egm_sensor;
  google::protobuf::TextFormat::ParseFromString(kEgmSensor, &egm_sensor);
  return egm_sensor;
}

TEST(EgmSerializationTest, EgmRobotRoundTrip) {
  abb::egm::EgmRobot egm_robot = ParseEgmRobotProto();
  egm_data::EgmRobot egm_robot_data;
  FromProto(egm_robot, &egm_robot_data);
  abb::egm::EgmRobot egm_robot_round_trip;
  ToProto(egm_robot_data, &egm_robot_round_trip);

  std::string egm_robot_string, egm_robot_roundtrip_string;
  egm_robot.SerializeToString(&egm_robot_string);
  egm_robot_round_trip.SerializeToString(&egm_robot_roundtrip_string);
  ASSERT_EQ(egm_robot_string, egm_robot_roundtrip_string);
}

TEST(EgmSerializationTest, EgmSensorRoundTrip) {
  abb::egm::EgmSensor egm_sensor = ParseEgmSensorProto();
  egm_data::EgmSensor egm_sensor_data;
  FromProto(egm_sensor, &egm_sensor_data);
  abb::egm::EgmSensor egm_sensor_round_trip;
  ToProto(egm_sensor_data, &egm_sensor_round_trip);
  std::string egm_sensor_string, egm_sensor_roundtrip_string;
  egm_sensor.SerializeToString(&egm_sensor_string);
  egm_sensor_round_trip.SerializeToString(&egm_sensor_roundtrip_string);
  ASSERT_EQ(egm_sensor_string, egm_sensor_roundtrip_string);
}

}  // namespace

#ifndef ABB_HARDWARE_MODULE_ABB_HWM_EGM_DATA_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_EGM_DATA_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace abb_hardware_module {
namespace egm_data {

struct EgmHeader {
  std::optional<uint32_t> seqno;
  std::optional<uint32_t> tm;
  enum MessageType {
    MSGTYPE_UNDEFINED = 0,
    MSGTYPE_COMMAND = 1,
    MSGTYPE_DATA = 2,
    MSGTYPE_CORRECTION = 3,
    MSGTYPE_PATH_CORRECTION = 4,
  };
  std::optional<MessageType> mtype;
};

struct EgmCartesian {
  double x;
  double y;
  double z;
};

struct EgmQuaternion {
  double u0;
  double u1;
  double u2;
  double u3;
};

struct EgmEuler {
  double x;
  double y;
  double z;
};

struct EgmClock {
  uint64_t sec;
  uint64_t usec;
};

struct EgmPose {
  std::optional<EgmCartesian> pos;
  std::optional<EgmQuaternion> orient;
  std::optional<EgmEuler> euler;
};

struct EgmCartesianSpeed {
  intrinsic::eigenmath::Vector6d value;
};

struct EgmJoints {
  intrinsic::eigenmath::VectorNd joints;
};

struct EgmExternalJoints {
  intrinsic::eigenmath::VectorNd joints;
};

struct EgmPlanned {
  std::optional<EgmJoints> joints;
  std::optional<EgmPose> cartesian;
  std::optional<EgmJoints> externalJoints;
  std::optional<EgmClock> time;
};

struct EgmSpeedRef {
  std::optional<EgmJoints> joints;
  std::optional<EgmCartesianSpeed> cartesians;
  std::optional<EgmJoints> externalJoints;
};

struct EgmPathCorr {
  EgmCartesian pos;
  uint32_t age;
};

struct EgmFeedBack {
  std::optional<EgmJoints> joints;
  std::optional<EgmPose> cartesian;
  std::optional<EgmJoints> externalJoints;
  std::optional<EgmClock> time;
};

struct EgmMotorState {
  enum MotorStateType {
    MOTORS_UNDEFINED = 0,
    MOTORS_ON = 1,
    MOTORS_OFF = 2,
  };
  MotorStateType state;
};

struct EgmMCIState {
  enum MCIStateType {
    MCI_UNDEFINED = 0,
    MCI_ERROR = 1,
    MCI_STOPPED = 2,
    MCI_RUNNING = 3,
  };
  MCIStateType state;
};

struct EgmRapidCtrlExecState {
  enum RapidCtrlExecStateType {
    RAPID_UNDEFINED = 0,
    RAPID_STOPPED = 1,
    RAPID_RUNNING = 2,
  };
  RapidCtrlExecStateType state;
};

struct EgmTestSignals {
  intrinsic::eigenmath::VectorNd signals;
};

struct EgmMeasuredForce {
  std::optional<bool> fcActive;
  intrinsic::eigenmath::Vector6d force;
};

struct EgmCollisionInfo {
  std::optional<bool> collisionTriggered;
  intrinsic::eigenmath::VectorNd collDetQuota;
};

struct EgmRAPIDdata {
  std::optional<bool> digVal;
  intrinsic::eigenmath::VectorNd dnum;
};

struct EgmRobot {
  std::optional<EgmHeader> header;
  std::optional<EgmFeedBack> feedBack;
  std::optional<EgmPlanned> planned;
  std::optional<EgmMotorState> motorState;
  std::optional<EgmMCIState> mciState;
  std::optional<bool> mciConvergenceMet;
  std::optional<EgmTestSignals> testSignals;
  std::optional<EgmRapidCtrlExecState> rapidExecState;
  std::optional<EgmMeasuredForce> measuredForce;
  std::optional<double> utilizationRate;
  std::optional<uint32_t> moveIndex;
  std::optional<EgmCollisionInfo> CollisionInfo;
  std::optional<EgmRAPIDdata> RAPIDfromRobot;
};

struct EgmSensor {
  std::optional<EgmHeader> header;
  std::optional<EgmPlanned> planned;
  std::optional<EgmSpeedRef> speedRef;
  std::optional<EgmRAPIDdata> RAPIDtoRobot;
};

struct EgmSensorPathCorr {
  std::optional<EgmHeader> header;
  std::optional<EgmPathCorr> pathCorr;
};

inline std::string PrintEgmRobot(const EgmRobot& robot)
    INTRINSIC_NON_REALTIME_ONLY {
  std::string str = "EgmRobot:\n";
  if (robot.header.has_value()) {
    str += "Header:\n";
    str += "  seqno: " + std::to_string(robot.header->seqno.value()) + "\n";
    str += "  tm: " + std::to_string(robot.header->tm.value()) + "\n";
    str += "  mtype: " + std::to_string(robot.header->mtype.value()) + "\n";
  }
  if (robot.feedBack.has_value()) {
    str += "FeedBack:\n";
    if (robot.feedBack->joints.has_value()) {
      str += "  Joints: ";
      for (auto& joint : robot.feedBack->joints.value().joints) {
        str += std::to_string(joint) + " ";
      }
      str += "\n";
    }
    if (robot.feedBack->cartesian.has_value()) {
      str += "  Cartesian:\n";
      if (robot.feedBack->cartesian->pos.has_value()) {
        str += "    Pos: " + std::to_string(robot.feedBack->cartesian->pos->x) +
               " " + std::to_string(robot.feedBack->cartesian->pos->y) + " " +
               std::to_string(robot.feedBack->cartesian->pos->z) + "\n";
      }
      if (robot.feedBack->cartesian->orient.has_value()) {
        str += "    Orient: " +
               std::to_string(robot.feedBack->cartesian->orient->u0) + " " +
               std::to_string(robot.feedBack->cartesian->orient->u1) + " " +
               std::to_string(robot.feedBack->cartesian->orient->u2) + " " +
               std::to_string(robot.feedBack->cartesian->orient->u3) + "\n";
      }
      if (robot.feedBack->cartesian->euler.has_value()) {
        str += "    Euler: " +
               std::to_string(robot.feedBack->cartesian->euler->x) + " " +
               std::to_string(robot.feedBack->cartesian->euler->y) + " " +
               std::to_string(robot.feedBack->cartesian->euler->z) + "\n";
      }
    }
    if (robot.feedBack->externalJoints.has_value()) {
      str += "  ExternalJoints: ";
      for (auto& joint : robot.feedBack->externalJoints.value().joints) {
        str += std::to_string(joint) + " ";
      }
      str += "\n";
    }
    if (robot.feedBack->time.has_value()) {
      str += "  Time: " + std::to_string(robot.feedBack->time->sec) + " " +
             std::to_string(robot.feedBack->time->usec) + "\n";
    }
  }
  return str;
}

inline std::string PrintEgmSensor(const EgmSensor& sensor)
    INTRINSIC_NON_REALTIME_ONLY {
  std::string str = "EgmSensor:\n";
  if (sensor.header.has_value()) {
    str += "Header:\n";
    str += "  seqno: " + std::to_string(sensor.header->seqno.value()) + "\n";
    str += "  tm: " + std::to_string(sensor.header->tm.value()) + "\n";
    str += "  mtype: " + std::to_string(sensor.header->mtype.value()) + "\n";
  }
  if (sensor.planned.has_value()) {
    str += "Planned:\n";
    if (sensor.planned->joints.has_value()) {
      str += "  Joints: ";
      for (auto& joint : sensor.planned->joints.value().joints) {
        str += std::to_string(joint) + " ";
      }
      str += "\n";
    }
    if (sensor.planned->cartesian.has_value()) {
      str += "  Cartesian:\n";
      if (sensor.planned->cartesian->pos.has_value()) {
        str += "    Pos: " + std::to_string(sensor.planned->cartesian->pos->x) +
               " " + std::to_string(sensor.planned->cartesian->pos->y) + " " +
               std::to_string(sensor.planned->cartesian->pos->z) + "\n";
      }
      if (sensor.planned->cartesian->orient.has_value()) {
        str += "    Orient: " +
               std::to_string(sensor.planned->cartesian->orient->u0) + " " +
               std::to_string(sensor.planned->cartesian->orient->u1) + " " +
               std::to_string(sensor.planned->cartesian->orient->u2) + " " +
               std::to_string(sensor.planned->cartesian->orient->u3) + "\n";
      }
      if (sensor.planned->cartesian->euler.has_value()) {
        str += "    Euler: " +
               std::to_string(sensor.planned->cartesian->euler->x) + " " +
               std::to_string(sensor.planned->cartesian->euler->y) + " " +
               std::to_string(sensor.planned->cartesian->euler->z) + "\n";
      }
    }
    if (sensor.planned->externalJoints.has_value()) {
      str += "  ExternalJoints: ";
      for (auto& joint : sensor.planned->externalJoints.value().joints) {
        str += std::to_string(joint) + " ";
      }
      str += "\n";
    }
    if (sensor.planned->time.has_value()) {
      str += "  Time: " + std::to_string(sensor.planned->time->sec) + " " +
             std::to_string(sensor.planned->time->usec) + "\n";
    }

    if (sensor.speedRef.has_value()) {
      str += "SpeedRef:\n";
      if (sensor.speedRef->joints.has_value()) {
        str += "  Joints: ";
        for (auto& joint : sensor.speedRef->joints.value().joints) {
          str += std::to_string(joint) + " ";
        }
        str += "\n";
      }
    }
  }
  return str;
}

}  // namespace egm_data
}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_EGM_DATA_H_

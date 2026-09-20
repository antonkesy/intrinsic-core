#ifndef ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERIALIZATION_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERIALIZATION_H_

#include "intrinsic/icon/testing/realtime_annotations.h"
#include "third_party/abb_egm/egm.pb.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"

namespace abb_hardware_module {

void FromProto(const abb::egm::EgmHeader& src,
               egm_data::EgmHeader* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmHeader& src,
             abb::egm::EgmHeader* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmCartesian& src,
               egm_data::EgmCartesian* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmCartesian& src,
             abb::egm::EgmCartesian* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmQuaternion& src,
               egm_data::EgmQuaternion* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmQuaternion& src,
             abb::egm::EgmQuaternion* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmEuler& src,
               egm_data::EgmEuler* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmEuler& src,
             abb::egm::EgmEuler* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmClock& src,
               egm_data::EgmClock* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmClock& src,
             abb::egm::EgmClock* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmPose& src,
               egm_data::EgmPose* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmPose& src,
             abb::egm::EgmPose* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmCartesianSpeed& src,
               egm_data::EgmCartesianSpeed* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmCartesianSpeed& src,
             abb::egm::EgmCartesianSpeed* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmJoints& src,
               egm_data::EgmJoints* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmJoints& src,
             abb::egm::EgmJoints* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmPlanned& src,
               egm_data::EgmPlanned* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmPlanned& src,
             abb::egm::EgmPlanned* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmSpeedRef& src,
               egm_data::EgmSpeedRef* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmSpeedRef& src,
             abb::egm::EgmSpeedRef* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmPathCorr& src,
               egm_data::EgmPathCorr* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmPathCorr& src,
             abb::egm::EgmPathCorr* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmFeedBack& src,
               egm_data::EgmFeedBack* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmFeedBack& src,
             abb::egm::EgmFeedBack* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmMotorState& src,
               egm_data::EgmMotorState* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmMotorState& src,
             abb::egm::EgmMotorState* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmMCIState& src,
               egm_data::EgmMCIState* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmMCIState& src,
             abb::egm::EgmMCIState* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmRapidCtrlExecState& src,
               egm_data::EgmRapidCtrlExecState* dst)
    INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmRapidCtrlExecState& src,
             abb::egm::EgmRapidCtrlExecState* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmTestSignals& src,
               egm_data::EgmTestSignals* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmTestSignals& src,
             abb::egm::EgmTestSignals* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmMeasuredForce& src,
               egm_data::EgmMeasuredForce* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmMeasuredForce& src,
             abb::egm::EgmMeasuredForce* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmCollisionInfo& src,
               egm_data::EgmCollisionInfo* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmCollisionInfo& src,
             abb::egm::EgmCollisionInfo* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmRAPIDdata& src,
               egm_data::EgmRAPIDdata* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmRAPIDdata& src,
             abb::egm::EgmRAPIDdata* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmRobot& src,
               egm_data::EgmRobot* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmRobot& src,
             abb::egm::EgmRobot* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmSensor& src,
               egm_data::EgmSensor* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmSensor& src,
             abb::egm::EgmSensor* dst) INTRINSIC_NON_REALTIME_ONLY;

void FromProto(const abb::egm::EgmSensorPathCorr& src,
               egm_data::EgmSensorPathCorr* dst) INTRINSIC_NON_REALTIME_ONLY;
void ToProto(const egm_data::EgmSensorPathCorr& src,
             abb::egm::EgmSensorPathCorr* dst) INTRINSIC_NON_REALTIME_ONLY;

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERIALIZATION_H_

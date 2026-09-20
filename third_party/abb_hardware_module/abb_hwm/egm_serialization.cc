#include "third_party/abb_hardware_module/abb_hwm/egm_serialization.h"

#include <cstddef>

#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"

namespace abb_hardware_module {
void FromProto(const abb::egm::EgmHeader& src, egm_data::EgmHeader* dst) {
  if (src.has_seqno()) {
    dst->seqno = src.seqno();
  }
  if (src.has_tm()) {
    dst->tm = src.tm();
  }
  if (src.has_mtype()) {
    dst->mtype = static_cast<egm_data::EgmHeader::MessageType>(src.mtype());
  }
}

void ToProto(const egm_data::EgmHeader& src, abb::egm::EgmHeader* dst) {
  if (src.seqno.has_value()) {
    dst->set_seqno(src.seqno.value());
  }
  if (src.tm.has_value()) {
    dst->set_tm(src.tm.value());
  }
  if (src.mtype.has_value()) {
    dst->set_mtype(abb::egm::EgmHeader_MessageType(src.mtype.value()));
  }
}

void FromProto(const abb::egm::EgmCartesian& src, egm_data::EgmCartesian* dst) {
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}

void ToProto(const egm_data::EgmCartesian& src, abb::egm::EgmCartesian* dst) {
  dst->set_x(src.x);
  dst->set_y(src.y);
  dst->set_z(src.z);
}

void FromProto(const abb::egm::EgmQuaternion& src,
               egm_data::EgmQuaternion* dst) {
  dst->u0 = src.u0();
  dst->u1 = src.u1();
  dst->u2 = src.u2();
  dst->u3 = src.u3();
}

void ToProto(const egm_data::EgmQuaternion& src, abb::egm::EgmQuaternion* dst) {
  dst->set_u0(src.u0);
  dst->set_u1(src.u1);
  dst->set_u2(src.u2);
  dst->set_u3(src.u3);
}

void FromProto(const abb::egm::EgmEuler& src, egm_data::EgmEuler* dst) {
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}

void ToProto(const egm_data::EgmEuler& src, abb::egm::EgmEuler* dst) {
  dst->set_x(src.x);
  dst->set_y(src.y);
  dst->set_z(src.z);
}

void FromProto(const abb::egm::EgmClock& src, egm_data::EgmClock* dst) {
  dst->sec = src.sec();
  dst->usec = src.usec();
}

void ToProto(const egm_data::EgmClock& src, abb::egm::EgmClock* dst) {
  dst->set_sec(src.sec);
  dst->set_usec(src.usec);
}

void FromProto(const abb::egm::EgmPose& src, egm_data::EgmPose* dst) {
  if (src.has_pos()) {
    egm_data::EgmCartesian pos;
    FromProto(src.pos(), &pos);
    dst->pos = pos;
  }
  if (src.has_orient()) {
    egm_data::EgmQuaternion orient;
    FromProto(src.orient(), &orient);
    dst->orient = orient;
  }
  if (src.has_euler()) {
    egm_data::EgmEuler euler;
    FromProto(src.euler(), &euler);
    dst->euler = euler;
  }
}

void ToProto(const egm_data::EgmPose& src, abb::egm::EgmPose* dst) {
  if (src.pos.has_value()) {
    ToProto(src.pos.value(), dst->mutable_pos());
  }
  if (src.orient.has_value()) {
    ToProto(src.orient.value(), dst->mutable_orient());
  }
  if (src.euler.has_value()) {
    ToProto(src.euler.value(), dst->mutable_euler());
  }
}

void FromProto(const abb::egm::EgmCartesianSpeed& src,
               egm_data::EgmCartesianSpeed* dst) {
  for (size_t k = 0; k < 6; ++k) {
    dst->value(k) = src.value(k);
  }
}

void ToProto(const egm_data::EgmCartesianSpeed& src,
             abb::egm::EgmCartesianSpeed* dst) {
  for (auto& value : src.value) {
    dst->add_value(value);
  }
}

void FromProto(const abb::egm::EgmJoints& src, egm_data::EgmJoints* dst) {
  dst->joints.resize(src.joints_size());
  for (size_t k = 0; k < src.joints_size(); ++k) {
    dst->joints(k) = src.joints(k);
  }
}

void ToProto(const egm_data::EgmJoints& src, abb::egm::EgmJoints* dst) {
  for (auto& joint : src.joints) {
    dst->add_joints(joint);
  }
}

void FromProto(const abb::egm::EgmPlanned& src, egm_data::EgmPlanned* dst) {
  if (src.has_joints()) {
    egm_data::EgmJoints joints;
    FromProto(src.joints(), &joints);
    dst->joints = joints;
  }
  if (src.has_cartesian()) {
    egm_data::EgmPose cartesian;
    FromProto(src.cartesian(), &cartesian);
    dst->cartesian = cartesian;
  }
  if (src.has_externaljoints()) {
    egm_data::EgmJoints externalJoints;
    FromProto(src.externaljoints(), &externalJoints);
    dst->externalJoints = externalJoints;
  }
  if (src.has_time()) {
    egm_data::EgmClock time;
    FromProto(src.time(), &time);
    dst->time = time;
  }
}

void ToProto(const egm_data::EgmPlanned& src, abb::egm::EgmPlanned* dst) {
  if (src.joints.has_value()) {
    ToProto(src.joints.value(), dst->mutable_joints());
  }
  if (src.cartesian.has_value()) {
    ToProto(src.cartesian.value(), dst->mutable_cartesian());
  }
  if (src.externalJoints.has_value()) {
    ToProto(src.externalJoints.value(), dst->mutable_externaljoints());
  }
  if (src.time.has_value()) {
    ToProto(src.time.value(), dst->mutable_time());
  }
}

void FromProto(const abb::egm::EgmSpeedRef& src, egm_data::EgmSpeedRef* dst) {
  if (src.has_joints()) {
    egm_data::EgmJoints joints;
    FromProto(src.joints(), &joints);
    dst->joints = joints;
  }
  if (src.has_cartesians()) {
    egm_data::EgmCartesianSpeed cartesians;
    FromProto(src.cartesians(), &cartesians);
    dst->cartesians = cartesians;
  }
  if (src.has_externaljoints()) {
    egm_data::EgmJoints externalJoints;
    FromProto(src.externaljoints(), &externalJoints);
    dst->externalJoints = externalJoints;
  }
}

void ToProto(const egm_data::EgmSpeedRef& src, abb::egm::EgmSpeedRef* dst) {
  if (src.joints.has_value()) {
    ToProto(src.joints.value(), dst->mutable_joints());
  }
  if (src.cartesians.has_value()) {
    ToProto(src.cartesians.value(), dst->mutable_cartesians());
  }
  if (src.externalJoints.has_value()) {
    ToProto(src.externalJoints.value(), dst->mutable_externaljoints());
  }
}

void FromProto(const abb::egm::EgmPathCorr& src, egm_data::EgmPathCorr* dst) {
  if (src.has_pos()) {
    egm_data::EgmCartesian pos;
    FromProto(src.pos(), &pos);
    dst->pos = pos;
  }
  if (src.has_age()) {
    dst->age = src.age();
  }
}

void ToProto(const egm_data::EgmPathCorr& src, abb::egm::EgmPathCorr* dst) {
  ToProto(src.pos, dst->mutable_pos());
  dst->set_age(src.age);
}

void FromProto(const abb::egm::EgmFeedBack& src, egm_data::EgmFeedBack* dst) {
  if (src.has_joints()) {
    egm_data::EgmJoints joints;
    FromProto(src.joints(), &joints);
    dst->joints = joints;
  }
  if (src.has_cartesian()) {
    egm_data::EgmPose cartesian;
    FromProto(src.cartesian(), &cartesian);
    dst->cartesian = cartesian;
  }
  if (src.has_externaljoints()) {
    egm_data::EgmJoints externalJoints;
    FromProto(src.externaljoints(), &externalJoints);
    dst->externalJoints = externalJoints;
  }
  if (src.has_time()) {
    egm_data::EgmClock time;
    FromProto(src.time(), &time);
    dst->time = time;
  }
}

void ToProto(const egm_data::EgmFeedBack& src, abb::egm::EgmFeedBack* dst) {
  if (src.joints.has_value()) {
    ToProto(src.joints.value(), dst->mutable_joints());
  }
  if (src.cartesian.has_value()) {
    ToProto(src.cartesian.value(), dst->mutable_cartesian());
  }
  if (src.externalJoints.has_value()) {
    ToProto(src.externalJoints.value(), dst->mutable_externaljoints());
  }
  if (src.time.has_value()) {
    ToProto(src.time.value(), dst->mutable_time());
  }
}

void FromProto(const abb::egm::EgmMotorState& src,
               egm_data::EgmMotorState* dst) {
  dst->state =
      static_cast<egm_data::EgmMotorState::MotorStateType>(src.state());
}

void ToProto(const egm_data::EgmMotorState& src, abb::egm::EgmMotorState* dst) {
  dst->set_state(
      static_cast<abb::egm::EgmMotorState_MotorStateType>(src.state));
}

void FromProto(const abb::egm::EgmMCIState& src, egm_data::EgmMCIState* dst) {
  dst->state = static_cast<egm_data::EgmMCIState::MCIStateType>(src.state());
}

void ToProto(const egm_data::EgmMCIState& src, abb::egm::EgmMCIState* dst) {
  dst->set_state(static_cast<abb::egm::EgmMCIState_MCIStateType>(src.state));
}

void FromProto(const abb::egm::EgmRapidCtrlExecState& src,
               egm_data::EgmRapidCtrlExecState* dst) {
  dst->state =
      static_cast<egm_data::EgmRapidCtrlExecState::RapidCtrlExecStateType>(
          src.state());
}

void ToProto(const egm_data::EgmRapidCtrlExecState& src,
             abb::egm::EgmRapidCtrlExecState* dst) {
  dst->set_state(
      static_cast<abb::egm::EgmRapidCtrlExecState_RapidCtrlExecStateType>(
          src.state));
}

void FromProto(const abb::egm::EgmTestSignals& src,
               egm_data::EgmTestSignals* dst) {
  dst->signals.resize(src.signals_size());
  for (size_t k = 0; k < src.signals_size(); ++k) {
    dst->signals(k) = src.signals(k);
  }
}

void ToProto(const egm_data::EgmTestSignals& src,
             abb::egm::EgmTestSignals* dst) {
  dst->clear_signals();
  for (auto& signal : src.signals) {
    dst->add_signals(signal);
  }
}

void FromProto(const abb::egm::EgmMeasuredForce& src,
               egm_data::EgmMeasuredForce* dst) {
  if (src.has_fcactive()) {
    dst->fcActive = src.fcactive();
  }
  for (size_t k = 0; k < 6; ++k) {
    dst->force(k) = src.force(k);
  }
}

void ToProto(const egm_data::EgmMeasuredForce& src,
             abb::egm::EgmMeasuredForce* dst) {
  if (src.fcActive.has_value()) {
    dst->set_fcactive(src.fcActive.value());
  }
  dst->clear_force();
  for (auto& force : src.force) {
    dst->add_force(force);
  }
}

void FromProto(const abb::egm::EgmCollisionInfo& src,
               egm_data::EgmCollisionInfo* dst) {
  if (src.has_collsiontriggered()) {
    dst->collisionTriggered = src.collsiontriggered();
  }
  dst->collDetQuota.resize(src.colldetquota_size());
  for (size_t k = 0; k < src.colldetquota_size(); ++k) {
    dst->collDetQuota(k) = src.colldetquota(k);
  }
}

void ToProto(const egm_data::EgmCollisionInfo& src,
             abb::egm::EgmCollisionInfo* dst) {
  if (src.collisionTriggered.has_value()) {
    dst->set_collsiontriggered(src.collisionTriggered.value());
  }
  dst->clear_colldetquota();
  for (auto& quota : src.collDetQuota) {
    dst->add_colldetquota(quota);
  }
}

void FromProto(const abb::egm::EgmRAPIDdata& src, egm_data::EgmRAPIDdata* dst) {
  if (src.has_digval()) {
    dst->digVal = src.digval();
  }
  dst->dnum.resize(src.dnum_size());
  for (size_t k = 0; k < src.dnum_size(); ++k) {
    dst->dnum(k) = src.dnum(k);
  }
}

void ToProto(const egm_data::EgmRAPIDdata& src, abb::egm::EgmRAPIDdata* dst) {
  if (src.digVal.has_value()) {
    dst->set_digval(src.digVal.value());
  }
  dst->clear_dnum();
  for (auto& num : src.dnum) {
    dst->add_dnum(num);
  }
}

void FromProto(const abb::egm::EgmRobot& src, egm_data::EgmRobot* dst) {
  if (src.has_header()) {
    egm_data::EgmHeader header;
    FromProto(src.header(), &header);
    dst->header = header;
  }
  if (src.has_feedback()) {
    egm_data::EgmFeedBack feedback;
    FromProto(src.feedback(), &feedback);
    dst->feedBack = feedback;
  }
  if (src.has_planned()) {
    egm_data::EgmPlanned planned;
    FromProto(src.planned(), &planned);
    dst->planned = planned;
  }
  if (src.has_motorstate()) {
    egm_data::EgmMotorState motorstate;
    FromProto(src.motorstate(), &motorstate);
    dst->motorState = motorstate;
  }
  if (src.has_mcistate()) {
    egm_data::EgmMCIState mcistate;
    FromProto(src.mcistate(), &mcistate);
    dst->mciState = mcistate;
  }
  if (src.has_mciconvergencemet()) {
    dst->mciConvergenceMet = src.mciconvergencemet();
  }
  if (src.has_testsignals()) {
    egm_data::EgmTestSignals testsignals;
    FromProto(src.testsignals(), &testsignals);
    dst->testSignals = testsignals;
  }
  if (src.has_rapidexecstate()) {
    egm_data::EgmRapidCtrlExecState rapidctrlexecstate;
    FromProto(src.rapidexecstate(), &rapidctrlexecstate);
    dst->rapidExecState = rapidctrlexecstate;
  }
  if (src.has_measuredforce()) {
    egm_data::EgmMeasuredForce measuredforce;
    FromProto(src.measuredforce(), &measuredforce);
    dst->measuredForce = measuredforce;
  }
  if (src.has_utilizationrate()) {
    dst->utilizationRate = src.utilizationrate();
  }
  if (src.has_moveindex()) {
    dst->moveIndex = src.moveindex();
  }
  if (src.has_collisioninfo()) {
    egm_data::EgmCollisionInfo collisionInfo;
    FromProto(src.collisioninfo(), &collisionInfo);
    dst->CollisionInfo = collisionInfo;
  }

  if (src.has_rapidfromrobot()) {
    egm_data::EgmRAPIDdata rapiddata;
    FromProto(src.rapidfromrobot(), &rapiddata);
    dst->RAPIDfromRobot = rapiddata;
  }
}

void ToProto(const egm_data::EgmRobot& src, abb::egm::EgmRobot* dst) {
  if (src.header.has_value()) {
    ToProto(src.header.value(), dst->mutable_header());
  }
  if (src.feedBack.has_value()) {
    ToProto(src.feedBack.value(), dst->mutable_feedback());
  }
  if (src.planned.has_value()) {
    ToProto(src.planned.value(), dst->mutable_planned());
  }
  if (src.motorState.has_value()) {
    ToProto(src.motorState.value(), dst->mutable_motorstate());
  }
  if (src.mciState.has_value()) {
    ToProto(src.mciState.value(), dst->mutable_mcistate());
  }
  if (src.mciConvergenceMet.has_value()) {
    dst->set_mciconvergencemet(src.mciConvergenceMet.value());
  }
  if (src.testSignals.has_value()) {
    ToProto(src.testSignals.value(), dst->mutable_testsignals());
  }
  if (src.rapidExecState.has_value()) {
    ToProto(src.rapidExecState.value(), dst->mutable_rapidexecstate());
  }
  if (src.measuredForce.has_value()) {
    ToProto(src.measuredForce.value(), dst->mutable_measuredforce());
  }
  if (src.utilizationRate.has_value()) {
    dst->set_utilizationrate(src.utilizationRate.value());
  }
  if (src.moveIndex.has_value()) {
    dst->set_moveindex(src.moveIndex.value());
  }
  if (src.CollisionInfo.has_value()) {
    ToProto(src.CollisionInfo.value(), dst->mutable_collisioninfo());
  }
  if (src.RAPIDfromRobot.has_value()) {
    ToProto(src.RAPIDfromRobot.value(), dst->mutable_rapidfromrobot());
  }
}

void FromProto(const abb::egm::EgmSensor& src, egm_data::EgmSensor* dst) {
  if (src.has_header()) {
    egm_data::EgmHeader header;
    FromProto(src.header(), &header);
    dst->header = header;
  }
  if (src.has_planned()) {
    egm_data::EgmPlanned planned;
    FromProto(src.planned(), &planned);
    dst->planned = planned;
  }
  if (src.has_speedref()) {
    egm_data::EgmSpeedRef speedref;
    FromProto(src.speedref(), &speedref);
    dst->speedRef = speedref;
  }
  if (src.has_rapidtorobot()) {
    egm_data::EgmRAPIDdata rapidtorobot;
    FromProto(src.rapidtorobot(), &rapidtorobot);
    dst->RAPIDtoRobot = rapidtorobot;
  }
}

void ToProto(const egm_data::EgmSensor& src, abb::egm::EgmSensor* dst) {
  if (src.header.has_value()) {
    ToProto(src.header.value(), dst->mutable_header());
  }
  if (src.planned.has_value()) {
    ToProto(src.planned.value(), dst->mutable_planned());
  }
  if (src.speedRef.has_value()) {
    ToProto(src.speedRef.value(), dst->mutable_speedref());
  }
  if (src.RAPIDtoRobot.has_value()) {
    ToProto(src.RAPIDtoRobot.value(), dst->mutable_rapidtorobot());
  }
}

void FromProto(const abb::egm::EgmSensorPathCorr& src,
               egm_data::EgmSensorPathCorr* dst) {
  if (src.has_header()) {
    egm_data::EgmHeader header;
    FromProto(src.header(), &header);
    dst->header = header;
  }
  if (src.has_pathcorr()) {
    egm_data::EgmPathCorr pathcorr;
    FromProto(src.pathcorr(), &pathcorr);
    dst->pathCorr = pathcorr;
  }
}
}  // namespace abb_hardware_module

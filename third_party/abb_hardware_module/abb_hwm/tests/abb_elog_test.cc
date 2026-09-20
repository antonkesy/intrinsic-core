#include "third_party/abb_hardware_module/abb_hwm/abb_elog.h"

#include <gtest/gtest.h>

#include "absl/strings/string_view.h"
#include "nlohmann/json.hpp"

using namespace abb_hardware_module;
using json = nlohmann::json;

namespace {

constexpr absl::string_view event_log_json =
    R"(
    [
            {
                "_links": {
                    "self": {
                        "href": "0/76?lang=en"
                    }
                },
                "_title": "/rw/elog/0/76",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "2",
                "argv": [
                    {
                        "type": "long",
                        "value": 2
                    },
                    {
                        "type": "string",
                        "value": "T_ROB1"
                    }
                ],
                "causes": "",
                "code": "10125",
                "conseqs": "",
                "desc": "The task T_ROB1 has stopped. The reason is that an external or internal stop has occurred.",
                "msgtype": "1",
                "title": "Program stopped",
                "tstamp": "2025-04-02 T 09:17:20"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/75?lang=en"
                    }
                },
                "_title": "/rw/elog/0/75",
                "_type": "elog-message-li",
                "actions": "Increase the maximum allowed correction speed to allow faster ramp-down, or increase the user defined ramp time.",
                "argc": "2",
                "argv": [
                    {
                        "type": "long",
                        "value": 0
                    },
                    {
                        "type": "float",
                        "value": 0
                    }
                ],
                "causes": "",
                "code": "50446",
                "conseqs": "The user defined ramp time was increased to avoid over-speed.",
                "desc": "External Motion Interface ramp time was increased in order to avoid over-speed when ramping down correction 0.New ramp time: 2.909833 seconds.",
                "msgtype": "2",
                "title": "External Motion Interface ramp time increased",
                "tstamp": "2025-04-02 T 09:17:20"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/74?lang=en"
                    }
                },
                "_title": "/rw/elog/0/74",
                "_type": "elog-message-li",
                "actions": "Check which emergency stop device caused the stop.",
                "argc": "0",
                "causes": "Any emergency stop device connected to the emergency stop input have been opened. These may be internal (on the controller or on the FlexPendant) or external (devices connected by the system builder). The internal devices are shown in the Circuit Diagram.",
                "code": "10013",
                "conseqs": "All program execution and thus robot actions are immediately halted. The robot axes are meanwhile held in position by mechanical holding brakes.",
                "desc": "The system is in the Emergency stop state, since the Motors ON circuit has been opened by an Emergency Stop device.",
                "msgtype": "1",
                "title": "Emergency stop state",
                "tstamp": "2025-04-02 T 09:17:20"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/73?lang=en"
                    }
                },
                "_title": "/rw/elog/0/73",
                "_type": "elog-message-li",
                "actions": "Deactivate the emergency stop and restart the program.",
                "argc": "1",
                "argv": [
                    {
                        "type": "string",
                        "value": "Local Emergency Stop"
                    }
                ],
                "causes": "An emergency stop request has been received by the safety controller.",
                "code": "90518",
                "conseqs": "The safety controller will stop all robot movements.",
                "desc": "The Emergency Stop, Local Emergency Stop, has been triggered in the safety controller.",
                "msgtype": "3",
                "title": "Safety Controller Emergency Stop triggered",
                "tstamp": "2025-04-02 T 09:17:20"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/70?lang=en"
                    }
                },
                "_title": "/rw/elog/0/70",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "2",
                "argv": [
                    {
                        "type": "long",
                        "value": 2
                    },
                    {
                        "type": "string",
                        "value": "T_ROB1"
                    }
                ],
                "causes": "",
                "code": "10151",
                "conseqs": "",
                "desc": "Execution of task T_ROB1 has been started from the first instruction of the task&apos;s entry routine. The originator is an external client.",
                "msgtype": "1",
                "title": "Program started",
                "tstamp": "2025-04-02 T 09:13:52"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/69?lang=en"
                    }
                },
                "_title": "/rw/elog/0/69",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "1",
                "argv": [
                    {
                        "type": "long",
                        "value": 0
                    }
                ],
                "causes": "",
                "code": "10053",
                "conseqs": "",
                "desc": "The regain movement is ready.",
                "msgtype": "1",
                "title": "Regain ready",
                "tstamp": "2025-04-02 T 09:13:52"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/68?lang=en"
                    }
                },
                "_title": "/rw/elog/0/68",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "1",
                "argv": [
                    {
                        "type": "long",
                        "value": 0
                    }
                ],
                "causes": "",
                "code": "10052",
                "conseqs": "",
                "desc": "A regain movement has started.",
                "msgtype": "1",
                "title": "Regain start",
                "tstamp": "2025-04-02 T 09:13:52"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/67?lang=en"
                    }
                },
                "_title": "/rw/elog/0/67",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "10011",
                "conseqs": "The Motors ON circuit has been closed, enabling power supply to the manipulator's motors. Normal operation may be resumed.",
                "desc": "The system is in the Motors ON state.",
                "msgtype": "1",
                "title": "Motors ON state",
                "tstamp": "2025-04-01 T 23:03:55"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/66?lang=en"
                    }
                },
                "_title": "/rw/elog/0/66",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "10010",
                "conseqs": "No operation will be possible until after closing the Motors ON circuit. The manipulator's axes are meanwhile held in position by mechanical holding brakes.",
                "desc": "The system is in the Motors OFF state. It enters this state either after switching from Manual mode to Automatic, or after the Motors ON circuit has been opened during program execution.",
                "msgtype": "1",
                "title": "Motors OFF state",
                "tstamp": "2025-04-01 T 23:03:22"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/65?lang=en"
                    }
                },
                "_title": "/rw/elog/0/65",
                "_type": "elog-message-li",
                "actions": "Check which safety device caused the stop.",
                "argc": "0",
                "causes": "Any safety device connected to the system's stop inputs have been opened. These are shown in the Circuit Diagram.",
                "code": "10012",
                "conseqs": "No operation will be possible until after closing the Motors ON circuit. The manipulator's axes are meanwhile held in position by mechanical holding brakes.",
                "desc": "The system is in the Guard stop state. It enters this state either after switching from Automatic- to Manual mode, or after the Motors ON circuit has been opened by an Emergency Stop or Automatic Stop, or in Manual mode if Enabling device was released.",
                "msgtype": "1",
                "title": "Safety guard stop state",
                "tstamp": "2025-04-01 T 23:03:22"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/64?lang=en"
                    }
                },
                "_title": "/rw/elog/0/64",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "10010",
                "conseqs": "No operation will be possible until after closing the Motors ON circuit. The manipulator's axes are meanwhile held in position by mechanical holding brakes.",
                "desc": "The system is in the Motors OFF state. It enters this state either after switching from Manual mode to Automatic, or after the Motors ON circuit has been opened during program execution.",
                "msgtype": "1",
                "title": "Motors OFF state",
                "tstamp": "2025-04-01 T 23:03:22"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/63?lang=en"
                    }
                },
                "_title": "/rw/elog/0/63",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "10011",
                "conseqs": "The Motors ON circuit has been closed, enabling power supply to the manipulator's motors. Normal operation may be resumed.",
                "desc": "The system is in the Motors ON state.",
                "msgtype": "1",
                "title": "Motors ON state",
                "tstamp": "2025-04-01 T 23:03:07"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/62?lang=en"
                    }
                },
                "_title": "/rw/elog/0/62",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "90526",
                "conseqs": "",
                "desc": "The active safety controller configuration has not been locked.",
                "msgtype": "2",
                "title": "Safety Controller Automatic Mode Warning",
                "tstamp": "2025-04-01 T 23:01:28"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/61?lang=en"
                    }
                },
                "_title": "/rw/elog/0/61",
                "_type": "elog-message-li",
                "actions": "",
                "argc": "0",
                "causes": "",
                "code": "10010",
                "conseqs": "No operation will be possible until after closing the Motors ON circuit. The manipulator's axes are meanwhile held in position by mechanical holding brakes.",
                "desc": "The system is in the Motors OFF state. It enters this state either after switching from Manual mode to Automatic, or after the Motors ON circuit has been opened during program execution.",
                "msgtype": "1",
                "title": "Motors OFF state",
                "tstamp": "2025-04-01 T 23:01:27"
            },
            {
                "_links": {
                    "self": {
                        "href": "0/60?lang=en"
                    }
                },
                "_title": "/rw/elog/0/60",
                "_type": "elog-message-li",
                "actions": "Check other elogs for more details.",
                "argc": "0",
                "causes": "",
                "code": "90867",
                "conseqs": "The system goes to Guard stop state.",
                "desc": "An attempt to go to Motors on was rejected by the Safety Controller.",
                "msgtype": "3",
                "title": "Safety Controller not ready",
                "tstamp": "2025-04-01 T 23:01:27"
            }
        ]
    )";

TEST(AbbElogTest, ParseAll) {
  // parse the test string and make sure the number of entries is 15
  json event_list = json::parse(event_log_json);
  auto entries_or = ParseAbbEventLog(event_list, AbbEventLogEntry::Level::INFO);
  ASSERT_TRUE(entries_or.ok());
  ASSERT_EQ(entries_or->size(), 15);
}

TEST(AbbElogTest, ParseWarningsAndErrors) {
  json event_list = json::parse(event_log_json);
  auto entries_or =
      ParseAbbEventLog(event_list, AbbEventLogEntry::Level::WARNING);
  ASSERT_TRUE(entries_or.ok());
  ASSERT_EQ(entries_or->size(), 4);
}

TEST(AbbElogTest, ParseErrors) {
  json event_list = json::parse(event_log_json);
  auto entries_or = ParseAbbEventLog(event_list);
  ASSERT_TRUE(entries_or.ok());
  ASSERT_EQ(entries_or->size(), 2);
}

}  // namespace

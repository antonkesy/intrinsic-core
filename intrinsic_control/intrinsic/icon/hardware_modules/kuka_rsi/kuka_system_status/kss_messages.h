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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KSS_MESSAGES_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KSS_MESSAGES_H_
// This file is generated with extract_kss_messages.py.

#include <cstddef>

#include "absl/strings/string_view.h"

namespace intrinsic {

constexpr size_t kNumKssMessages = 511;
constexpr size_t kNumKssColumns = 4;

constexpr absl::string_view kss_message_data[kNumKssMessages][kNumKssColumns] =
    {
        {
            // Manually added since missing in KUKA sources.
            "CrossMeld",
            "15047",
            "Mastering test required (internal)",
            "Acknowledgment",
        },
        {
            "VwInlineForms",
            "110",
            "Initialization macro Makro{MakroNumberSpecialRun} not available. "
            "Initialization not possible.",
            "Info",
        },
        {
            "CrossMeld",
            "2710",
            "{Module name} too many local subprograms",
            "Info",
        },
        {
            "CrossMeld",
            "2359",
            "This KRL statement must not be used in a spline block.",
            "Info",
        },
        {
            "CrossMeld",
            "1366",
            "Cartesian end point specification not possible",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26129",
            "Mastering notch dirty or damaged",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15045",
            "Error at mastering reference switch",
            "State",
        },
        {
            "CrossMeld",
            "10090",
            "{Bus instance name}: Station {Faulty module} reports "
            "configuration error.",
            "State",
        },
        {
            "CrossMeld",
            "10092",
            "{Bus instance name}: Station {Faulty module} signals diagnostics "
            "buffer overflow.",
            "State",
        },
        {
            "CrossMeld",
            "26132",
            "Error: Group brake error ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "15134",
            "Ackn: Safety stop before violation of monitoring space no. "
            "{Number of monitoring space}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2857",
            "System restored from Hibernate mode",
            "Info",
        },
        {
            "CrossMeld",
            "1137",
            "EMD mastering distance exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15065",
            "Level at mastering reference switch was unexpectedly Low",
            "Info",
        },
        {
            "CrossMeld",
            "1411",
            "Block selection outside buffer: next start deletes buffer",
            "Info",
        },
        {
            "CrossMeld",
            "26032",
            "Error: Overload error IxT ({Device type}) ({Number}).",
            "State",
        },
        {
            "EthernetKRL",
            "13",
            "Initialization of Ethernet parameters failed",
            "Error",
        },
        {
            "CrossMeld",
            "15014",
            "smartPAD connection error",
            "State",
        },
        {
            "CrossMeld",
            "10002",
            "{Instance name} System error: {Error code}, additional info: "
            "{Expanded error code}",
            "State",
        },
        {
            "CrossMeld",
            "1409",
            "Trigger not allowed in interrupt or *.SUB",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13073",
            "EIP I/O driver cannot determine the IP address of the KLI",
            "State",
        },
        {
            "CrossMeld",
            "26252",
            "Drive initialization error ({Device number}).",
            "State",
        },
        {
            "CrossMeld",
            "15052",
            "Ackn.: Mastering reference switch not actuated",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "114",
            "Workspace no. {Workspace number} violated.",
            "State",
        },
        {
            "CrossMeld",
            "81",
            "Robot stopped by submit",
            "State",
        },
        {
            "CrossMeld",
            "10004",
            "{Instance name} Bus error. Error location in progress.",
            "State",
        },
        {
            "CrossMeld",
            "10072",
            "Temperature sensor {Sensor name} ({Temperature} °C) out of range.",
            "State",
        },
        {
            "CrossMeld",
            "15049",
            "Mastering test failed",
            "State",
        },
        {
            "CrossMeld",
            "1355",
            "Enabling switch required",
            "Info",
        },
        {
            "CrossMeld",
            "27022",
            "T1 mode prevents brake test execution",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13068",
            "<{Bus instance}> EtherCAT device {Device name} is not connected "
            "to bus.",
            "State",
        },
        {
            "CrossMeld",
            "10006",
            "{Instance name} Error opening file {File name}",
            "Info",
        },
        {
            "CrossMeld",
            "13067",
            "EthernetIP stack cannot be set to Running state",
            "State",
        },
        {
            "CrossMeld",
            "2032",
            "Inadmissible command",
            "Info",
        },
        {
            "CrossMeld",
            "3158",
            "{Module name}: inadmissible module name.",
            "Info",
        },
        {
            "CrossMeld",
            "26035",
            "Error: Intermediate circuit voltage too high ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "13063",
            "Error reading {Name}",
            "State",
        },
        {
            "CrossMeld",
            "27019",
            "Full torque of brake {Brake}{Axis} could not be verified, but "
            "holding torque is reached",
            "Info",
        },
        {
            "CrossMeld",
            "10014",
            "{Instance name} Synchronization error",
            "State",
        },
        {
            "Simulate",
            "3",
            "Wait for {Expression}",
            "Wait",
        },
        {
            "CrossMeld",
            "27020",
            "Asynchronous axes prevent execution of brake test",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10013",
            "{Instance name} Error {Error code} switching on segment {Segment "
            "number}",
            "Info",
        },
        {
            "CrossMeld",
            "147",
            "Perform safe robot retraction in axis-specific workspace "
            "{Workspace number}",
            "State",
        },
        {
            "CrossMeld",
            "3070",
            "Suitable mam file not present. Error {Cause of error}",
            "Info",
        },
        {
            "CrossMeld",
            "13064",
            "EthernetIP stack cannot be set to Offline state",
            "State",
        },
        {
            "CrossMeld",
            "13016",
            "<{Bus ID}> missing reception of network frames [{Details}]",
            "State",
        },
        {
            "CrossMeld",
            "10061",
            "{Name of the instance} Configuration error, device number {Number "
            "of the device}",
            "State",
        },
        {
            "CrossMeld",
            "15048",
            "Ackn.: Mastering test time interval expired",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10026",
            "{Instance name} Version mismatch in the file {File name}, "
            "required version is {Required version}",
            "State",
        },
        {
            "CrossMeld",
            "426",
            "6D mouse deflected - move mouse to zero position, press enabling "
            "switch again",
            "State",
        },
        {
            "CrossMeld",
            "26107",
            "Warning: Intermediate circuit voltage too high ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "15034",
            "Ackn.: More than one tool activated in the safety controller",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "2048",
            "Server time limit reached",
            "Error",
        },
        {
            "TPBASIS",
            "104",
            "TouchUp not possible. Change of orientation {0} out of range.",
            "Error",
        },
        {
            "CrossMeld",
            "1347",
            "Robot not mastered",
            "InfoOrAcknowledgment",
        },
        {
            "EthernetKRL",
            "2560",
            "Fatal error",
            "Error",
        },
        {
            "CrossMeld",
            "26051",
            "Error: Overload ballast circuit ({Device type}) ({Number}).",
            "State",
        },
        {
            "EthernetKRL",
            "4",
            "Requested function not implemented",
            "Error",
        },
        {
            "CrossMeld",
            "10043",
            "Timeout establishing connection to EIP device {Name}",
            "State",
        },
        {
            "CrossMeld",
            "10046",
            "Timeout establishing connection between PLC and {Name}",
            "State",
        },
        {
            "CrossMeld",
            "13066",
            "EthernetIP stack cannot be set to Online state",
            "State",
        },
        {
            "TechHandler",
            "21",
            "Touch-up not possible. Cartesian distance out of range.",
            "Error",
        },
        {
            "CrossMeld",
            "112",
            "Invalid $TOOL: workspace monitoring not possible",
            "State",
        },
        {
            "EthernetKRL",
            "1024",
            "Error while reading received XML data",
            "Error",
        },
        {
            "MadaConfiguration",
            "7",
            "There are no supply servo parameters for {0} with a resistor "
            "strength of {1}",
            "Error",
        },
        {
            "CrossMeld",
            "2821",
            "$ORI_TYPE implicitly set to #VAR",
            "Info",
        },
        {
            "CrossMeld",
            "6503",
            "I/O driver {Driver name} configuration error",
            "State",
        },
        {
            "CrossMeld",
            "26013",
            "General servo error ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "1447",
            "Software limit switch point cannot be reached {Motion "
            "direction}{Axis number}",
            "InfoOrAcknowledgment",
        },
        {
            "Cross3Archive",
            "61",
            "Archiving to \"{0}\" not configured!",
            "Info",
        },
        {
            "CrossMeld",
            "3217",
            "Braking distance for CONST_VEL END in motion in {Module name@line "
            "number[:point name] of the limiting segment} is too short. "
            "Instead of $VEL.CP = {Programmed velocity} m/s, only {Velocity "
            "reached} m/s is reached.",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "3238",
            "Energy measurement cannot currently be stopped",
            "Info",
        },
        {
            "CrossMeld",
            "26036",
            "Error: Intermediate circuit voltage too low ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "10035",
            "{Instance name} Error on starting Interbus (firmware service: "
            "{Number of the firmware service} ErrCode: {Error code of the "
            "firmware service})",
            "State",
        },
        {
            "CrossMeld",
            "26271",
            "Ackn. Charging failed (power supply {Power supply})",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15109",
            "Level at mastering confirmation input is unexpectedly high",
            "State",
        },
        {
            "CrossMeld",
            "1212",
            "Ackn. Operator safety / safety fence closed",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26034",
            "Error: Overcurrent ({Device type}) ({Number}) ({Error code}).",
            "State",
        },
        {
            "CrossMeld",
            "26047",
            "Error: Mains power failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "205",
            "Software limit switch {Motion direction} {Axis number}",
            "State",
        },
        {
            "CrossMeld",
            "3237",
            "An energy measurement is already active.",
            "Info",
        },
        {
            "CrossMeld",
            "3193",
            "Ackn. Robot stopped by submit",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2074",
            "Ascending indices expected",
            "Info",
        },
        {
            "KRmsgNET",
            "12",
            "Configuration invalid!",
            "Info",
        },
        {
            "CrossMeld",
            "2106",
            "Block must not be deleted",
            "Info",
        },
        {
            "CrossMeld",
            "309",
            "Block selection: BCO run required in T1/T2",
            "State",
        },
        {
            "CrossMeld",
            "1039",
            "Backward motion not possible: trace already executed",
            "Info",
        },
        {
            "CrossMeld",
            "29004",
            "Internal RSI error",
            "State",
        },
        {
            "CrossMeld",
            "127",
            "{Axis number} asynchronous external axis",
            "State",
        },
        {
            "CrossMeld",
            "15117",
            "External safety stop 2",
            "State",
        },
        {
            "CrossMeld",
            "2013",
            "Unused",
            "Info",
        },
        {
            "CrossMeld",
            "27018",
            "No application of brake {Brake}{Axis} observed",
            "Info",
        },
        {
            "CrossMeld",
            "10030",
            "{Instance name} Loading of SVC file aborted due to a formatting "
            "error",
            "Info",
        },
        {
            "CrossMeld",
            "15113",
            "Mastering without reference request confirmed",
            "State",
        },
        {
            "CrossMeld",
            "2047",
            "Object is not available",
            "Info",
        },
        {
            "CrossMeld",
            "3331",
            "Backward motion with modified Motion mode not permissible",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2829",
            "OriJoint not possible: configuration of start and end points "
            "different",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1200",
            "Ackn. EMERGENCY STOP",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "3",
            "Message buffer overflow",
            "State",
        },
        {
            "CrossMeld",
            "26042",
            "Error: Communication error ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "10052",
            "{Bus instance name}: DeviceNet slave configuration error ({Reason "
            "1}{Reason 2})",
            "State",
        },
        {
            "CrossMeld",
            "1008",
            "Cold start of controller",
            "Info",
        },
        {
            "CrossMeld",
            "201",
            "Enabling switch released",
            "State",
        },
        {
            "EthernetKRL",
            "2816",
            "UDP packet contains too much data.",
            "Error",
        },
        {
            "CrossMeld",
            "10011",
            "{Instance name} error in slave circuit {Error code}, additional "
            "info {Expanded error code}",
            "Info",
        },
        {
            "CrossMeld",
            "26106",
            "Warning: Overcurrent ({Device type}) ({Number}) ({Error code}).",
            "State",
        },
        {
            "CrossMeld",
            "15058",
            "Controller in failsafe state.",
            "State",
        },
        {
            "CrossMeld",
            "1007",
            "Cannot open channel {Name of the channel}",
            "Info",
        },
        {
            "CrossMeld",
            "15135",
            "Ackn.: Safety stop before leaving cell area.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1349",
            "Robot mastered",
            "Info",
        },
        {
            "CrossMeld",
            "1446",
            "Value assignment inadmissible: {Assignment} {ModuleLine}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "1458",
            "Auxiliary point identical to end point",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "470",
            "Safe robot override reduction active",
            "State",
        },
        {
            "CrossMeld",
            "13036",
            "Enabling switch timeout. Release enabling switch and press it "
            "again.",
            "State",
        },
        {
            "CrossMeld",
            "1358",
            "Selection required",
            "Info",
        },
        {
            "CrossMeld",
            "2038",
            "Declaration not in declaration section",
            "Info",
        },
        {
            "CrossMeld",
            "15101",
            "Safe device {Device}: Error at input {Input}",
            "State",
        },
        {
            "CrossMeld",
            "2296",
            "Not a name of a constant of this type",
            "Info",
        },
        {
            "EthernetKRL",
            "18",
            "Send data failed",
            "Error",
        },
        {
            "CrossMeld",
            "68",
            "No motion enable present",
            "State",
        },
        {
            "CrossMeld",
            "1072",
            "Brake holding torque {Axis number} exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "264",
            "{Path} correction data loader aborted",
            "State",
        },
        {
            "CrossMeld",
            "1342",
            "Workspace error",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "320",
            "Axes are simulated {Bit mask of simulated axes}",
            "State",
        },
        {
            "CrossMeld",
            "13018",
            "<{Bus ID}> Error during ECat stack initialization. Device: "
            "{Faulty device} [{Additional information}]",
            "State",
        },
        {
            "CrossMeld",
            "2998",
            "Asynchronous motion active",
            "Info",
        },
        {
            "CrossMeld",
            "72",
            "RESUME inadmissible for global interrupts",
            "State",
        },
        {
            "CrossMeld",
            "13012",
            "<{Bus ID}> Error during ECat stack initialization [{Reason} "
            "{Reason 2}]",
            "State",
        },
        {
            "CrossMeld",
            "3330",
            "Assignment to $MOTION_MODE invalid, reason: {Reason} active "
            "{ModuleLine}",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "7",
            "Writing of data to send failed",
            "Error",
        },
        {
            "CrossMeld",
            "10047",
            "Connection between PLC and {Name} terminated.",
            "State",
        },
        {
            "CrossMeld",
            "12040",
            "Position comparison robot controller <-> safety controller not "
            "possible",
            "State",
        },
        {
            "CrossMeld",
            "10096",
            "{Bus instance name}: Station {Faulty module} reports insufficient "
            "resources to process PROFIBUS telegram",
            "State",
        },
        {
            "CrossMeld",
            "26039",
            "Error: Device temperature is too high ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "3266",
            "Acknowledge to resume T2 operation with the set velocity.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13038",
            "Profinet device stack cannot be started, error code: {Code}",
            "State",
        },
        {
            "CrossMeld",
            "26016",
            "Ackn. Position controller in limit ({Drive}).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "14001",
            "Ackn. Drive bus power-down",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2858",
            "Ackn. Stop due to field bus error",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1362",
            "STOP due to operating mode change",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1516",
            "Reference system not programmed",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "2295",
            "Array components cannot be indexed",
            "Info",
        },
        {
            "CrossMeld",
            "3122",
            "$CIRC_MODE incorrectly programmed (rule {Reason} violated).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2807",
            "Deadlock detected; local program is waiting for '{Workspace "
            "name}' from program '{Remote program}' ({Remote KRC})",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2275",
            "Current/formal parameter types incompatible",
            "Info",
        },
        {
            "CrossMeld",
            "276",
            "Machine data do not match robot type",
            "State",
        },
        {
            "CrossMeld",
            "27014",
            "Invalid configuration data for brake {Brake}{Axis} (error code "
            "{Error code})",
            "Info",
        },
        {
            "CrossMeld",
            "1609",
            "Assignment of runtime data to $CYCFLAG inadmissible {ModuleLine}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "1457",
            "Start point identical to auxiliary point",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2937",
            "$CP_STATMON: approximate positioning not possible",
            "Info",
        },
        {
            "CrossMeld",
            "27017",
            "Friction impedes verification of brake {Brake}{Axis}",
            "Info",
        },
        {
            "CrossMeld",
            "2923",
            "Statement not allowed in spline block",
            "Info",
        },
        {
            "CrossMeld",
            "27021",
            "Simulated axes prevent execution of brake test",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26007",
            "Permitted actual velocity exceeded ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "26050",
            "Error: Fault in brake resistor ({Device type}) ({Number}).",
            "State",
        },
        {
            "RobotData",
            "31",
            "No connection to RDC possible (bus error, etc.)",
            "Info",
        },
        {
            "CrossMeld",
            "1543",
            "All cyclical analog inputs assigned",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13080",
            "<{Bus instance}> Ethercat device: '{Device name}' cannot be "
            "started. {Reason}",
            "State",
        },
        {
            "Cross3Archive",
            "69",
            "Restoration canceled! The following technology packages are "
            "missing in the archive({0}) or on the controller ({1})!",
            "Info",
        },
        {
            "EthernetKRL",
            "15",
            "Access to empty element memory",
            "Error",
        },
        {
            "CrossMeld",
            "13020",
            "<{Bus ID}> Ethercat bus error. Device: {Details} [{Details}]",
            "State",
        },
        {
            "CrossMeld",
            "13025",
            "6D mouse deflected - move mouse to zero position, press enabling "
            "switch again.",
            "State",
        },
        {
            "CrossMeld",
            "10017",
            "{Instance name} Error in status register {Status register "
            "diagnosis}, parameter register {Parameter register diagnosis}",
            "State",
        },
        {
            "CrossMeld",
            "26217",
            "Ackn. brake cool off time ({Drive}).",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "17",
            "Assembly of data to send failed",
            "Error",
        },
        {
            "CrossMeld",
            "15108",
            "Error was at the mastering confirmation input",
            "Info",
        },
        {
            "CrossMeld",
            "3199",
            "SETINFO on {KRL variable name} ({Declaration location}) suspended",
            "State",
        },
        {
            "CrossMeld",
            "2871",
            "Workspace request for '{Workspace name}' failed",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "6",
            "Interpretation of configuration failed",
            "Error",
        },
        {
            "CrossMeld",
            "29009",
            "Configuration file of the RSI I/Os is invalid.",
            "State",
        },
        {
            "CrossMeld",
            "2052",
            "Specified array name inadmissible here",
            "Info",
        },
        {
            "CrossMeld",
            "3214",
            "$VEL.CP = {Programmed velocity}m/s cannot be implemented in "
            "CONST_VEL range due to velocity reduction to {Velocity reached} "
            "m/s in motion in {Module name@line number[:point name] of the "
            "limiting segment}.",
            "InfoOrAcknowledgment",
        },
        {
            "XEdit",
            "28",
            "Exception: Code: {0}, Ip: {1}",
            "Info",
        },
        {
            "CrossMeld",
            "13039",
            "Error initializing the Profinet firmware",
            "State",
        },
        {
            "CrossMeld",
            "15074",
            "Error safety node of drive {Device name}, channel {Channel "
            "number}",
            "State",
        },
        {
            "TechHandler",
            "22",
            "Touch-up not possible. Change of orientation out of range.",
            "Error",
        },
        {
            "CrossMeld",
            "13013",
            "<ECat> Error creating Ethercat stack instances",
            "State",
        },
        {
            "SubmitControl",
            "10",
            "Interpreter {Submit module} not available. “Select/Start” not "
            "possible.",
            "Info",
        },
        {
            "CrossMeld",
            "151",
            "The set velocity must still be confirmed after pressing the "
            "enabling switch.",
            "State",
        },
        {
            "CrossMeld",
            "1444",
            "Array index inadmissible: {Variables} {ModuleLine}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "1372",
            "Process active",
            "InfoOrAcknowledgment",
        },
        {
            "Navigator",
            "30",
            "Restoring failed",
            "Info",
        },
        {
            "CrossMeld",
            "249",
            "$MOVE_ENABLE configuration not allowed",
            "State",
        },
        {
            "CrossMeld",
            "29005",
            "RSI cannot set any outputs due to operator protection",
            "State",
        },
        {
            "CrossMeld",
            "10015",
            "{Instance name} bus error {Error code}, additional info {Expanded "
            "error code}",
            "State",
        },
        {
            "CrossMeld",
            "1615",
            "PRIO 40-80 closed",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1384",
            "Parameter not evaluated",
            "Info",
        },
        {
            "CrossMeld",
            "1074",
            "Command motor torque {Axis number}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1323",
            "{Path} inadmissible module name",
            "Info",
        },
        {
            "CrossMeld",
            "1402",
            "Select Start-up",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "5",
            "Creation of XML parser failed",
            "Error",
        },
        {
            "CrossMeld",
            "26177",
            "No force rise in force control ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "132",
            "Brake defective {Axis number}.",
            "State",
        },
        {
            "CrossMeld",
            "2008",
            "Block must not be modified",
            "Info",
        },
        {
            "CrossMeld",
            "159",
            "KRC IO bus error suppression active for bus instance: {Bus "
            "instance}{Bus instance}{Bus instance}",
            "State",
        },
        {
            "TechHandler",
            "23",
            "Touch-up not possible. Distance of external axis out of range.",
            "Error",
        },
        {
            "CrossMeld",
            "10028",
            "{Instance name} File {File name} cannot be read (formatting "
            "error)",
            "State",
        },
        {
            "CrossMeld",
            "313",
            "Internal error (file: {File name}, line: {Line number}, value: "
            "{Return value})",
            "State",
        },
        {
            "Navigator",
            "4",
            "Archiving failed",
            "Info",
        },
        {
            "CrossMeld",
            "3129",
            "Operator safety acknowledged",
            "Info",
        },
        {
            "CrossMeld",
            "26167",
            "Warning: Error in safety node interface (KSK) ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "3260",
            "Reconfiguration not possible in BUSPOWER OFF state",
            "Info",
        },
        {
            "CrossMeld",
            "2004",
            "DEFDAT expected",
            "Info",
        },
        {
            "EthernetKRL",
            "22",
            "Error while reading the configuration. XML not valid.",
            "Error",
        },
        {
            "CrossMeld",
            "10008",
            "{Instance name} No restart carried out, as no error present.",
            "Info",
        },
        {
            "CrossMeld",
            "15081",
            "Monitoring space no. {Number of monitoring space} exceeded",
            "State",
        },
        {
            "CrossMeld",
            "15066",
            "Level at mastering reference switch is unexpectedly \"low\"",
            "State",
        },
        {
            "EthernetKRL",
            "1536",
            "Received string too long",
            "Error",
        },
        {
            "CrossMeld",
            "2815",
            "The workspace has changed (program update required).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2125",
            "Component not of this type",
            "Info",
        },
        {
            "CrossMeld",
            "3171",
            "Velocity 0 m/s at point {Module name@line number[:point name] of "
            "the motion} (start and end points identical)",
            "Info",
        },
        {
            "CrossMeld",
            "10001",
            "{Instance name} user error: {Error code} additional info: "
            "{Expanded error code}",
            "State",
        },
        {
            "CrossMeld",
            "106",
            "Perform mastering {Axis number}",
            "State",
        },
        {
            "CrossMeld",
            "13041",
            "Error reading the MAC address from the KLI",
            "State",
        },
        {
            "CrossMeld",
            "12019",
            "Speed of PC fan below alarm threshold",
            "State",
        },
        {
            "CrossMeld",
            "2335",
            "Bit width of combined signal exceeds 32 bits",
            "Info",
        },
        {
            "CrossMeld",
            "26046",
            "Error: Mains phase failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "10068",
            "{Name of the instance} Error in the power supply to device number "
            "{Number of the device}",
            "State",
        },
        {
            "CrossMeld",
            "1350",
            "Programmed path reached (BCO)",
            "Info",
        },
        {
            "TPBASIS",
            "123",
            "Please copy content of the spline block separately.",
            "Info",
        },
        {
            "CrossMeld",
            "10024",
            "{Instance name} slave ID on card ({Slave ID on card}) is "
            "different from that in XML file ({Slave ID in XML file})",
            "Info",
        },
        {
            "CrossMeld",
            "99",
            "Error on reading/writing: {Bus instance}",
            "State",
        },
        {
            "CrossMeld",
            "404",
            "Safety stop",
            "State",
        },
        {
            "CrossMeld",
            "29003",
            "Correction reset required",
            "Info",
        },
        {
            "CrossMeld",
            "14006",
            "System error {Exception vector} in task {Originator}",
            "State",
        },
        {
            "CrossMeld",
            "10054",
            "Timeout establishing connection ID {ID} (Slot {ID}) to the EIP "
            "device {Name}",
            "State",
        },
        {
            "CrossMeld",
            "10091",
            "{Bus instance name}: Station {Faulty module} is in a static "
            "diagnostic state.",
            "State",
        },
        {
            "CrossMeld",
            "14009",
            "Boot failed for module: {Module name}",
            "State",
        },
        {
            "KRmsgNET",
            "13",
            "Configuration not found! Default configuration used.",
            "Info",
        },
        {
            "CrossMeld",
            "149",
            "Cartesian velocity is still limited in T2",
            "State",
        },
        {
            "EthernetKRL",
            "2304",
            "Insufficient memory",
            "Error",
        },
        {
            "CrossMeld",
            "15079",
            "Monitoring space no. {Number of monitoring space} violated",
            "State",
        },
        {
            "CrossMeld",
            "29000",
            "{Type} Permissible overall correction exceeded: RSI is stopped",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "200",
            "Drives not ready",
            "State",
        },
        {
            "CrossMeld",
            "1172",
            "Value cannot be changed with program selected",
            "Info",
        },
        {
            "CrossMeld",
            "12023",
            "Temperature above ballast resistor has reached alarm threshold",
            "State",
        },
        {
            "CrossMeld",
            "158",
            "Safe robot override reduction with low velocity",
            "State",
        },
        {
            "CrossMeld",
            "2255",
            "Expression not equal to CHAR, INT, ENUM",
            "Info",
        },
        {
            "CrossMeld",
            "15068",
            "Safety configuration activation code error",
            "State",
        },
        {
            "CrossMeld",
            "1044",
            "Ackn. Brake defective {Axis}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15012",
            "Enabling switch error",
            "State",
        },
        {
            "CrossMeld",
            "12014",
            "Speed of outer fan too low",
            "State",
        },
        {
            "CrossMeld",
            "1608",
            "Assignment of function value to $CYCFLAG inadmissible "
            "{ModuleLine}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "26105",
            "Warning: Ground fault ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "3157",
            "{Module name} : Name longer than 24 characters",
            "Info",
        },
        {
            "CrossMeld",
            "3297",
            "Axis {Axis index} in torque limitation",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26017",
            "Position controller in limit ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "2866",
            "Enter/exit workspace request not allowed in trigger.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13065",
            "Error configuring EthernetIP stack with the file {Name}",
            "State",
        },
        {
            "CrossMeld",
            "29008",
            "RSI I/Os configuration file not found.",
            "State",
        },
        {
            "CrossMeld",
            "27004",
            "Brake test required",
            "State",
        },
        {
            "CrossMeld",
            "10019",
            "{Instance name} Restart already in progress.",
            "Info",
        },
        {
            "EthernetKRL",
            "768",
            "Ping reports no contact",
            "Error",
        },
        {
            "CrossMeld",
            "277",
            "Automatic repositioning",
            "State",
        },
        {
            "CrossMeld",
            "29001",
            "{Type} Correction outside the permissible range: {Value}",
            "Info",
        },
        {
            "CrossMeld",
            "10095",
            "{Bus instance name}: Station {Faulty module} sends an invalid "
            "PROFIBUS telegram.",
            "State",
        },
        {
            "CrossMeld",
            "26015",
            "General power supply error ({Power supply}).",
            "State",
        },
        {
            "CrossMeld",
            "13040",
            "Error reading file {Configuration file}",
            "State",
        },
        {
            "CrossMeld",
            "10031",
            "{Instance name} Loading of SVC file aborted, firmware service "
            "error: {Number of the firmware service} ErrCode/AddInfo: {Error "
            "code and info about error}",
            "Info",
        },
        {
            "CrossMeld",
            "1",
            "EMERGENCY STOP",
            "State",
        },
        {
            "CrossMeld",
            "3263",
            "Max. brake time in MADA ($RED_ACC_EMX) and safety controller "
            "inconsistent",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2352",
            "$OUT_C[n] not allowed here",
            "Info",
        },
        {
            "CrossMeld",
            "26056",
            "CF: Invalid configuration parameter (Drive {Drive}, Type: {Type}, "
            "Index: {Index}).",
            "State",
        },
        {
            "CrossMeld",
            "10086",
            "{Bus instance name}: Station {Faulty module} does not respond to "
            "PROFIBUS.",
            "State",
        },
        {
            "CrossMeld",
            "15002",
            "Safe device communication error {Device}",
            "State",
        },
        {
            "CrossMeld",
            "1309",
            "Object is not available",
            "Info",
        },
        {
            "CrossMeld",
            "29007",
            "Error updating RSI signal {Signal name}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10009",
            "{Instance name} Periphery failure, device number {Device number}",
            "State",
        },
        {
            "CrossMeld",
            "1006",
            "No more user memory available",
            "Info",
        },
        {
            "EthernetKRL",
            "3",
            "File access failed",
            "Error",
        },
        {
            "CrossMeld",
            "20",
            "External EMERGENCY STOP",
            "State",
        },
        {
            "CrossMeld",
            "26197",
            "Motor {Motor} not available for coupling to drive {Drive}.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1127",
            "Mastering performed {Axis number}",
            "Info",
        },
        {
            "CrossMeld",
            "15053",
            "Ackn.: Mastering reference group no. {Number of the reference "
            "group} not referenced",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1421",
            "{Object name}: {Error number} compilation error",
            "Info",
        },
        {
            "CrossMeld",
            "10003",
            "{Instance name} Current configuration is not identical to active "
            "configuration",
            "State",
        },
        {
            "CrossMeld",
            "2940",
            "Simultaneously active cyclical flag limit exceeded (max. "
            "{Number}) {ModuleLine}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "455",
            "Error while reading {XML file}",
            "State",
        },
        {
            "CrossMeld",
            "25005",
            "EDS of device {Module name} is empty. No mastering data available "
            "(error {Internal error number}).",
            "Info",
        },
        {
            "CrossMeld",
            "15019",
            "Ackn.: Maximum axis-specific velocity in T1 mode exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15040",
            "Ackn.: Maximum global axis velocity exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15016",
            "Ackn.: Stop due to standstill monitoring violation",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1038",
            "Invalid operating mode",
            "Info",
        },
        {
            "CrossMeld",
            "115",
            "Perform safe robot retraction in workspace {Workspace number}",
            "State",
        },
        {
            "CrossMeld",
            "27007",
            "Insufficient holding torque of brake {Brake no.}{Axis no.}",
            "State",
        },
        {
            "CrossMeld",
            "1141",
            "TTS does not exist",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15112",
            "Pulse time monitoring of mastering confirmation violated",
            "Info",
        },
        {
            "CrossMeld",
            "304",
            "Start-up",
            "State",
        },
        {
            "CrossMeld",
            "10020",
            "{Instance name} Error accessing controller board",
            "Info",
        },
        {
            "CrossMeld",
            "1569",
            "Parameter change inadmissible, channel assigned",
            "Info",
        },
        {
            "CrossMeld",
            "26123",
            "Warning: Overload ballast circuit ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2274",
            "Current/formal parameter types incompatible",
            "Info",
        },
        {
            "CrossMeld",
            "15054",
            "Workspace monitoring functions deactivated (mastering error)",
            "State",
        },
        {
            "CrossMeld",
            "1155",
            "Approximate positioning not possible, torque too high {+ "
            "(redundant message)}",
            "Info",
        },
        {
            "CrossMeld",
            "2014",
            "Unused",
            "Info",
        },
        {
            "CrossMeld",
            "3230",
            "Conditional stop in {Module name@line no. where the stop "
            "condition is programmed} to motion in {Module name@line number of "
            "assigned motion [:point name]} active after a delay",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1508",
            "No module present",
            "Info",
        },
        {
            "CrossMeld",
            "15115",
            "External safety stop 1",
            "State",
        },
        {
            "CrossMeld",
            "1422",
            "{$ variable} value invalid",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "37026",
            "Catalog element for control not up to date (nexxtCtrl: {Drive}, "
            "{testarg_2}, {testarg_3})",
            "Info",
        },
        {
            "CrossMeld",
            "26119",
            "Warning: Mains power failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2051",
            "Name in first line must be same as module name",
            "Info",
        },
        {
            "CrossMeld",
            "15033",
            "More than one tool activated in the safety controller",
            "State",
        },
        {
            "CrossMeld",
            "15114",
            "Mastering confirmed without reference request",
            "Info",
        },
        {
            "CrossMeld",
            "272",
            "No robot number programmed",
            "State",
        },
        {
            "CrossMeld",
            "10063",
            "{Name of the instance} Overtemperature, device number {Number of "
            "the device}",
            "State",
        },
        {
            "CrossMeld",
            "1603",
            "Safety fence open",
            "Info",
        },
        {
            "CrossMeld",
            "26195",
            "Decoupling not possible ({Drive}).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26108",
            "Warning: Intermediate circuit voltage too low ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "3094",
            "$ORI_TYPE=#IGNORE not allowed, reason {Reason}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10018",
            "{Instance name} The watchdog has expired",
            "State",
        },
        {
            "CrossMeld",
            "82",
            "Robot stopped by submit",
            "State",
        },
        {
            "CrossMeld",
            "336",
            "Motion Cooperation package not installed",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "11",
            "Ethernet connection to external system established",
            "Error",
        },
        {
            "CrossMeld",
            "26157",
            "Ackn. Error: Charging of intermediate circuit failed ({Device "
            "type}) ({Number}).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "27001",
            "Brake {Brake no.}{Axis no.} has reached the wear limit",
            "State",
        },
        {
            "CrossMeld",
            "2972",
            "Checksum error file {File name}",
            "State",
        },
        {
            "CrossMeld",
            "10045",
            "Connection to EIP device {Name} terminated",
            "State",
        },
        {
            "CrossMeld",
            "1419",
            "{X,Y,Z,A,B,C} TOOL not programmed",
            "InfoOrAcknowledgment",
        },
        {
            "EthernetKRL",
            "1792",
            "Limit of element memory reached",
            "Error",
        },
        {
            "CrossMeld",
            "3329",
            "Assignment to $MOTION_MODE invalid; Motion mode {Specific Motion "
            "mode} not installed {ModuleLine}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15051",
            "Ackn.: Mastering test position not reached",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "14010",
            "System restored from Hibernate mode",
            "Info",
        },
        {
            "CrossMeld",
            "26111",
            "Warning: Device temperature is too high ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2078",
            "Identical predefined signal names expected",
            "Info",
        },
        {
            "CrossMeld",
            "12038",
            "Monitoring of the transformer temperature not activated",
            "State",
        },
        {
            "CrossMeld",
            "26113",
            "Warning: Motor phase failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2825",
            "A submit program must not execute workspace sharing commands.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15111",
            "Pulse time monitoring of mastering confirmation violated",
            "State",
        },
        {
            "CrossMeld",
            "10050",
            "{Bus instance name}: DeviceNet master configuration error "
            "({Reason}{Reason})",
            "State",
        },
        {
            "CrossMeld",
            "13015",
            "<{Bus ID}> Ethercat bus scan error. Device: {Incorrect device} "
            "[{Additional info}]",
            "State",
        },
        {
            "CrossMeld",
            "26122",
            "Warning: Fault in brake resistor ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "255",
            "{$Variable} invalid value",
            "State",
        },
        {
            "CrossMeld",
            "3170",
            "Axis {Axis} not in mechanical zero position",
            "Info",
        },
        {
            "CrossMeld",
            "26041",
            "Error: Motor phase failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2868",
            "PTP end point in singularity {Name of singularity}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "2809",
            "Interlock area {Workspace name} does not exist {KRC name}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15035",
            "No tool activated in safety controller",
            "State",
        },
        {
            "CrossMeld",
            "2360",
            "Approximate positioning parameters not allowed in spline",
            "Info",
        },
        {
            "CrossMeld",
            "10040",
            "{Bus instance name}: Profibus master configuration error "
            "({Reason}{Reason})",
            "State",
        },
        {
            "CrossMeld",
            "1505",
            "Invalid variable combination {Block number}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10027",
            "{Instance name} The data for this instance are not available in "
            "the file {File name}.",
            "State",
        },
        {
            "CrossMeld",
            "29010",
            "Wrong version number of the RSI I/O configuration file.",
            "State",
        },
        {
            "CrossMeld",
            "1153",
            "Deflection at start point of a PTP motion",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1424",
            "Program stack overflow {Location}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "416",
            "Controller is shutting down",
            "State",
        },
        {
            "CrossMeld",
            "1211",
            "STOP due to software limit switch {Motion direction} {Axis "
            "number}",
            "InfoOrAcknowledgment",
        },
        {
            "EthernetKRL",
            "24",
            "Link to internal parameters (Port, IP) failed",
            "Error",
        },
        {
            "CrossMeld",
            "27023",
            "Automatic safety factor reduction for brake {Brake}{Axis}",
            "Info",
        },
        {
            "CrossMeld",
            "14000",
            "Drive bus power switched off",
            "State",
        },
        {
            "CrossMeld",
            "10037",
            "{Instance name} Negative return value ({Error number and info}) "
            "for firmware service ({Number of the firmware service}) of the "
            "SVC file.",
            "Info",
        },
        {
            "CrossMeld",
            "1009",
            "Point conversion impossible without absolutely accurate model",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1443",
            "Impermissible start motion",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "12003",
            "Transformer overtemperature",
            "State",
        },
        {
            "EthernetKRL",
            "26",
            "FRAME array not initialized",
            "Error",
        },
        {
            "CrossMeld",
            "26033",
            "Error: Ground fault ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2162",
            "Incorrect input character",
            "Info",
        },
        {
            "CrossMeld",
            "10012",
            "{Instance name} Error {Error code} switching off segment {Segment "
            "number}",
            "Info",
        },
        {
            "CrossMeld",
            "27009",
            "Brake {Brake no.}{Axis no.} OK",
            "Info",
        },
        {
            "CrossMeld",
            "3236",
            "Energy measurement cannot currently be started.",
            "Info",
        },
        {
            "EthernetKRL",
            "9",
            "Connection not available",
            "Error",
        },
        {
            "CrossMeld",
            "2808",
            "Interlock {Workspace name} taken out of sequence. Deadlock "
            "possible.",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "16",
            "Element not found",
            "Error",
        },
        {
            "CrossMeld",
            "2285",
            "Component name expected",
            "Info",
        },
        {
            "CrossMeld",
            "1356",
            "Start key required",
            "Info",
        },
        {
            "CrossMeld",
            "1506",
            "Circular parameter inadmissible",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "3186",
            "Drives have been discharged (reason: {Reason for discharge})",
            "State",
        },
        {
            "CrossMeld",
            "375",
            "Warm-up active",
            "State",
        },
        {
            "CrossMeld",
            "11",
            "Task stack for command execution too small",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26118",
            "Warning: Mains phase failure ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "203",
            "General motion enable",
            "State",
        },
        {
            "CrossMeld",
            "10029",
            "{Instance name} no master or slave circuit activated",
            "Info",
        },
        {
            "CrossMeld",
            "26218",
            "Brake cooling time {Brake cooling time} s ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "13008",
            "<{Instance name}> Ethercat bus error. {Additional parameter} "
            "{Additional parameter}",
            "State",
        },
        {
            "EthernetKRL",
            "10",
            "Ethernet is disconnected",
            "Error",
        },
        {
            "CrossMeld",
            "10010",
            "{Instance name} Bus error in slave circuit",
            "State",
        },
        {
            "CrossMeld",
            "29002",
            "Signal flow ({Mode}): Object {ObjName} returns error {ErrorCode}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10000",
            "{Instance name} bus error: {Error code} device number: {Device "
            "number}",
            "State",
        },
        {
            "CrossMeld",
            "26194",
            "Mastering impermissible in the decoupled state {Drive}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "13037",
            "Profinet controller stack cannot be started, error code: {Code}",
            "State",
        },
        {
            "CrossMeld",
            "3191",
            "Active sensors not possible during {Incompatible functionality}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10049",
            "{Bus instance name}: DeviceNet master error in module ({Faulty "
            "module}) (error code: {Reason for error})",
            "State",
        },
        {
            "CrossMeld",
            "27010",
            "Unable to verify performance of brake {Brake}{Axis}",
            "Info",
        },
        {
            "CrossMeld",
            "26114",
            "Warning: Communication error ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "26173",
            "Maximum permissible force exceeded ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "2865",
            "Max. no. of workspace requests between two motion commands",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15041",
            "Ackn.: Maximum safe reduced Cartesian velocity exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "3275",
            "Positioning motions to PalMode only allowed with individual PTP "
            "and SPTP blocks.",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10023",
            "{Instance name} slave address unknown",
            "Info",
        },
        {
            "CrossMeld",
            "10021",
            "{Instance name} waiting for external start of Interbus",
            "State",
        },
        {
            "EthernetKRL",
            "2",
            "Out of system memory",
            "Error",
        },
        {
            "Cross3Archive",
            "24",
            "Timeout in {0}!",
            "Info",
        },
        {
            "CrossMeld",
            "1376",
            "Active commands inhibited",
            "Info",
        },
        {
            "CrossMeld",
            "15039",
            "Ackn.: Maximum global Cartesian velocity exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1442",
            "Sequence of statements that cannot be approximated {Module "
            "name@line number of the statement} {+ (redundant message)}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "148",
            "Axis-specific workspace no. {Workspace number} violated",
            "State",
        },
        {
            "CrossMeld",
            "2997",
            "Jogging not allowed: Start key pressed",
            "Info",
        },
        {
            "CrossMeld",
            "10094",
            "{Bus instance name}: Substantial bus error at Station {Faulty "
            "module}.",
            "State",
        },
        {
            "CrossMeld",
            "117",
            "Collision detection axis {Axis number}",
            "State",
        },
        {
            "CrossMeld",
            "2813",
            "The interlock area {Workspace name} is not blocked by a different "
            "program",
            "Info",
        },
        {
            "EthernetKRL",
            "8",
            "Add new element failed",
            "Error",
        },
        {
            "CrossMeld",
            "26230",
            "Error: Short-circuit in charging thyristor ({Device type}) "
            "({Number}) ",
            "State",
        },
        {
            "CrossMeld",
            "29012",
            "Error initializing the RSI I/Os.",
            "State",
        },
        {
            "EthernetKRL",
            "1280",
            "Limit of element storage reached",
            "Error",
        },
        {
            "CrossMeld",
            "1102",
            "Command velocity {Axis number}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26196",
            "Error during coupling ({Drive}) ({Error type}).",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "474",
            "{Name of iosys.ini} section [{Name of section}] line {Incorrect "
            "line}, incorrect address range",
            "State",
        },
        {
            "CrossMeld",
            "1100",
            "Stopped {(Axis number)}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10065",
            "{Name of the instance} Fault in communication to device number "
            "{Number of the device}",
            "State",
        },
        {
            "CrossMeld",
            "15037",
            "Cell area exceeded",
            "State",
        },
        {
            "CrossMeld",
            "26037",
            "Error: Logic supply voltage too high ({Device type}) ({Number}).",
            "State",
        },
        {
            "EthernetKRL",
            "20",
            "Mismatch in type of data",
            "Error",
        },
        {
            "CrossMeld",
            "2380",
            "Statement not allowed in PTP_SPLINE",
            "Info",
        },
        {
            "CrossMeld",
            "108",
            "Dynamic braking active",
            "State",
        },
        {
            "CrossMeld",
            "15036",
            "Ackn.: No tool activated in safety controller",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "263",
            "Incorrect correction data",
            "State",
        },
        {
            "CrossMeld",
            "3215",
            "CONST_VEL cannot be implemented due to velocity reduction to 0 "
            "m/s in motion in {Module name@line number[:point name] of the "
            "segment responsible}.",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "27",
            "CHAR[] Array too small.",
            "Error",
        },
        {
            "CrossMeld",
            "15107",
            "Error at the mastering confirmation input",
            "State",
        },
        {
            "CrossMeld",
            "10067",
            "{Name of the instance} Channel error at device {Device number}",
            "Info",
        },
        {
            "CrossMeld",
            "10064",
            "{Instance name} Device: {Device number} missing",
            "State",
        },
        {
            "CrossMeld",
            "10033",
            "{Instance name} Error in connection to firmware",
            "Info",
        },
        {
            "CrossMeld",
            "2144",
            "ENDLOOP missing",
            "Info",
        },
        {
            "EthernetKRL",
            "12",
            "Create server failed",
            "Error",
        },
        {
            "CrossMeld",
            "15018",
            "Ackn.: Maximum Cartesian velocity in T1 mode exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "1040",
            "Backward motion not possible: no trace available",
            "Info",
        },
        {
            "CrossMeld",
            "26133",
            "Warning: Group brake error ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "1456",
            "Start point identical to end point",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26067",
            "CF: Configuration invalid ({Drive}) ({Error type and parameter "
            "type}, index={Parameter index}).",
            "State",
        },
        {
            "CrossMeld",
            "27012",
            "Brake test successful",
            "Info",
        },
        {
            "CrossMeld",
            "10053",
            "Connection ID {ID} (Slot {ID}) to EIP device {Name} terminated",
            "State",
        },
        {
            "CrossMeld",
            "26112",
            "Warning: Heat sink temperature is too high ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "27011",
            "Test of brake {Brake}{Axis} not completed",
            "Info",
        },
        {
            "CrossMeld",
            "29011",
            "Couldn't find configured bus.",
            "State",
        },
        {
            "EthernetKRL",
            "25",
            "Internal software error",
            "Error",
        },
        {
            "CrossMeld",
            "1388",
            "{Name} variable write-protected in module {File name}, line "
            "{Block number}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "12053",
            "Configured cabinet type does not match EtherCat configuration",
            "State",
        },
        {
            "CrossMeld",
            "287",
            "PC fan defective",
            "State",
        },
        {
            "CrossMeld",
            "26188",
            "Servo parameters {Axis} no. {Parameter number} faulty",
            "Info",
        },
        {
            "CrossMeld",
            "3246",
            "Ackn. de-/coupling not allowed for axis {Axis}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2363",
            "Distance trigger not allowed in spline motion",
            "Info",
        },
        {
            "CrossMeld",
            "12017",
            "Operator safety not acknowledged",
            "State",
        },
        {
            "CrossMeld",
            "3216",
            "Acceleration distance for CONST_VEL START in motion in {Module "
            "name@line number[:point name] of the limiting segment} is too "
            "short. Instead of $VEL.CP = {Programmed velocity} m/s, only "
            "{Velocity reached} m/s is reached.",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "14011",
            "Boot error in file {File name} in line {Line number}: {Error "
            "text}",
            "Info",
        },
        {
            "CrossMeld",
            "26040",
            "Error: Heat sink temperature is too high ({Device type}) "
            "({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "2010",
            "Data list must not be renamed",
            "Info",
        },
        {
            "CrossMeld",
            "10051",
            "{Bus instance name}: Error in DeviceNet slave ring (error code: "
            "{Reason 1}{Reason 2})",
            "State",
        },
        {
            "CrossMeld",
            "10005",
            "{Instance name} diagnostic register shows faulty data cycle bit",
            "Info",
        },
        {
            "CrossMeld",
            "2039",
            "Instruction not in instruction section",
            "Info",
        },
        {
            "CrossMeld",
            "1524",
            "Max. number of active triggers exceeded",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "19",
            "No data to send",
            "Error",
        },
        {
            "CrossMeld",
            "13021",
            "<{Bus ID}> Ethercat network error. {Details} [{Details}]",
            "State",
        },
        {
            "CrossMeld",
            "29013",
            "RSI cannot communicate with field bus: {Field bus instance}",
            "State",
        },
        {
            "CrossMeld",
            "3201",
            "Following error {Axis number} in torque mode exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "15042",
            "Ackn.: Safe reduced axis velocity exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "26104",
            "Warning: Overload error IxT ({Device type}) ({Number}).",
            "State",
        },
        {
            "CrossMeld",
            "10022",
            "{Instance name} Caution! Bus mode is not 'Asynchronous with "
            "synchronization pulse'",
            "Info",
        },
        {
            "CrossMeld",
            "13011",
            "<{Bus ID}> Error reading configuration file [{XML file name}]",
            "State",
        },
        {
            "CrossMeld",
            "10036",
            "{Instance name} Error on starting slave ring (firmware service: "
            "{Number of the firmware service} ErrCode: {Error code of the "
            "firmware service})",
            "State",
        },
        {
            "CrossMeld",
            "10062",
            "{Name of the instance} Error in peripheral electronics of device "
            "number: {Number of the device}",
            "Info",
        },
        {
            "CrossMeld",
            "2086",
            "Array limit missing",
            "Info",
        },
        {
            "CrossMeld",
            "10089",
            "{Bus instance name}: Station {Faulty module} does not support DP "
            "function.",
            "State",
        },
        {
            "CrossMeld",
            "2256",
            "Operand not equal to INT, REAL",
            "Info",
        },
        {
            "CrossMeld",
            "3200",
            "Action not allowed while robot program selected",
            "Info",
        },
        {
            "CrossMeld",
            "3166",
            "Initial cold start of controller",
            "Info",
        },
        {
            "CrossMeld",
            "3273",
            "Spline too long",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "512",
            "Ethernet connection disrupted",
            "Error",
        },
        {
            "CrossMeld",
            "1401",
            "Control structure next block {Block number}",
            "InfoOrAcknowledgment",
        },
        {
            "EthernetKRL",
            "21",
            "System memory insufficient with maximum data storage",
            "Error",
        },
        {
            "CrossMeld",
            "301",
            "Measurement run interrupted - finished as test run",
            "Info",
        },
        {
            "KRMSGNET",
            "14",
            "KRMSGNET xsd schema not found!",
            "Info",
        },
        {
            "CrossMeld",
            "1106",
            "Tool weight not yet taught",
            "Info",
        },
        {
            "ConfigMon",
            "1",
            "No groups available for the current user",
            "Info",
        },
        {
            "CrossMeld",
            "2098",
            "Variable not declared in data list",
            "Info",
        },
        {
            "CrossMeld",
            "10088",
            "{Bus instance name}: Station {Faulty module} reports "
            "parameterization error.",
            "State",
        },
        {
            "CrossMeld",
            "188",
            "Motion mode {Motion mode} not installed",
            "State",
        },
        {
            "CrossMeld",
            "67",
            "Acknowledge operator safety",
            "State",
        },
        {
            "CrossMeld",
            "1133",
            "Maximum gear torque, axis {Axis number}",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10093",
            "{Bus instance name}: Physical fault in station {Faulty module}.",
            "State",
        },
        {
            "CrossMeld",
            "15046",
            "Error was at the mastering reference switch",
            "Info",
        },
        {
            "TPBASIS",
            "103",
            "Touch-up not possible. Cartesian distance {0} out of range.",
            "Error",
        },
        {
            "CrossMeld",
            "26128",
            "No connection with EMD",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2187",
            "String expected",
            "Info",
        },
        {
            "CrossMeld",
            "15050",
            "Reference stop",
            "State",
        },
        {
            "CrossMeld",
            "12032",
            "Position deviation (motion controller <-> safety controller)",
            "State",
        },
        {
            "CrossMeld",
            "10034",
            "Unable to project {Instance name} diagnostic registers into I/O "
            "map",
            "Info",
        },
        {
            "CrossMeld",
            "2814",
            "exitspace '{Workspace name}' failed - workspace not currently "
            "requested",
            "Info",
        },
        {
            "CrossMeld",
            "1005",
            "No more system memory available",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "29006",
            "RSI: Signal calculation timeout {CalcTime} usec",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "1",
            "Unknown error",
            "Error",
        },
        {
            "CrossMeld",
            "13074",
            "EIP I/O driver cannot determine the subnet mask of the KLI",
            "State",
        },
        {
            "CrossMeld",
            "207",
            "Operator safety / safety fence open",
            "State",
        },
        {
            "CrossMeld",
            "26091",
            "Max. soft following error limit exceeded ({Drive}).",
            "State",
        },
        {
            "CrossMeld",
            "1375",
            "Command inadmissible",
            "Info",
        },
        {
            "CrossMeld",
            "10066",
            "{Name of the instance} short circuit at device {Number of the "
            "device}",
            "State",
        },
        {
            "CrossMeld",
            "10087",
            "{Bus instance name}: Station {Faulty module} is currently "
            "performing cyclical data exchange with another master.",
            "State",
        },
        {
            "CrossMeld",
            "13072",
            "EIP I/O driver cannot determine the MAC address of the KLI",
            "State",
        },
        {
            "CrossMeld",
            "2810",
            "This interlock area has not been defined",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "14005",
            "Boot failed for module: {Module name}",
            "State",
        },
        {
            "CrossMeld",
            "15127",
            "Ackn.: Stop because workspace exceeded",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "10016",
            "{Instance name} transfer quality bit has been activated.",
            "Info",
        },
        {
            "EthernetKRL",
            "23",
            "Initialization already performed",
            "Error",
        },
        {
            "CrossMeld",
            "22",
            "Motherboard overtemperature",
            "State",
        },
        {
            "CrossMeld",
            "1123",
            "Approximate positioning not possible in {Module name@line "
            "number[:point name]} {Reason - optional}",
            "InfoOrAcknowledgment",
        },
        {
            "CrossMeld",
            "2812",
            "EXITSPACE '{Workspace name}' failed - not possible to establish "
            "connection to {KRC name}",
            "State",
        },
        {
            "CrossMeld",
            "1426",
            "Variable nesting depth exceeded",
            "Acknowledgment",
        },
        {
            "EthernetKRL",
            "14",
            "Ethernet connection to external system failed",
            "Error",
        },
        {
            "CrossMeld",
            "15110",
            "Level at mastering confirmation input was unexpectedly High",
            "Info",
        },
        {
            "CrossMeld",
            "27002",
            "Cyclical check for brake test request not made",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "3009",
            "Ownership of workspace {Workspace name} lost",
            "Acknowledgment",
        },
        {
            "CrossMeld",
            "2936",
            "$CP_STATMON: incorrect axis angle",
            "Acknowledgment",
        },
};
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KSS_MESSAGES_H_

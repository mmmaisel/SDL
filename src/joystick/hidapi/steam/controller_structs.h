/*
  Simple DirectMedia Layer
  Copyright (C) 2020 Valve Corporation

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/
#ifndef _CONTROLLER_STRUCTS_
#define _CONTROLLER_STRUCTS_

#pragma pack(1)

#define HID_FEATURE_REPORT_BYTES 64

// Header for all host <==> target messages
typedef struct
{
	unsigned char type;
	unsigned char length;
} FeatureReportHeader;

// Generic controller attribute structure
typedef struct
{
	unsigned char attributeTag;
	uint32_t attributeValue;
} ControllerAttribute;

// Generic controller settings structure
typedef struct
{
	ControllerAttribute attributes[ ( HID_FEATURE_REPORT_BYTES - sizeof( FeatureReportHeader ) ) / sizeof( ControllerAttribute ) ];
} MsgGetAttributes;


// This is the only message struct that application code should use to interact with feature request messages. Any new
// messages should be added to the union. The structures defined here should correspond to the ones defined in
// ValveDeviceCore.cpp.
//
typedef struct
{
	FeatureReportHeader header;
	union
	{
		MsgGetAttributes				getAttributes;
	} payload;

} FeatureReportMsg;

// Roll this version forward anytime that you are breaking compatibility of existing
// message types within ValveInReport_t or the header itself.  Hopefully this should
// be super rare and instead you shoudl just add new message payloads to the union,
// or just add fields to the end of existing payload structs which is expected to be 
// safe in all code consuming these as they should just consume/copy upto the prior size 
// they were aware of when processing.
#define k_ValveInReportMsgVersion 0x01

typedef enum
{
	ID_CONTROLLER_STATE = 1,
	ID_CONTROLLER_DEBUG = 2,
	ID_CONTROLLER_WIRELESS = 3,
	ID_CONTROLLER_STATUS = 4,
	ID_CONTROLLER_DEBUG2 = 5,
	ID_CONTROLLER_SECONDARY_STATE = 6,
	ID_CONTROLLER_BLE_STATE = 7,
	ID_CONTROLLER_DECK_STATE = 9,
	ID_CONTROLLER_MSG_COUNT
} ValveInReportMessageIDs; 

typedef struct 
{
	unsigned short unReportVersion;
	
	unsigned char ucType;
	unsigned char ucLength;
	
} ValveInReportHeader_t;

// State payload
typedef struct 
{
	// If packet num matches that on your prior call, then the controller state hasn't been changed since 
	// your last call and there is no need to process it
	uint32 unPacketNum;
	
	// Button bitmask and trigger data.
	union
	{
		uint64 ulButtons;
		struct
		{
			unsigned char _pad0[3];
			unsigned char nLeft;
			unsigned char nRight;
			unsigned char _pad1[3];
		} Triggers;
	} ButtonTriggerData;
	
	// Left pad coordinates
	short sLeftPadX;
	short sLeftPadY;
	
	// Right pad coordinates
	short sRightPadX;
	short sRightPadY;
	
	// This is redundant, packed above, but still sent over wired
	unsigned short sTriggerL;
	unsigned short sTriggerR;

	// FIXME figure out a way to grab this stuff over wireless
	short sAccelX;
	short sAccelY;
	short sAccelZ;
	
	short sGyroX;
	short sGyroY;
	short sGyroZ;
	
	short sGyroQuatW;
	short sGyroQuatX;
	short sGyroQuatY;
	short sGyroQuatZ;

} ValveControllerStatePacket_t;

// BLE State payload this has to be re-formatted from the normal state because BLE controller shows up as 
//a HID device and we don't want to send all the optional parts of the message. Keep in sync with struct above.
typedef struct
{
	// If packet num matches that on your prior call, then the controller state hasn't been changed since 
	// your last call and there is no need to process it
	uint32 unPacketNum;

	// Button bitmask and trigger data.
	union
	{
		uint64 ulButtons;
		struct
		{
			unsigned char _pad0[3];
			unsigned char nLeft;
			unsigned char nRight;
			unsigned char _pad1[3];
		} Triggers;
	} ButtonTriggerData;

	// Left pad coordinates
	short sLeftPadX;
	short sLeftPadY;

	// Right pad coordinates
	short sRightPadX;
	short sRightPadY;

	//This mimcs how the dongle reconstitutes HID packets, there will be 0-4 shorts depending on gyro mode
	unsigned char ucGyroDataType; //TODO could maybe find some unused bits in the button field for this info (is only 2bits)
	short sGyro[4];

} ValveControllerBLEStatePacket_t;

// Define a payload for reporting debug information
typedef struct
{
	// Left pad coordinates
	short sLeftPadX;
	short sLeftPadY;

	// Right pad coordinates
	short sRightPadX;
	short sRightPadY;

	// Left mouse deltas
	short sLeftPadMouseDX;
	short sLeftPadMouseDY;

	// Right mouse deltas
	short sRightPadMouseDX;
	short sRightPadMouseDY;
	
	// Left mouse filtered deltas
	short sLeftPadMouseFilteredDX;
	short sLeftPadMouseFilteredDY;

	// Right mouse filtered deltas
	short sRightPadMouseFilteredDX;
	short sRightPadMouseFilteredDY;
	
	// Pad Z values
	unsigned char ucLeftZ;
	unsigned char ucRightZ;
	
	// FingerPresent
	unsigned char ucLeftFingerPresent;
	unsigned char ucRightFingerPresent;
	
	// Timestamps
	unsigned char ucLeftTimestamp;
	unsigned char ucRightTimestamp;
	
	// Double tap state
	unsigned char ucLeftTapState;
	unsigned char ucRightTapState;
	
	unsigned int unDigitalIOStates0;
	unsigned int unDigitalIOStates1;
	
} ValveControllerDebugPacket_t;

typedef struct
{
	unsigned char ucPadNum;
	unsigned char ucPad[3]; // need Data to be word aligned
	short Data[20];
	unsigned short unNoise;
} ValveControllerTrackpadImage_t;

typedef struct
{
	unsigned char ucPadNum;
	unsigned char ucOffset;
	unsigned char ucPad[2]; // need Data to be word aligned
	short rgData[28];
} ValveControllerRawTrackpadImage_t;

// Payload for wireless metadata
typedef struct 
{
	unsigned char ucEventType;
} SteamControllerWirelessEvent_t;

typedef struct 
{
	// Current packet number.
    unsigned int unPacketNum;
	
	// Event codes and state information.
    unsigned short sEventCode;
    unsigned short unStateFlags;

    // Current battery voltage (mV).
    unsigned short sBatteryVoltage;
	
	// Current battery level (0-100).
	unsigned char ucBatteryLevel;
} SteamControllerStatusEvent_t;

// Deck State payload
typedef struct 
{
	// If packet num matches that on your prior call, then the controller state hasn't been changed since 
	// your last call and there is no need to process it
	// *  4-7   | u32   | --        | sequence number
	uint32 unPacketNum;
	
	// Button bitmask and trigger data.
	// *  8-15  | u64   | see below | buttons

	// * The buttons are:
	// *  Bit  | Mapped to  | Description
	// * ------+------------+--------------------------------
	// *  8.0  | BTN_TR2    | right trigger fully pressed
	// *  8.1  | BTN_TL2    | left trigger fully pressed
	// *  8.2  | BTN_TR     | right shoulder
	// *  8.3  | BTN_TL     | left shoulder
	// *  8.4  | BTN_Y      | button Y
	// *  8.5  | BTN_B      | button B
	// *  8.6  | BTN_X      | button X
	// *  8.7  | BTN_A      | button A
	// *  9.0  | BTN_DPAD_UP    | left-pad up
	// *  9.1  | BTN_DPAD_RIGHT | left-pad right
	// *  9.2  | BTN_DPAD_LEFT  | left-pad left
	// *  9.3  | BTN_DPAD_DOWN  | left-pad down
	// *  9.4  | BTN_SELECT | menu left
	// *  9.5  | BTN_MODE   | steam logo
	// *  9.6  | BTN_START  | menu right
	// *  9.7  | BTN_TRIGGER_HAPPY3 | left bottom grip button
	// *  10.0 | BTN_TRIGGER_HAPPY4 | right bottom grip button
	// *  10.1 | BTN_THUMB  | left pad pressed
	// *  10.2 | BTN_THUMB2 | right pad pressed
	// *  10.3 | --         | left pad touched
	// *  10.4 | --         | right pad touched
	// *  10.5 | --         | unknown
	// *  10.6 | BTN_THUMBL | left joystick clicked
	// *  10.7 | --         | unknown
	// *  11.0 | --         | unknown
	// *  11.1 | --         | unknown
	// *  11.2 | BTN_THUMBR | right joystick clicked
	// *  11.3 | --         | unknown
	// *  11.4 | --         | unknown
	// *  11.5 | --         | unknown
	// *  11.6 | --         | unknown
	// *  11.7 | --         | unknown
	// *  12.0 | --         | unknown
	// *  12.1 | --         | unknown
	// *  12.2 | --         | unknown
	// *  12.3 | --         | unknown
	// *  12.4 | --         | unknown
	// *  12.5 | --         | unknown
	// *  12.6 | --         | unknown
	// *  12.7 | --         | unknown
	// *  13.0 | --         | unknown
	// *  13.1 | BTN_TRIGGER_HAPPY1 | left top grip button
	// *  13.2 | BTN_TRIGGER_HAPPY2 | right top grip button
	// *  13.3 | --         | unknown
	// *  13.4 | --         | unknown
	// *  13.5 | --         | unknown
	// *  13.6 | --         | left joystick touched
	// *  13.7 | --         | right joystick touched
	// *  14.0 | --         | unknown
	// *  14.1 | --         | unknown
	// *  14.2 | BTN_BASE   | quick access button
	// *  14.3 | --         | unknown
	// *  14.4 | --         | unknown
	// *  14.5 | --         | unknown
	// *  14.6 | --         | unknown
	// *  14.7 | --         | unknown
	// *  15.0 | --         | unknown
	// *  15.1 | --         | unknown
	// *  15.2 | --         | unknown
	// *  15.3 | --         | unknown
	// *  15.4 | --         | unknown
	// *  15.5 | --         | unknown
	// *  15.6 | --         | unknown
	// *  15.7 | --         | unknown
	uint64 ulButtons;
	
	// Left pad coordinates
	// *  16-17 | s16   | ABS_HAT0X | left-pad X value
	// *  18-19 | s16   | ABS_HAT0Y | left-pad Y value
	short sLeftPadX;
	short sLeftPadY;
	
	// Right pad coordinates
	// *  20-21 | s16   | ABS_HAT1X | right-pad X value
	// *  22-23 | s16   | ABS_HAT1Y | right-pad Y value
	short sRightPadX;
	short sRightPadY;
	
	// *  24-25 | s16   | --        | accelerometer X value
	// *  26-27 | s16   | --        | accelerometer Y value
	// *  28-29 | s16   | --        | accelerometer Z value
	short sAccelX;
	short sAccelY;
	short sAccelZ;

	// *  30-31 | s16   | --        | gyro X value
	// *  32-33 | s16   | --        | gyro Y value
	// *  34-35 | s16   | --        | gyro Z value
	short sGyroX;
	short sGyroY;
	short sGyroZ;

	// *  36-37 | s16   | --        | quaternion W value
	// *  38-39 | s16   | --        | quaternion X value
	// *  40-41 | s16   | --        | quaternion Y value
	// *  42-43 | s16   | --        | quaternion Z value
	short sGyroQuatW;
	short sGyroQuatX;
	short sGyroQuatY;
	short sGyroQuatZ;

	// *  44-45 | u16   | ABS_HAT2Y | left trigger (uncalibrated)
	short sLeftTrigger;
	// *  46-47 | u16   | ABS_HAT2X | right trigger (uncalibrated)
	short sRightTrigger;

	// *  48-49 | s16   | ABS_X     | left joystick X
	// *  50-51 | s16   | ABS_Y     | left joystick Y
	short sLeftStickX;
	short sLeftStickY;

	// *  52-53 | s16   | ABS_RX    | right joystick X
	// *  54-55 | s16   | ABS_RY    | right joystick Y
	short sRightStickX;
	short sRightStickY;

	// *  56-57 | u16   | --        | left pad pressure
	// *  58-59 | u16   | --        | right pad pressure
	short sLeftPadPressure;
	short sRightPadPressure;


} SteamDeckStatePacket_t;

typedef struct
{
	ValveInReportHeader_t header;
	
	union
	{
		ValveControllerStatePacket_t controllerState;
		ValveControllerBLEStatePacket_t controllerBLEState;
		ValveControllerDebugPacket_t debugState;
		ValveControllerTrackpadImage_t padImage;
		ValveControllerRawTrackpadImage_t rawPadImage;
		SteamControllerWirelessEvent_t wirelessEvent;
		SteamControllerStatusEvent_t statusEvent;
		SteamDeckStatePacket_t deckState;
	} payload;
	
} ValveInReport_t;


// Enumeration for BLE packet protocol
enum EBLEPacketReportNums
{
	// Skipping past 2-3 because they are escape characters in Uart protocol
	k_EBLEReportState = 4,
	k_EBLEReportStatus = 5,
};


// Enumeration of data chunks in BLE state packets
enum EBLEOptionDataChunksBitmask
{
	// First byte uppper nibble
	k_EBLEButtonChunk1 = 0x10,
	k_EBLEButtonChunk2 = 0x20,
	k_EBLEButtonChunk3 = 0x40,
	k_EBLELeftJoystickChunk = 0x80,

	// Second full byte
	k_EBLELeftTrackpadChunk = 0x100,
	k_EBLERightTrackpadChunk = 0x200,
	k_EBLEIMUAccelChunk = 0x400,
	k_EBLEIMUGyroChunk = 0x800,
	k_EBLEIMUQuatChunk = 0x1000,
};

#pragma pack()

#endif // _CONTROLLER_STRUCTS

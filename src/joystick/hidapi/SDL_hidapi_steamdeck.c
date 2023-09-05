/*
  Simple DirectMedia Layer
  Copyright (C) 2023 Max Maisel <max.maisel@posteo.de>

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
#include "../../SDL_internal.h"

#ifdef SDL_JOYSTICK_HIDAPI

#include "../SDL_sysjoystick.h"
#include "SDL_hidapijoystick_c.h"

#ifdef SDL_JOYSTICK_HIDAPI_STEAMDECK

/*****************************************************************************************************/

#include <stdint.h>

#define bool  SDL_bool
#define true  SDL_TRUE
#define false SDL_FALSE

typedef uint32_t uint32;
typedef uint64_t uint64;

static int ReadSteamDeck(SDL_hid_device *dev, uint8_t *pData, int nDataSize)
{
    SDL_memset(pData, 0, nDataSize);
    return SDL_hid_read(dev, pData, nDataSize);
}

/*
 * The size for this message payload is 56.
 * The known values are:
 *  Offset| Type  | Mapped to |Meaning
 * -------+-------+-----------+--------------------------
 *  4-7   | u32   | --        | sequence number
 *  8-15  | u64   | see below | buttons
 *  16-17 | s16   | ABS_HAT0X | left-pad X value
 *  18-19 | s16   | ABS_HAT0Y | left-pad Y value
 *  20-21 | s16   | ABS_HAT1X | right-pad X value
 *  22-23 | s16   | ABS_HAT1Y | right-pad Y value
 *  24-25 | s16   | --        | accelerometer X value
 *  26-27 | s16   | --        | accelerometer Y value
 *  28-29 | s16   | --        | accelerometer Z value
 *  30-31 | s16   | --        | gyro X value
 *  32-33 | s16   | --        | gyro Y value
 *  34-35 | s16   | --        | gyro Z value
 *  36-37 | s16   | --        | quaternion W value
 *  38-39 | s16   | --        | quaternion X value
 *  40-41 | s16   | --        | quaternion Y value
 *  42-43 | s16   | --        | quaternion Z value
 *  44-45 | u16   | ABS_HAT2Y | left trigger (uncalibrated)
 *  46-47 | u16   | ABS_HAT2X | right trigger (uncalibrated)
 *  48-49 | s16   | ABS_X     | left joystick X
 *  50-51 | s16   | ABS_Y     | left joystick Y
 *  52-53 | s16   | ABS_RX    | right joystick X
 *  54-55 | s16   | ABS_RY    | right joystick Y
 *  56-57 | u16   | --        | left pad pressure
 *  58-59 | u16   | --        | right pad pressure
 *
 * The buttons are:
 *  Bit  | Mapped to  | Description
 * ------+------------+--------------------------------
 *  8.0  | BTN_TR2    | right trigger fully pressed
 *  8.1  | BTN_TL2    | left trigger fully pressed
 *  8.2  | BTN_TR     | right shoulder
 *  8.3  | BTN_TL     | left shoulder
 *  8.4  | BTN_Y      | button Y
 *  8.5  | BTN_B      | button B
 *  8.6  | BTN_X      | button X
 *  8.7  | BTN_A      | button A
 *  9.0  | BTN_DPAD_UP    | left-pad up
 *  9.1  | BTN_DPAD_RIGHT | left-pad right
 *  9.2  | BTN_DPAD_LEFT  | left-pad left
 *  9.3  | BTN_DPAD_DOWN  | left-pad down
 *  9.4  | BTN_SELECT | menu left
 *  9.5  | BTN_MODE   | steam logo
 *  9.6  | BTN_START  | menu right
 *  9.7  | BTN_TRIGGER_HAPPY3 | left bottom grip button
 *  10.0 | BTN_TRIGGER_HAPPY4 | right bottom grip button
 *  10.1 | BTN_THUMB  | left pad pressed
 *  10.2 | BTN_THUMB2 | right pad pressed
 *  10.3 | --         | left pad touched
 *  10.4 | --         | right pad touched
 *  10.5 | --         | unknown
 *  10.6 | BTN_THUMBL | left joystick clicked
 *  10.7 | --         | unknown
 *  11.0 | --         | unknown
 *  11.1 | --         | unknown
 *  11.2 | BTN_THUMBR | right joystick clicked
 *  11.3 | --         | unknown
 *  11.4 | --         | unknown
 *  11.5 | --         | unknown
 *  11.6 | --         | unknown
 *  11.7 | --         | unknown
 *  12.0 | --         | unknown
 *  12.1 | --         | unknown
 *  12.2 | --         | unknown
 *  12.3 | --         | unknown
 *  12.4 | --         | unknown
 *  12.5 | --         | unknown
 *  12.6 | --         | unknown
 *  12.7 | --         | unknown
 *  13.0 | --         | unknown
 *  13.1 | BTN_TRIGGER_HAPPY1 | left top grip button
 *  13.2 | BTN_TRIGGER_HAPPY2 | right top grip button
 *  13.3 | --         | unknown
 *  13.4 | --         | unknown
 *  13.5 | --         | unknown
 *  13.6 | --         | left joystick touched
 *  13.7 | --         | right joystick touched
 *  14.0 | --         | unknown
 *  14.1 | --         | unknown
 *  14.2 | BTN_BASE   | quick access button
 *  14.3 | --         | unknown
 *  14.4 | --         | unknown
 *  14.5 | --         | unknown
 *  14.6 | --         | unknown
 *  14.7 | --         | unknown
 *  15.0 | --         | unknown
 *  15.1 | --         | unknown
 *  15.2 | --         | unknown
 *  15.3 | --         | unknown
 *  15.4 | --         | unknown
 *  15.5 | --         | unknown
 *  15.6 | --         | unknown
 *  15.7 | --         | unknown
 */

#if 0
static void steam_do_deck_input_event(struct steam_device *steam,
		struct input_dev *input, u8 *data)
{
	u8 b8, b9, b10, b11, b13, b14;
	bool lpad_touched, rpad_touched;

	b8 = data[8];
	b9 = data[9];
	b10 = data[10];
	b11 = data[11];
	b13 = data[13];
	b14 = data[14];

	lpad_touched = b10 & BIT(3);
	rpad_touched = b10 & BIT(4);

	if (lpad_touched) {
		input_report_abs(input, ABS_HAT0X, steam_le16(data + 16));
		input_report_abs(input, ABS_HAT0Y, steam_le16(data + 18));
	} else {
		input_report_abs(input, ABS_HAT0X, 0);
		input_report_abs(input, ABS_HAT0Y, 0);
	}

	if (rpad_touched) {
		input_report_abs(input, ABS_HAT1X, steam_le16(data + 20));
		input_report_abs(input, ABS_HAT1Y, steam_le16(data + 22));
	} else {
		input_report_abs(input, ABS_HAT1X, 0);
		input_report_abs(input, ABS_HAT1Y, 0);
	}

	input_report_abs(input, ABS_X, steam_le16(data + 48));
	input_report_abs(input, ABS_Y, -steam_le16(data + 50));
	input_report_abs(input, ABS_RX, steam_le16(data + 52));
	input_report_abs(input, ABS_RY, -steam_le16(data + 54));

	input_report_abs(input, ABS_HAT2Y, steam_le16(data + 44));
	input_report_abs(input, ABS_HAT2X, steam_le16(data + 46));

	input_event(input, EV_KEY, BTN_TR2, !!(b8 & BIT(0)));
	input_event(input, EV_KEY, BTN_TL2, !!(b8 & BIT(1)));
	input_event(input, EV_KEY, BTN_TR, !!(b8 & BIT(2)));
	input_event(input, EV_KEY, BTN_TL, !!(b8 & BIT(3)));
	input_event(input, EV_KEY, BTN_Y, !!(b8 & BIT(4)));
	input_event(input, EV_KEY, BTN_B, !!(b8 & BIT(5)));
	input_event(input, EV_KEY, BTN_X, !!(b8 & BIT(6)));
	input_event(input, EV_KEY, BTN_A, !!(b8 & BIT(7)));
	input_event(input, EV_KEY, BTN_SELECT, !!(b9 & BIT(4)));
	input_event(input, EV_KEY, BTN_MODE, !!(b9 & BIT(5)));
	input_event(input, EV_KEY, BTN_START, !!(b9 & BIT(6)));
	input_event(input, EV_KEY, BTN_TRIGGER_HAPPY3, !!(b9 & BIT(7)));
	input_event(input, EV_KEY, BTN_TRIGGER_HAPPY4, !!(b10 & BIT(0)));
	input_event(input, EV_KEY, BTN_THUMBL, !!(b10 & BIT(6)));
	input_event(input, EV_KEY, BTN_THUMBR, !!(b11 & BIT(2)));
	input_event(input, EV_KEY, BTN_DPAD_UP, !!(b9 & BIT(0)));
	input_event(input, EV_KEY, BTN_DPAD_RIGHT, !!(b9 & BIT(1)));
	input_event(input, EV_KEY, BTN_DPAD_LEFT, !!(b9 & BIT(2)));
	input_event(input, EV_KEY, BTN_DPAD_DOWN, !!(b9 & BIT(3)));
	input_event(input, EV_KEY, BTN_THUMB, !!(b10 & BIT(1)));
	input_event(input, EV_KEY, BTN_THUMB2, !!(b10 & BIT(2)));
	input_event(input, EV_KEY, BTN_TRIGGER_HAPPY1, !!(b13 & BIT(1)));
	input_event(input, EV_KEY, BTN_TRIGGER_HAPPY2, !!(b13 & BIT(2)));
	input_event(input, EV_KEY, BTN_BASE, !!(b14 & BIT(2)));

	input_sync(input);
}

static void steam_do_deck_sensors_event(struct steam_device *steam,
		struct input_dev *sensors, u8 *data)
{
	input_report_abs(sensors, ABS_X, steam_le16(data + 24));
	input_report_abs(sensors, ABS_Y, steam_le16(data + 26));
	input_report_abs(sensors, ABS_Z, steam_le16(data + 28));
	input_report_abs(sensors, ABS_RX, steam_le16(data + 30));
	input_report_abs(sensors, ABS_RY, steam_le16(data + 32));
	input_report_abs(sensors, ABS_RZ, steam_le16(data + 34));

	input_sync(sensors);
}
#endif

/*****************************************************************************************************/

static void HIDAPI_DriverSteamDeck_RegisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_AddHintCallback(SDL_HINT_JOYSTICK_HIDAPI_STEAMDECK, callback, userdata);
}

static void HIDAPI_DriverSteamDeck_UnregisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_DelHintCallback(SDL_HINT_JOYSTICK_HIDAPI_STEAMDECK, callback, userdata);
}

static SDL_bool HIDAPI_DriverSteamDeck_IsEnabled(void)
{
    return SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI_STEAMDECK,
                              SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI, SDL_HIDAPI_DEFAULT));
}

static SDL_bool HIDAPI_DriverSteamDeck_IsSupportedDevice(
    SDL_HIDAPI_Device *device,
    const char *name,
    SDL_GameControllerType type,
    Uint16 vendor_id,
    Uint16 product_id,
    Uint16 version,
    int interface_number,
    int interface_class,
    int interface_subclass,
    int interface_protocol)
{
    return SDL_IsJoystickSteamDeck(vendor_id, product_id);
}

static SDL_bool HIDAPI_DriverSteamDeck_InitDevice(SDL_HIDAPI_Device *device)
{
#if 0
    SDL_DriverSteam_Context *ctx;

    ctx = (SDL_DriverSteam_Context *)SDL_calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        SDL_OutOfMemory();
        return SDL_FALSE;
    }
    device->context = ctx;
#endif

    // TODO: pick the correct device, there is controler, mouse, keyboard with the same vid/pid
    SDL_Log("HIDAPI Steam Deck initialize called\n");
    HIDAPI_SetDeviceName(device, "Steam Deck");

    return HIDAPI_JoystickConnected(device, NULL);
}

static int HIDAPI_DriverSteamDeck_GetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id)
{
    return -1;
}

static void HIDAPI_DriverSteamDeck_SetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id, int player_index)
{
}

static SDL_bool HIDAPI_DriverSteamDeck_UpdateDevice(SDL_HIDAPI_Device *device)
{
    //SDL_DriverSteam_Context *ctx = (SDL_DriverSteam_Context *)device->context;
    SDL_Joystick *joystick = NULL;

    if (device->num_joysticks > 0) {
        joystick = SDL_JoystickFromInstanceID(device->joysticks[0]);
    } else {
        return SDL_FALSE;
    }

    for (;;) {
        uint8_t data[56];
        int r, nPacketLength;
        const Uint8 *pPacket;

        r = ReadSteamDeck(device->dev, data, sizeof(data));
        if (r == 0) {
            break;
        }

        if (joystick == NULL) {
            continue;
        }

        // TODO: parse and validate packet
        nPacketLength = 0;
        if (r > 0) {
            //nPacketLength = WriteSegmentToSteamControllerPacketAssembler(&ctx->m_assembler, data, r);
        }

        //pPacket = ctx->m_assembler.uBuffer;

        if (nPacketLength > 0) { // && UpdateSteamControllerState(pPacket, nPacketLength, &ctx->m_state)) {
            //Uint64 timestamp = SDL_GetTicksNS();

            /*if (ctx->m_state.ulButtons != ctx->m_last_state.ulButtons) {
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_A,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_3_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_B,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_1_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_X,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_2_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_Y,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_0_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,
                                          (ctx->m_state.ulButtons & STEAM_LEFT_BUMPER_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER,
                                          (ctx->m_state.ulButtons & STEAM_RIGHT_BUMPER_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_BACK,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_MENU_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_START,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_ESCAPE_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_GUIDE,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_STEAM_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_LEFT_STICK,
                                          (ctx->m_state.ulButtons & STEAM_JOYSTICK_BUTTON_MASK) ? SDL_PRESSED : SDL_RELEASED);
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_MISC1 + 0,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_BACK_LEFT_MASK) ? SDL_PRESSED : SDL_RELEASED);
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_MISC1 + 1,
                                          (ctx->m_state.ulButtons & STEAM_BUTTON_BACK_RIGHT_MASK) ? SDL_PRESSED : SDL_RELEASED);

                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_DPAD_UP,
                                          (ctx->m_state.ulButtons & STEAM_TOUCH_0_MASK) ? SDL_PRESSED : SDL_RELEASED);
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_DPAD_DOWN,
                                          (ctx->m_state.ulButtons & STEAM_TOUCH_3_MASK) ? SDL_PRESSED : SDL_RELEASED);
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_DPAD_LEFT,
                                          (ctx->m_state.ulButtons & STEAM_TOUCH_2_MASK) ? SDL_PRESSED : SDL_RELEASED);
                SDL_SendJoystickButton(timestamp, joystick, SDL_GAMEPAD_BUTTON_DPAD_RIGHT,
                                          (ctx->m_state.ulButtons & STEAM_TOUCH_1_MASK) ? SDL_PRESSED : SDL_RELEASED);
            }*/

            /*SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFT_TRIGGER, (int)ctx->m_state.sTriggerL * 2 - 32768);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, (int)ctx->m_state.sTriggerR * 2 - 32768);

            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFTX, ctx->m_state.sLeftStickX);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFTY, ~ctx->m_state.sLeftStickY);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHTX, ctx->m_state.sRightPadX);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHTY, ~ctx->m_state.sRightPadY);*/

            /*if (ctx->report_sensors) {
                float values[3];

                ctx->sensor_timestamp += SDL_US_TO_NS(ctx->update_rate_in_us);

                values[0] = (ctx->m_state.sGyroX / 32768.0f) * (2000.0f * (SDL_PI_F / 180.0f));
                values[1] = (ctx->m_state.sGyroZ / 32768.0f) * (2000.0f * (SDL_PI_F / 180.0f));
                values[2] = (ctx->m_state.sGyroY / 32768.0f) * (2000.0f * (SDL_PI_F / 180.0f));
                SDL_SendJoystickSensor(timestamp, joystick, SDL_SENSOR_GYRO, ctx->sensor_timestamp, values, 3);

                values[0] = (ctx->m_state.sAccelX / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
                values[1] = (ctx->m_state.sAccelZ / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
                values[2] = (-ctx->m_state.sAccelY / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
                SDL_SendJoystickSensor(timestamp, joystick, SDL_SENSOR_ACCEL, ctx->sensor_timestamp, values, 3);
            }*/

            //ctx->m_last_state = ctx->m_state;
        }

        if (r <= 0) {
            /* Failed to read from controller */
            HIDAPI_JoystickDisconnected(device, device->joysticks[0]);
            return SDL_FALSE;
        }
    }
    return SDL_TRUE;
}

static SDL_bool HIDAPI_DriverSteamDeck_OpenJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    float update_rate_in_hz = 1000.0f;
    SDL_Log("HIDAPI Steam Deck open joystick called\n");
    //SDL_DriverSteam_Context *ctx = (SDL_DriverSteam_Context *)device->context;
    // Should be always 1kHz according to USB descriptor

    SDL_AssertJoysticksLocked();

    /*ctx->report_sensors = SDL_FALSE;
    SDL_zero(ctx->m_assembler);
    SDL_zero(ctx->m_state);
    SDL_zero(ctx->m_last_state);*/

    /*if (!ResetSteamController(device->dev, false, &ctx->update_rate_in_us)) {
        SDL_SetError("Couldn't reset controller");
        return SDL_FALSE;
    }
    if (ctx->update_rate_in_us > 0) {
        update_rate_in_hz = 1000000.0f / ctx->update_rate_in_us;
    }

    InitializeSteamControllerPacketAssembler(&ctx->m_assembler);*/

    /* Initialize the joystick capabilities */
    joystick->nbuttons = 20;
    joystick->naxes = SDL_CONTROLLER_AXIS_MAX;

    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_GYRO, update_rate_in_hz);
    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_ACCEL, update_rate_in_hz);

    return SDL_TRUE;
}

static int HIDAPI_DriverSteamDeck_RumbleJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 low_frequency_rumble, Uint16 high_frequency_rumble)
{
    /* You should use the full Steam Input API for rumble support */
    return SDL_Unsupported();
}

static int HIDAPI_DriverSteamDeck_RumbleJoystickTriggers(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 left_rumble, Uint16 right_rumble)
{
    return SDL_Unsupported();
}

static Uint32 HIDAPI_DriverSteamDeck_GetJoystickCapabilities(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    return 0;
}

static int HIDAPI_DriverSteamDeck_SetJoystickLED(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint8 red, Uint8 green, Uint8 blue)
{
    return SDL_Unsupported();
}

static int HIDAPI_DriverSteamDeck_SendJoystickEffect(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, const void *data, int size)
{
    return SDL_Unsupported();
}

static int HIDAPI_DriverSteamDeck_SetSensorsEnabled(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, SDL_bool enabled)
{
    SDL_Log("HIDAPI Steam Deck set sensors enabled called\n");
    // XXX: nothing to do here for wired ?
#if 0
    SDL_DriverSteam_Context *ctx = (SDL_DriverSteam_Context *)device->context;
    unsigned char buf[65];
    int nSettings = 0;

    SDL_memset(buf, 0, 65);
    buf[1] = ID_SET_SETTINGS_VALUES;
    if (enabled) {
        ADD_SETTING(SETTING_GYRO_MODE, 0x18 /* SETTING_GYRO_SEND_RAW_ACCEL | SETTING_GYRO_MODE_SEND_RAW_GYRO */);
    } else {
        ADD_SETTING(SETTING_GYRO_MODE, 0x00 /* SETTING_GYRO_MODE_OFF */);
    }
    buf[2] = (unsigned char)(nSettings * 3);
    if (SetFeatureReport(device->dev, buf, 3 + nSettings * 3) < 0) {
        return SDL_SetError("Couldn't write feature report");
    }

    ctx->report_sensors = enabled;
#endif
    return 0;
}

static void HIDAPI_DriverSteamDeck_CloseJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    // nothing to do here, kernel automatically switches back to lizard mode
}

static void HIDAPI_DriverSteamDeck_FreeDevice(SDL_HIDAPI_Device *device)
{
}

SDL_HIDAPI_DeviceDriver SDL_HIDAPI_DriverSteamDeck = {
    SDL_HINT_JOYSTICK_HIDAPI_STEAMDECK,
    SDL_TRUE,
    HIDAPI_DriverSteamDeck_RegisterHints,
    HIDAPI_DriverSteamDeck_UnregisterHints,
    HIDAPI_DriverSteamDeck_IsEnabled,
    HIDAPI_DriverSteamDeck_IsSupportedDevice,
    HIDAPI_DriverSteamDeck_InitDevice,
    HIDAPI_DriverSteamDeck_GetDevicePlayerIndex,
    HIDAPI_DriverSteamDeck_SetDevicePlayerIndex,
    HIDAPI_DriverSteamDeck_UpdateDevice,
    HIDAPI_DriverSteamDeck_OpenJoystick,
    HIDAPI_DriverSteamDeck_RumbleJoystick,
    HIDAPI_DriverSteamDeck_RumbleJoystickTriggers,
    HIDAPI_DriverSteamDeck_GetJoystickCapabilities,
    HIDAPI_DriverSteamDeck_SetJoystickLED,
    HIDAPI_DriverSteamDeck_SendJoystickEffect,
    HIDAPI_DriverSteamDeck_SetSensorsEnabled,
    HIDAPI_DriverSteamDeck_CloseJoystick,
    HIDAPI_DriverSteamDeck_FreeDevice,
};

#endif /* SDL_JOYSTICK_HIDAPI_STEAMDECK */

#endif /* SDL_JOYSTICK_HIDAPI */

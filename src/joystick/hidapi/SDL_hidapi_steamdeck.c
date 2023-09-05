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

#include "SDL_events.h"
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

#include "steam/controller_structs.h"

typedef struct
{
    Uint32 update_rate_in_us;
    Uint32 sensor_timestamp_us;
    Uint64 last_button_state;
} SDL_DriverSteamDeck_Context;

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

|01000940 fd9a0000 00000000 00000000| ...@............ 00000000
|00000000 00000000 c1005903 7e400000| ..........Y.~@.. 00000010
|0100ffff c8aeeafc 8efd289d 00000000| ..........(..... 00000020
|31042f08 f7053807 00000000 00000000| 1./...8......... 00000030
                                                       00000040

#endif

#define STEAMDECK_BUTTON_RT2 (1 << 0)
#define STEAMDECK_BUTTON_LT2 (1 << 1)
#define STEAMDECK_BUTTON_RT (1 << 2)
#define STEAMDECK_BUTTON_LT (1 << 3)
#define STEAMDECK_BUTTON_Y (1 << 4)
#define STEAMDECK_BUTTON_B (1 << 5)
#define STEAMDECK_BUTTON_X (1 << 6)
#define STEAMDECK_BUTTON_A (1 << 7)
#define STEAMDECK_BUTTON_DPAD_UP (1 << 8)
#define STEAMDECK_BUTTON_DPAD_RIGHT (1 << 9)
#define STEAMDECK_BUTTON_DPAD_LEFT (1 << 10)
#define STEAMDECK_BUTTON_DPAD_DOWN (1 << 11)
#define STEAMDECK_BUTTON_SELECT (1 << 12)
#define STEAMDECK_BUTTON_MODE (1 << 13)
#define STEAMDECK_BUTTON_START (1 << 14)
#define STEAMDECK_BUTTON_PADDLE3 (1 << 15)
#define STEAMDECK_BUTTON_PADDLE4 (1 << 16)
#define STEAMDECK_BUTTON_PADL (1 << 17)
#define STEAMDECK_BUTTON_PADR (1 << 18)
#define STEAMDECK_BUTTON_STICKL (1 << 22)
#define STEAMDECK_BUTTON_STICKR (1 << 26)
#define STEAMDECK_BUTTON_PADDLE1 0x20000000000
#define STEAMDECK_BUTTON_PADDLE2 0x50000000000
#define STEAMDECK_BUTTON_BASE 0x4000000000000

enum
{
    SDL_STEAMDECK_BUTTON_BASE = SDL_CONTROLLER_BUTTON_MISC1 + 1,
    SDL_STEAMDECK_BUTTON_PADDLE1,
    SDL_STEAMDECK_BUTTON_PADDLE2,
    SDL_STEAMDECK_BUTTON_PADDLE3,
    SDL_STEAMDECK_BUTTON_PADDLE4,
};

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
    int size;
    Uint8 data[64];
    SDL_DriverSteamDeck_Context *ctx;

    ctx = (SDL_DriverSteamDeck_Context *)SDL_calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        SDL_OutOfMemory();
        return SDL_FALSE;
    }
    device->context = ctx;

    // Read a report to see if this is the correct endpoint.
    // Mouse, Keyboard and Controller have the same VID/PID but
    // only the controller hidraw device receives hid reports.
    size = SDL_hid_read_timeout(device->dev, data, sizeof(data), 16);
    if (size == 0)
        return SDL_FALSE;

    // TODO: disable lizard mode

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
    SDL_DriverSteamDeck_Context *ctx = (SDL_DriverSteamDeck_Context *)device->context;
    SDL_Joystick *joystick = NULL;
    int r;
    uint8_t data[64];
    float values[3];
    ValveInReport_t *pInReport = (ValveInReport_t *)data;

    if (device->num_joysticks > 0) {
        joystick = SDL_JoystickFromInstanceID(device->joysticks[0]);
        if (joystick == NULL) {
            SDL_Log("SteamDeck_UpdateDevice() joystick is NULL\n");
            return SDL_FALSE;
        }
    } else {
        SDL_Log("SteamDeck_UpdateDevice() no joystick\n");
        return SDL_FALSE;
    }

    SDL_memset(data, 0, sizeof(data));
    r = SDL_hid_read(device->dev, data, sizeof(data));
    //SDL_Log("SteamDeck_UpdateDevice() SDL_hid_read returned %d\n", r);
    if (r == 0) {
        return SDL_FALSE;
    } else if (r <= 0) {
        /* Failed to read from controller */
        HIDAPI_JoystickDisconnected(device, device->joysticks[0]);
        return SDL_FALSE;
    }

    // TODO: parse and validate packet
    if (!(r == 64 && pInReport->header.unReportVersion == k_ValveInReportMsgVersion
            && pInReport->header.ucType == ID_CONTROLLER_DECK_STATE
            && pInReport->header.ucLength == 64)) {
        SDL_Log("SteamDeck_UpdateDevice() received invalid packet: v: %d, t: %d, l: %d\n",
                pInReport->header.unReportVersion, pInReport->header.ucType, pInReport->header.ucLength);
        return SDL_FALSE;
    }

    //Uint64 timestamp = SDL_GetTicksNS();
    // TODO: paddles and BASE button do not work
    // TODO: send disable lizard mode packet every 200 ms

    if (pInReport->payload.deckState.ulButtons != ctx->last_button_state) {
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_A,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_A) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_B,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_B) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_X,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_X) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_Y,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_Y) ? SDL_PRESSED : SDL_RELEASED);

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_LT) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_RT) ? SDL_PRESSED : SDL_RELEASED);

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_BACK,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_SELECT) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_START,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_MODE) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_GUIDE,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_START) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_STEAMDECK_BUTTON_BASE,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_BASE) ? SDL_PRESSED : SDL_RELEASED);
        SDL_Log("misc is pressed: %ld\n", pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_BASE);

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_LEFTSTICK,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_STICKL) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_RIGHTSTICK,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_STICKR) ? SDL_PRESSED : SDL_RELEASED);

        SDL_PrivateJoystickButton(joystick, SDL_STEAMDECK_BUTTON_PADDLE1,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_PADDLE1) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_STEAMDECK_BUTTON_PADDLE2,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_PADDLE2) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_STEAMDECK_BUTTON_PADDLE3,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_PADDLE3) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_STEAMDECK_BUTTON_PADDLE4,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_PADDLE4) ? SDL_PRESSED : SDL_RELEASED);
        SDL_Log("paddle4 is pressed: %ld\n", pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_PADDLE4);

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_UP,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_DPAD_UP) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_DOWN,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_DPAD_DOWN) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_LEFT,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_DPAD_LEFT) ? SDL_PRESSED : SDL_RELEASED);
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_RIGHT,
            (pInReport->payload.deckState.ulButtons & STEAMDECK_BUTTON_DPAD_RIGHT) ? SDL_PRESSED : SDL_RELEASED);
        ctx->last_button_state = pInReport->payload.deckState.ulButtons;
    }

    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERLEFT,
        (int)pInReport->payload.deckState.sLeftTrigger * 2 - 32768);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERRIGHT,
        (int)pInReport->payload.deckState.sRightTrigger * 2 - 32768);

    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_LEFTX,
        pInReport->payload.deckState.sLeftStickX);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_LEFTY,
        -pInReport->payload.deckState.sLeftStickY);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTX,
        pInReport->payload.deckState.sRightStickX);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTY,
        -pInReport->payload.deckState.sRightStickY);

    ctx->sensor_timestamp_us += ctx->update_rate_in_us;

    values[0] = (pInReport->payload.deckState.sGyroX / 32768.0f) * (2000.0f * ((float)M_PI / 180.0f));
    values[1] = (pInReport->payload.deckState.sGyroZ / 32768.0f) * (2000.0f * ((float)M_PI / 180.0f));
    values[2] = (pInReport->payload.deckState.sGyroY / 32768.0f) * (2000.0f * ((float)M_PI / 180.0f));
    SDL_PrivateJoystickSensor(joystick, SDL_SENSOR_GYRO, ctx->sensor_timestamp_us, values, 3);

    values[0] = (pInReport->payload.deckState.sAccelX / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
    values[1] = (pInReport->payload.deckState.sAccelZ / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
    values[2] = (-pInReport->payload.deckState.sAccelY / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
    SDL_PrivateJoystickSensor(joystick, SDL_SENSOR_ACCEL, ctx->sensor_timestamp_us, values, 3);

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

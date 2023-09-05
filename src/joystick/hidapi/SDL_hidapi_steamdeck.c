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

#ifdef __WIN32__
    if (device->serial) {
        /* We get a garbage serial number on Windows */
        SDL_free(device->serial);
        device->serial = NULL;
    }
#endif /* __WIN32__ */

    HIDAPI_SetDeviceName(device, "Steam Controller");

    return HIDAPI_JoystickConnected(device, NULL);
#endif
    SDL_Log("HIDAPI Steam Deck initialize called\n");
    return SDL_FALSE;
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
#if 0
    SDL_DriverSteam_Context *ctx = (SDL_DriverSteam_Context *)device->context;
    SDL_Joystick *joystick = NULL;

    if (device->num_joysticks > 0) {
        joystick = SDL_GetJoystickFromInstanceID(device->joysticks[0]);
    } else {
        return SDL_FALSE;
    }

    for (;;) {
        uint8_t data[128];
        int r, nPacketLength;
        const Uint8 *pPacket;

        r = ReadSteamController(device->dev, data, sizeof(data));
        if (r == 0) {
            break;
        }

        if (joystick == NULL) {
            continue;
        }

        nPacketLength = 0;
        if (r > 0) {
            nPacketLength = WriteSegmentToSteamControllerPacketAssembler(&ctx->m_assembler, data, r);
        }

        pPacket = ctx->m_assembler.uBuffer;

        if (nPacketLength > 0 && UpdateSteamControllerState(pPacket, nPacketLength, &ctx->m_state)) {
            Uint64 timestamp = SDL_GetTicksNS();

            if (ctx->m_state.ulButtons != ctx->m_last_state.ulButtons) {
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
            }

            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFT_TRIGGER, (int)ctx->m_state.sTriggerL * 2 - 32768);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, (int)ctx->m_state.sTriggerR * 2 - 32768);

            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFTX, ctx->m_state.sLeftStickX);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_LEFTY, ~ctx->m_state.sLeftStickY);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHTX, ctx->m_state.sRightPadX);
            SDL_SendJoystickAxis(timestamp, joystick, SDL_GAMEPAD_AXIS_RIGHTY, ~ctx->m_state.sRightPadY);

            if (ctx->report_sensors) {
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
            }

            ctx->m_last_state = ctx->m_state;
        }

        if (r <= 0) {
            /* Failed to read from controller */
            HIDAPI_JoystickDisconnected(device, device->joysticks[0]);
            return SDL_FALSE;
        }
    }
    return SDL_TRUE;
#endif
    return SDL_FALSE;
}

static SDL_bool HIDAPI_DriverSteamDeck_OpenJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
#if 0
    SDL_DriverSteam_Context *ctx = (SDL_DriverSteam_Context *)device->context;
    float update_rate_in_hz = 0.0f;

    SDL_AssertJoysticksLocked();

    ctx->report_sensors = SDL_FALSE;
    SDL_zero(ctx->m_assembler);
    SDL_zero(ctx->m_state);
    SDL_zero(ctx->m_last_state);

    if (!ResetSteamController(device->dev, false, &ctx->update_rate_in_us)) {
        SDL_SetError("Couldn't reset controller");
        return SDL_FALSE;
    }
    if (ctx->update_rate_in_us > 0) {
        update_rate_in_hz = 1000000.0f / ctx->update_rate_in_us;
    }

    InitializeSteamControllerPacketAssembler(&ctx->m_assembler);

    /* Initialize the joystick capabilities */
    joystick->nbuttons = 17;
    joystick->naxes = SDL_GAMEPAD_AXIS_MAX;

    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_GYRO, update_rate_in_hz);
    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_ACCEL, update_rate_in_hz);

    return SDL_TRUE;
#endif
    return SDL_FALSE;
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
    // CloseSteamController(device->dev);
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

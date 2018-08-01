
/*
Copyright <2018> <Elliott Wen>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <stdint.h>
#include <linux/input.h>

#ifndef MAIN_H
#define MAIN_H

#define XHEIGHT 960
#define XWIDTH 640
#define XSTRIDE 640*4
#define GRAPHIC_EVENT_TYPE 18391
#define rmask 0x000000ff
#define gmask 0x0000ff00
#define bmask 0x00ff0000
#define amask 0xff000000
#define MAINWORKDIR "/tmp/android-dbus/"
#define INPUTWORKDIR "/tmp/android-dbus/input"
#define OPENGL_SOCKET "/tmp/android-dbus/opengl"
#define MOUSE_SOCKET "/tmp/android-dbus/input/input0"
#define KEYBOARD_SOCKET "/tmp/android-dbus/input/input1"
#define AUDIO_PLAY_SOCKET "/tmp/android-dbus/alsa_out"
#define GPS_SOCKET "/tmp/android-dbus/gps"

struct DeviceInfo {
    char name[128];
    int driver_version;
    struct input_id id;
    char physical_location[128];
    char unique_id[128];
    uint8_t key_bitmask[(KEY_MAX + 1) / 8];
    uint8_t abs_bitmask[(ABS_MAX + 1) / 8];
    uint8_t rel_bitmask[(REL_MAX + 1) / 8];
    uint8_t sw_bitmask[(SW_MAX + 1) / 8];
    uint8_t led_bitmask[(LED_MAX + 1) / 8];
    uint8_t ff_bitmask[(FF_MAX + 1) / 8];
    uint8_t prop_bitmask[(INPUT_PROP_MAX + 1) / 8];
    uint32_t abs_max[ABS_CNT];
    uint32_t abs_min[ABS_CNT];
};

struct CompatEvent{
    unsigned long sec;
    unsigned long usec;
    unsigned short type;
    unsigned short code;
    unsigned int value;
};

// unsigned short convert_keycode(const SDL_Scancode &scan_code);
int _create_unix_server(const char *path);
int _mkdir_dir(const char *dir);
void set_bit(uint8_t* array, unsigned int bit);
int _recvfd(int s);


#endif
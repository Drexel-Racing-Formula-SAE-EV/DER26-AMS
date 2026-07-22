/* Compile the real ESP32 dashboard translation unit against host stubs and
 * expose its static JSON renderer for structural validation. */

#include "esp_test_stubs.h"
#include "../main/ams_can_decode.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define static
#define app_main dashboard_app_main_for_test
#include "../main/main.c"
#undef app_main
#undef static

int main(void)
{
    ams_dash_state_t state;
    char json[DASH_JSON_BUF_BYTES];
    ams_dash_state_init(&state);
    build_json_state(json, sizeof(json), &state);
    if((json[0] == '\0') || (fputs(json, stdout) == EOF))
    {
        return 1;
    }
    return 0;
}

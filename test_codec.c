#include "sparkmax.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

int main(void)
{
    /* ID packing matches the community frame map. */
    CHECK(sm_make_id(SM_API_HEARTBEAT, 0)          == 0x02052C80u);
    CHECK(sm_make_id(SM_API_SETPOINT_DUTY, 1)      == 0x02050081u);
    CHECK(sm_make_id(SM_API_SETPOINT_VELOCITY, 0)  == 0x02050480u);
    CHECK(sm_make_id(SM_API_SETPOINT_SMART_VEL, 0) == 0x020504C0u);
    CHECK(sm_make_id(SM_API_SETPOINT_POSITION, 0)  == 0x02050C80u);
    CHECK(sm_make_id(SM_API_SETPOINT_VOLTAGE, 0)   == 0x02051080u);
    CHECK(sm_make_id(SM_API_SETPOINT_CURRENT, 0)   == 0x020510C0u);
    CHECK(sm_make_id(SM_API_SETPOINT_SMART_MOT, 0) == 0x02051480u);
    CHECK(sm_make_id(SM_API_STATUS_0, 0)           == 0x02051800u);
    CHECK(sm_make_id(SM_API_STATUS_2, 7)           == 0x02051887u);

    sm_frc_id s = sm_split_id(0x02051847u);
    CHECK(s.devtype == 2 && s.mfr == 5 && s.api_class == 6 && s.api_index == 1 && s.device == 7);

    /* Setpoint: float LE, clamp, NaN, device range. */
    sm_can_frame f;
    CHECK(sm_encode_setpoint(&f, 3, SM_API_SETPOINT_DUTY, 0.5f, 0, 1) == SM_OK);
    CHECK(f.id == 0x02050083u && f.extended && f.dlc == 8);
    CHECK(f.data[0] == 0x00 && f.data[1] == 0x00 && f.data[2] == 0x00 && f.data[3] == 0x3F);
    CHECK(f.data[6] == 1);
    CHECK(sm_encode_setpoint(&f, 3, SM_API_SETPOINT_DUTY, 7.0f, 0, 0) == SM_OK);
    CHECK(sm_get_f32le(f.data) == 1.0f);
    CHECK(sm_encode_setpoint(&f, 3, SM_API_SETPOINT_DUTY, NAN, 0, 0) == SM_EINVAL);
    CHECK(sm_encode_setpoint(&f, 0, SM_API_SETPOINT_DUTY, 0.1f, 0, 0) == SM_EINVAL);
    CHECK(sm_encode_setpoint(&f, 63, SM_API_SETPOINT_DUTY, 0.1f, 0, 0) == SM_EINVAL);

    /* Heartbeat mask */
    uint8_t m[8] = {0};
    m[5 >> 3] |= 1u << (5 & 7);
    sm_encode_heartbeat(&f, m);
    CHECK(f.id == 0x02052C80u && f.data[0] == 0x20);

    /* Status 1: vel=1500 rpm, T=40C, V=12.5 (1600), I=3.25 (104) */
    memset(&f, 0, sizeof f);
    f.id = sm_make_id(SM_API_STATUS_1, 9); f.extended = 1; f.dlc = 8;
    sm_put_f32le(f.data, 1500.0f);
    f.data[4] = 40;
    uint16_t v12 = 1600, c12 = 104;
    f.data[5] = (uint8_t)(v12 & 0xFF);
    f.data[6] = (uint8_t)(((v12 >> 8) & 0x0F) | ((c12 & 0x0F) << 4));
    f.data[7] = (uint8_t)(c12 >> 4);
    sm_status1 s1;
    CHECK(sm_decode_status1(&f, &s1) == SM_OK);
    CHECK(s1.velocity_rpm == 1500.0f && s1.temperature_c == 40);
    CHECK(fabsf(s1.bus_voltage - 12.5f) < 1e-4f && fabsf(s1.output_current - 3.25f) < 1e-4f);
    CHECK(sm_decode_status0(&f, (sm_status0 *)&s1) == SM_EINVAL);   /* wrong API rejected */

    /* Status 0: applied output -0.5 */
    memset(&f, 0, sizeof f);
    f.id = sm_make_id(SM_API_STATUS_0, 9); f.extended = 1; f.dlc = 8;
    f.data[0] = 0x00; f.data[1] = 0xC0;   /* -16384 */
    sm_status0 s0;
    CHECK(sm_decode_status0(&f, &s0) == SM_OK && s0.applied_output == -0.5f);

    /* SLCAN round trip */
    char line[32];
    CHECK(sm_encode_setpoint(&f, 12, SM_API_SETPOINT_DUTY, 0.25f, 0, 0) == SM_OK);
    int n = slcan_format_frame(&f, line, sizeof line);
    CHECK(n == 27 && strcmp(line, "T0205008C80000803E00000000\r") == 0);
    sm_can_frame g;
    CHECK(slcan_parse_frame(line, n - 1, &g) == SM_OK);
    CHECK(g.id == f.id && g.extended && g.dlc == 8 && memcmp(g.data, f.data, 8) == 0);
    CHECK(slcan_parse_frame("T0205008C80000803E00000000EA5F", 30, &g) == SM_OK);  /* w/ timestamp */
    CHECK(slcan_parse_frame("t12320102", 9, &g) == SM_OK && g.id == 0x123 && !g.extended && g.dlc == 2);
    CHECK(slcan_parse_frame("T0205008C9", 10, &g) == SM_EPARSE);    /* dlc > 8 */
    CHECK(slcan_parse_frame("T0205008C800", 12, &g) == SM_EPARSE);  /* short */
    CHECK(slcan_parse_frame("t8001", 5, &g) == SM_EPARSE);          /* 0x800 > 11 bit */
    CHECK(slcan_parse_frame("TG205008C0", 10, &g) == SM_EPARSE);    /* bad hex */

    if (!fails) puts("all codec tests passed");
    return fails ? 1 : 0;
}

#include "pid_tuning.h"

#include "Control.h"
#include "Encoder.h"
#include "pid.h"
#include "uart.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *name;
    PidTuningCommandId id;
    float minimum;
    float maximum;
    bool integral;
} PidTuningValueSpec;

static const PidTuningValueSpec g_valueSpecs[] = {
    { "BS",    PID_TUNE_CMD_BS,    0.0f,  20.0f, true  },
    { "TP",    PID_TUNE_CMD_TP,    0.0f,  10.0f, false },
    { "TI",    PID_TUNE_CMD_TI,    0.0f,   2.0f, false },
    { "TD",    PID_TUNE_CMD_TD,    0.0f,  10.0f, false },
    { "LP",    PID_TUNE_CMD_LP,    0.0f, 200.0f, false },
    { "LI",    PID_TUNE_CMD_LI,    0.0f,  10.0f, false },
    { "RP",    PID_TUNE_CMD_RP,    0.0f, 200.0f, false },
    { "RI",    PID_TUNE_CMD_RI,    0.0f,  10.0f, false },
    { "RUN",   PID_TUNE_CMD_RUN,   0.0f,   1.0f, true  },
    { "TELEM", PID_TUNE_CMD_TELEM, 0.0f,   1.0f, true  },
};

static bool g_telemetryEnabled;
static uint32_t g_nextTelemetryMs;

static const char *skip_space(const char *text)
{
    while ((*text != '\0') && isspace((unsigned char)*text)) {
        text++;
    }
    return text;
}

static const char *trimmed_end(const char *begin)
{
    const char *end = begin + strlen(begin);

    while ((end > begin) && isspace((unsigned char)end[-1])) {
        end--;
    }
    return end;
}

static bool token_equals(
    const char *begin, const char *end, const char *expected)
{
    size_t expectedLength = strlen(expected);
    size_t tokenLength = (size_t)(end - begin);
    size_t index;

    if (tokenLength != expectedLength) {
        return false;
    }
    for (index = 0U; index < tokenLength; index++) {
        if (toupper((unsigned char)begin[index]) !=
            toupper((unsigned char)expected[index])) {
            return false;
        }
    }
    return true;
}

static const PidTuningValueSpec *find_value_spec(
    const char *begin, const char *end)
{
    size_t index;

    for (index = 0U;
         index < (sizeof(g_valueSpecs) / sizeof(g_valueSpecs[0]));
         index++) {
        if (token_equals(begin, end, g_valueSpecs[index].name)) {
            return &g_valueSpecs[index];
        }
    }
    return NULL;
}

PidTuningParseStatus pid_tuning_parse(
    const char *line, PidTuningCommand *command)
{
    const char *begin;
    const char *end;
    const char *separator;
    const char *nameEnd;
    const char *valueBegin;
    const PidTuningValueSpec *spec;
    char *valueEnd;
    float value;

    if ((line == NULL) || (command == NULL)) {
        return PID_TUNE_PARSE_ERR_VALUE;
    }

    begin = skip_space(line);
    end = trimmed_end(begin);
    separator = memchr(begin, '=', (size_t)(end - begin));
    if (separator == NULL) {
        if (token_equals(begin, end, "GET")) {
            command->id = PID_TUNE_CMD_GET;
            command->value = 0.0f;
            command->hasValue = false;
            return PID_TUNE_PARSE_OK;
        }
        if (token_equals(begin, end, "ZERO")) {
            command->id = PID_TUNE_CMD_ZERO;
            command->value = 0.0f;
            command->hasValue = false;
            return PID_TUNE_PARSE_OK;
        }
        return PID_TUNE_PARSE_ERR_CMD;
    }

    nameEnd = separator;
    while ((nameEnd > begin) && isspace((unsigned char)nameEnd[-1])) {
        nameEnd--;
    }
    spec = find_value_spec(begin, nameEnd);
    if (spec == NULL) {
        return PID_TUNE_PARSE_ERR_CMD;
    }

    valueBegin = skip_space(separator + 1);
    if (valueBegin >= end) {
        return PID_TUNE_PARSE_ERR_VALUE;
    }

    errno = 0;
    value = strtof(valueBegin, &valueEnd);
    if ((valueEnd == valueBegin) || (errno == ERANGE) || !isfinite(value)) {
        return PID_TUNE_PARSE_ERR_VALUE;
    }
    valueEnd = (char *)skip_space(valueEnd);
    if (valueEnd != end) {
        return PID_TUNE_PARSE_ERR_VALUE;
    }
    if ((value < spec->minimum) || (value > spec->maximum)) {
        return PID_TUNE_PARSE_ERR_RANGE;
    }
    if (spec->integral && (value != (float)(int32_t)value)) {
        return PID_TUNE_PARSE_ERR_VALUE;
    }

    command->id = spec->id;
    command->value = value;
    command->hasValue = true;
    return PID_TUNE_PARSE_OK;
}

static void format_milli(char *output, size_t capacity, float value)
{
    int32_t scaled = (int32_t)((value * 1000.0f) +
        ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;

    (void)snprintf(output, capacity, "%s%lu.%03lu",
        (scaled < 0) ? "-" : "",
        (unsigned long)(magnitude / 1000U),
        (unsigned long)(magnitude % 1000U));
}

static const char *command_name(PidTuningCommandId id)
{
    size_t index;

    for (index = 0U;
         index < (sizeof(g_valueSpecs) / sizeof(g_valueSpecs[0]));
         index++) {
        if (g_valueSpecs[index].id == id) {
            return g_valueSpecs[index].name;
        }
    }
    return "";
}

static void send_error(PidTuningParseStatus status)
{
    switch (status) {
    case PID_TUNE_PARSE_ERR_CMD:
        uart_communicate_printf("ERR CMD\r\n");
        break;
    case PID_TUNE_PARSE_ERR_RANGE:
        uart_communicate_printf("ERR RANGE\r\n");
        break;
    default:
        uart_communicate_printf("ERR VALUE\r\n");
        break;
    }
}

static void send_gain_ok(const char *name, float value)
{
    char rendered[16];

    format_milli(rendered, sizeof(rendered), value);
    uart_communicate_printf("OK %s=%s\r\n", name, rendered);
}

static void send_parameters(void)
{
    char tp[16], ti[16], td[16], lp[16], li[16], rp[16], ri[16];

    format_milli(tp, sizeof(tp), PID_AngleTurn.kp);
    format_milli(ti, sizeof(ti), PID_AngleTurn.ki);
    format_milli(td, sizeof(td), PID_AngleTurn.kd);
    format_milli(lp, sizeof(lp), PID_Speed_bL.kp);
    format_milli(li, sizeof(li), PID_Speed_bL.ki);
    format_milli(rp, sizeof(rp), PID_Speed_bR.kp);
    format_milli(ri, sizeof(ri), PID_Speed_bR.ki);
    uart_communicate_printf(
        "P,BS=%d,TP=%s,TI=%s,TD=%s,LP=%s,LI=%s,RP=%s,RI=%s,RUN=%u,TELEM=%u\r\n",
        basicSpeed, tp, ti, td, lp, li, rp, ri,
        start_run ? 1U : 0U, g_telemetryEnabled ? 1U : 0U);
}

static void apply_command(const PidTuningCommand *command, uint32_t nowMs)
{
    const char *name = command_name(command->id);

    switch (command->id) {
    case PID_TUNE_CMD_GET:
        send_parameters();
        return;
    case PID_TUNE_CMD_ZERO:
        PidResetState(&PID_AngleTurn);
        PidResetState(&PID_Speed_bL);
        PidResetState(&PID_Speed_bR);
        uart_communicate_printf("OK ZERO\r\n");
        return;
    case PID_TUNE_CMD_BS:
        basicSpeed = (int16_t)command->value;
        uart_communicate_printf("OK BS=%d\r\n", basicSpeed);
        return;
    case PID_TUNE_CMD_TP:
        PID_AngleTurn.kp = command->value;
        break;
    case PID_TUNE_CMD_TI:
        PID_AngleTurn.ki = command->value;
        break;
    case PID_TUNE_CMD_TD:
        PID_AngleTurn.kd = command->value;
        break;
    case PID_TUNE_CMD_LP:
        PID_Speed_bL.kp = command->value;
        break;
    case PID_TUNE_CMD_LI:
        PID_Speed_bL.ki = command->value;
        break;
    case PID_TUNE_CMD_RP:
        PID_Speed_bR.kp = command->value;
        break;
    case PID_TUNE_CMD_RI:
        PID_Speed_bR.ki = command->value;
        break;
    case PID_TUNE_CMD_RUN:
        control_set_run(command->value == 1.0f);
        uart_communicate_printf("OK RUN=%u\r\n", start_run ? 1U : 0U);
        return;
    case PID_TUNE_CMD_TELEM:
        g_telemetryEnabled = (command->value == 1.0f);
        g_nextTelemetryMs = nowMs;
        uart_communicate_printf("OK TELEM=%u\r\n",
            g_telemetryEnabled ? 1U : 0U);
        return;
    default:
        uart_communicate_printf("ERR CMD\r\n");
        return;
    }

    send_gain_ok(name, command->value);
}

static void send_telemetry(void)
{
    uart_communicate_printf(
        "T,err=%d,turn=%d,encL=%d,encR=%d,desL=%d,desR=%d,pwmL=%d,pwmR=%d\r\n",
        track_error, track_turn, encoder_bl_speed, encoder_br_speed,
        control_desired_left, control_desired_right,
        control_pwm_left, control_pwm_right);
}

void pid_tuning_init(void)
{
    g_telemetryEnabled = false;
    g_nextTelemetryMs = 0U;
}

void pid_tuning_task(uint32_t nowMs)
{
    /* UART 文本调参已停用；保留旧解析代码仅供离线参考。 */
    (void)nowMs;
}

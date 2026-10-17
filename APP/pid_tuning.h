#ifndef PID_TUNING_H_
#define PID_TUNING_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PID_TUNE_CMD_GET,
    PID_TUNE_CMD_ZERO,
    PID_TUNE_CMD_BS,
    PID_TUNE_CMD_TP,
    PID_TUNE_CMD_TI,
    PID_TUNE_CMD_TD,
    PID_TUNE_CMD_LP,
    PID_TUNE_CMD_LI,
    PID_TUNE_CMD_RP,
    PID_TUNE_CMD_RI,
    PID_TUNE_CMD_RUN,
    PID_TUNE_CMD_TELEM,
} PidTuningCommandId;

typedef enum {
    PID_TUNE_PARSE_OK,
    PID_TUNE_PARSE_ERR_CMD,
    PID_TUNE_PARSE_ERR_VALUE,
    PID_TUNE_PARSE_ERR_RANGE,
} PidTuningParseStatus;

typedef struct {
    PidTuningCommandId id;
    float value;
    bool hasValue;
} PidTuningCommand;

PidTuningParseStatus pid_tuning_parse(
    const char *line, PidTuningCommand *command);
void pid_tuning_init(void);
void pid_tuning_task(uint32_t nowMs);

#endif /* PID_TUNING_H_ */

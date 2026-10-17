#ifndef PARKING_H
#define PARKING_H

#include <stdbool.h>
#include <stdint.h>

#define PARK_TARGET_ACTIVE_COUNT          4U
#define PARK_SLOW_TARGET_MIN_ACTIVE_COUNT 3U
#define PARK_LEAVE_CONFIRM_TICKS          5U
#define PARK_FORWARD_CONFIRM_TICKS        3U
#define PARK_BRAKE_HOLD_MS                100U

typedef enum {
    PARK_STATE_IDLE = 0,
    PARK_STATE_WAIT_LEAVE_A,
    PARK_STATE_RUNNING,
    PARK_STATE_BRAKE_CHECK,
    PARK_STATE_STOPPED,
    PARK_STATE_FAILED
} ParkingState;

typedef enum {
    PARK_ACTION_FORWARD = 0,
    PARK_ACTION_STOP
} ParkingAction;

typedef struct {
    volatile ParkingState state;
    volatile uint32_t runStartMs;
    volatile uint32_t stateStartMs;
    volatile uint32_t elapsedMs;
    uint8_t leaveConfirmTicks;
    uint8_t forwardConfirmTicks;
} ParkingController;

void Parking_init(ParkingController *controller);
void Parking_start(ParkingController *controller, uint32_t nowMs);
void Parking_cancel(ParkingController *controller);
ParkingAction Parking_update(
    ParkingController *controller,
    uint8_t activeCount,
    bool relaxedTarget,
    uint32_t nowMs);
bool Parking_hasResult(const ParkingController *controller);
bool Parking_succeeded(const ParkingController *controller);

#endif /* PARKING_H */

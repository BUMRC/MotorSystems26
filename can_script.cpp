#include <stdint.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_CAN_PAYLOAD   8
#define RPM_LIMIT         100000.0f
#define COMMAND_PERIOD_US  1000

typedef enum {
    MODE_DUTY_CYCLE      = 0,
    MODE_CURRENT         = 1,
    MODE_CURRENT_BRAKE   = 2,
    MODE_RPM             = 3,
    MODE_POSITION        = 4,
    MODE_SET_ORIGIN      = 5,
    MODE_POSITION_SPEED  = 6
} MotorCommandMode;

typedef struct {
    int socket_fd;
    const char *interface_name;
} CanBus;

typedef struct {
    CanBus *bus;
    uint8_t controller_id;
} Motor;

typedef struct {
    Motor left;
    Motor right;
} DriveSide;

static void append_int32_big_endian(uint8_t *buffer, int32_t value, int32_t *offset) {
    buffer[(*offset)++] = (value >> 24) & 0xFF;
    buffer[(*offset)++] = (value >> 16) & 0xFF;
    buffer[(*offset)++] = (value >> 8)  & 0xFF;
    buffer[(*offset)++] = (value)       & 0xFF;
}

static float clamp_float(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static bool can_bus_init(CanBus *bus) {
    bus->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (bus->socket_fd < 0) {
        perror("socket creation failed");
        return false;
    }

    struct ifreq interface_request;
    strncpy(interface_request.ifr_name, bus->interface_name, IFNAMSIZ - 1);
    interface_request.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(bus->socket_fd, SIOCGIFINDEX, &interface_request) < 0) {
        perror("ioctl failed");
        return false;
    }

    struct sockaddr_can bind_address = {
        .can_family  = AF_CAN,
        .can_ifindex = interface_request.ifr_ifindex
    };

    if (bind(bus->socket_fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
        perror("bind failed");
        return false;
    }

    printf("CAN bus initialized: %s\n", bus->interface_name);
    return true;
}

static bool can_transmit_extended(CanBus *bus, uint32_t extended_id, const uint8_t *data, uint8_t length) {
    if (length > MAX_CAN_PAYLOAD)
        length = MAX_CAN_PAYLOAD;

    struct can_frame frame = {0};
    frame.can_id  = CAN_EFF_FLAG | extended_id;
    frame.can_dlc = length;
    memcpy(frame.data, data, length);

    int bytes_written = write(bus->socket_fd, &frame, sizeof(frame));
    if (bytes_written != sizeof(frame)) {
        perror("CAN write failed");
        return false;
    }
    return true;
}

static uint32_t build_motor_can_id(MotorCommandMode mode, uint8_t controller_id) {
    return controller_id | ((uint32_t)mode << 8);
}

static bool motor_set_rpm(Motor *motor, float rpm) {
    rpm = clamp_float(rpm, -RPM_LIMIT, RPM_LIMIT);

    int32_t write_offset = 0;
    uint8_t payload[4];
    append_int32_big_endian(payload, (int32_t)rpm, &write_offset);

    uint32_t can_id = build_motor_can_id(MODE_RPM, motor->controller_id);
    return can_transmit_extended(motor->bus, can_id, payload, write_offset);
}

static void drive_set_rpm(DriveSide *side, float left_rpm, float right_rpm) {
    motor_set_rpm(&side->left, left_rpm);
    motor_set_rpm(&side->right, right_rpm);
}

int main(void) {
    CanBus bus_left  = { .interface_name = "can0" };
    CanBus bus_right = { .interface_name = "can1" };

    if (!can_bus_init(&bus_left) || !can_bus_init(&bus_right))
        return EXIT_FAILURE;

    // Two motors per bus; motors on the same bus share IDs across buses
    DriveSide left_side = {
        .left  = { .bus = &bus_left,  .controller_id = 1 },
        .right = { .bus = &bus_left,  .controller_id = 2 }
    };

    DriveSide right_side = {
        .left  = { .bus = &bus_right, .controller_id = 1 },
        .right = { .bus = &bus_right, .controller_id = 2 }
    };

    float target_rpm = 5000.0f;

    while (1) {
        drive_set_rpm(&left_side,  target_rpm, target_rpm);
        drive_set_rpm(&right_side, target_rpm, target_rpm);
        usleep(COMMAND_PERIOD_US);
    }

    return EXIT_SUCCESS;
}

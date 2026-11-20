"""Execute only host C with UART/GPIO/RTOS stubs; never build/run firmware or app.c."""
from hashlib import sha256
import os
from pathlib import Path
import shutil
import subprocess

import pytest

DEFAULT_REFERENCE = (
    "/home/ubuntu/zsy_projects/real_operation_references/supplied_bundle_0caac47f2ab0/"
    "unpacked/jetrover_stm32_reference_20261120/servo_read_example/"
    "RosRobotControllerM4_serial_servo_read"
)
REFERENCE_HASHES = {
    "Hiwonder/Portings/serial_servo.h": "cb4dfbf4bc655efcab235ad0a74f86d83fc9228fcf3a394816f5008b396408b5",
    "Hiwonder/Peripherals/serial_servo.c": "80443ffc2e456a47f65e7accdab1a79ae72ae6406a5f446802b6f3dc899f026d",
    "Hiwonder/Portings/serial_servo_porting.c": "fd722c3a724d5038a9ced66803dabfa62d6630426c7b45ee143691ceaa1a1288",
    "Hiwonder/System/packet_handle.c": "8551fcd19d5786b3d666fc48de04941074c64bfeced156a604bca87a3875903f",
}

HARNESS = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial_servo.h"
#include "cmsis_os2.h"

int huart6;
osSemaphoreId_t serial_servo_rx_completeHandle = 1;
extern SerialServoControllerTypeDef serial_servo_controller;
extern void serial_servo_init(void);
static int completion_tokens = 0;

int osSemaphoreAcquire(osSemaphoreId_t id, unsigned int ticks) {
    (void)id; (void)ticks;
    if (completion_tokens) { completion_tokens--; return osOK; }
    return -1;
}
static int unmatched_reply(SerialServoControllerTypeDef *c,
                           SerialServoCmdTypeDef *request, bool tx_only) {
    (void)request; (void)tx_only;
    c->rx_frame.elements.servo_id = 2;
    c->rx_frame.elements.command = SERIAL_SERVO_VIN_READ;
    c->rx_frame.elements.args[0] = 9;
    c->rx_frame.elements.args[1] = 3; /* 777 */
    return 0;
}
int main(int argc, char **argv) {
    if (argc < 2) return 2;
    if (!strcmp(argv[1], "parser")) {
        if (argc != 5) return 2;
        SerialServoControllerTypeDef c;
        serial_servo_controller_object_init(&c);
        c.tx_frame.elements.servo_id = 1;
        c.tx_frame.elements.command = SERIAL_SERVO_POS_READ;
        SerialServoCmdTypeDef reply = {0};
        reply.header_1 = reply.header_2 = 0x55;
        reply.elements.servo_id = atoi(argv[2]);
        reply.elements.length = 5;
        reply.elements.command = atoi(argv[3]);
        reply.elements.args[0] = 244; reply.elements.args[1] = 1;
        reply.elements.args[2] = serial_servo_checksum((uint8_t *)&reply) ^ atoi(argv[4]);
        int ret = -1000;
        for (int i = 0; i < 8; i++)
            ret = serial_servo_rx_handler(&c, ((uint8_t *)&reply)[i]);
        printf("%d %u %u\n", ret, c.rx_frame.elements.servo_id, c.rx_frame.elements.command);
        return 0;
    }
    if (!strcmp(argv[1], "unmatched_position")) {
        SerialServoControllerTypeDef c;
        serial_servo_controller_object_init(&c);
        c.serial_write_and_read = unmatched_reply;
        int16_t position = -999;
        int ret = serial_servo_read_position(&c, 1, &position);
        printf("%d %d\n", ret, position);
        return 0;
    }
    if (!strcmp(argv[1], "late_completion")) {
        serial_servo_init();
        int16_t first_position = -999, second_position = -999;
        int first = serial_servo_read_position(&serial_servo_controller, 1, &first_position);
        /* Possible scheduling scenario: old RX frame and completion arrive
           after the first timeout. All hardware calls here are no-op stubs. */
        serial_servo_controller.rx_frame.elements.servo_id = 1;
        serial_servo_controller.rx_frame.elements.command = SERIAL_SERVO_POS_READ;
        serial_servo_controller.rx_frame.elements.args[0] = 111;
        serial_servo_controller.rx_frame.elements.args[1] = 0;
        completion_tokens = 1;
        int second = serial_servo_read_position(&serial_servo_controller, 2, &second_position);
        printf("%d %d %d %d\n", first, first_position, second, second_position);
        return 0;
    }
    return 2;
}
"""

STUBS = {
    "global.h": "/* Offline stub: no device declarations or initializers. */\n",
    "usart.h": "extern int huart6;\n",
    "gpio.h": """
#define SERIAL_SERVO_RX_EN_GPIO_Port 0
#define SERIAL_SERVO_TX_EN_GPIO_Port 0
#define SERIAL_SERVO_RX_EN_Pin 0
#define SERIAL_SERVO_TX_EN_Pin 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define HAL_GPIO_WritePin(port,pin,value) ((void)0)
""",
    "stm32f4xx_hal.h": """
#define UART_FLAG_RXNE 1
#define UART_FLAG_TC 2
#define UART_FLAG_TXE 3
#define UART_IT_TXE 1
#define UART_IT_TC 2
#define UART_IT_RXNE 3
#define __HAL_UART_CLEAR_FLAG(h,f) ((void)0)
#define __HAL_UART_ENABLE_IT(h,f) ((void)0)
""",
    "cmsis_os2.h": """
#ifndef OFFLINE_CMSIS_H
#define OFFLINE_CMSIS_H
typedef int osSemaphoreId_t;
#define osOK 0
int osSemaphoreAcquire(osSemaphoreId_t id, unsigned int ticks);
#endif
""",
}


@pytest.fixture(scope="module")
def reference_sources():
    root = Path(os.environ.get("JETROVER_STM32_REFERENCE_ROOT", DEFAULT_REFERENCE))
    if not root.is_dir():
        pytest.skip("supplied exact STM32 reference is required; not a hardware test")
    for name, expected in REFERENCE_HASHES.items():
        assert sha256((root/name).read_bytes()).hexdigest() == expected
    return root


@pytest.fixture(scope="module")
def reference_binary(reference_sources, tmp_path_factory):
    cc = shutil.which("cc")
    if not cc:
        pytest.skip("host C compiler unavailable; never install or use STM32 flashing tools")
    temp = tmp_path_factory.mktemp("r1_reference_no_hardware")
    for name, content in STUBS.items():
        (temp/name).write_text(content)
    source = temp/"harness.c"
    source.write_text(HARNESS)
    binary = temp/"reference_host_only"
    subprocess.run([
        cc, "-std=c11", "-I", str(temp),
        "-I", str(reference_sources/"Hiwonder/Portings"),
        str(source),
        str(reference_sources/"Hiwonder/Peripherals/serial_servo.c"),
        str(reference_sources/"Hiwonder/Portings/serial_servo_porting.c"),
        "-o", str(binary),
    ], check=True, capture_output=True, timeout=15)
    return binary


def execute(binary, *args):
    result = subprocess.run([str(binary), *map(str, args)], check=True,
                            capture_output=True, text=True, timeout=3)
    return [int(value) for value in result.stdout.split()]


@pytest.mark.parametrize("servo_id,command,corrupt,expected", [
    (1, 28, 0, 0), (2, 28, 0, 0), (1, 27, 0, 0), (1, 28, 1, -99),
])
def test_reference_parser_accepts_wrong_id_or_command_but_checks_checksum(
        reference_binary, servo_id, command, corrupt, expected):
    # PASS reproduces reference defects; it is NOT a protocol/hardware PASS.
    assert execute(reference_binary, "parser", servo_id, command, corrupt) == [
        expected, servo_id, command]


def test_reference_position_read_trusts_unmatched_shared_frame(reference_binary):
    assert execute(reference_binary, "unmatched_position") == [0, 777]


def test_reference_late_completion_can_satisfy_next_call_in_stubbed_schedule(reference_binary):
    # This proves the source admits this schedule, not that the actual firmware
    # or GPIO timing permits it. There is no real ISR, UART, GPIO, or RTOS here.
    assert execute(reference_binary, "late_completion") == [-1, -999, 0, 111]


def test_reference_report_identity_is_request_echo_and_example_is_not_runtime(reference_sources):
    text = (reference_sources/"Hiwonder/System/packet_handle.c").read_text()
    assert "report->servo_id = servo_id;" in text
    assert "report->sub_command = cmd;" in text
    assert "cmd->servo_id, cmd->cmd,  serial_servo_read_position" in text
    porting = (reference_sources/"Hiwonder/Portings/packet_porting.c").read_text()
    assert "//        packet_recv(&packet_controller);" in porting

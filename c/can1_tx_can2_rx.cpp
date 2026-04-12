#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "controlcan.h"

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int signum)
{
    (void)signum;
    g_stop = 1;
}

typedef struct ProgramOptions {
    int count;
    int interval_ms;
    unsigned int start_id;
} ProgramOptions;

static void print_usage(const char *program_name)
{
    printf("Usage: %s [--count N] [--interval-ms N] [--start-id ID]\n", program_name);
    printf("\n");
    printf("Options:\n");
    printf("  --count N        Send exactly N frames then exit.\n");
    printf("                   If omitted, runs continuously until Ctrl+C.\n");
    printf("  --interval-ms N  Transmit interval in milliseconds (default: 100).\n");
    printf("  --start-id ID    Start CAN ID in decimal or hex (default: 0x100).\n");
    printf("                   Valid range for standard frame: 0x000 - 0x7FF.\n");
    printf("  -h, --help       Show this help text.\n");
}

static int parse_non_negative_int(const char *text, int *value)
{
    char *end = NULL;
    long parsed = strtol(text, &end, 10);

    if (end == text || *end != '\0' || parsed < 0 || parsed > 2147483647L) {
        return 0;
    }

    *value = (int)parsed;
    return 1;
}

static int parse_u32_value(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 0);

    if (end == text || *end != '\0' || parsed > 0xFFFFFFFFUL) {
        return 0;
    }

    *value = (unsigned int)parsed;
    return 1;
}

static int parse_args(int argc, char **argv, ProgramOptions *options)
{
    int i = 0;

    options->count = -1;
    options->interval_ms = 100;
    options->start_id = 0x100U;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 1;
        }

        if (strcmp(argv[i], "--count") == 0) {
            if (i + 1 >= argc || !parse_non_negative_int(argv[i + 1], &options->count)) {
                fprintf(stderr, "Invalid value for --count\n");
                return -1;
            }
            ++i;
            continue;
        }

        if (strcmp(argv[i], "--interval-ms") == 0) {
            if (i + 1 >= argc || !parse_non_negative_int(argv[i + 1], &options->interval_ms) || options->interval_ms <= 0) {
                fprintf(stderr, "Invalid value for --interval-ms\n");
                return -1;
            }
            ++i;
            continue;
        }

        if (strcmp(argv[i], "--start-id") == 0) {
            if (i + 1 >= argc || !parse_u32_value(argv[i + 1], &options->start_id)) {
                fprintf(stderr, "Invalid value for --start-id\n");
                return -1;
            }
            ++i;
            continue;
        }

        fprintf(stderr, "Unknown argument: %s\n", argv[i]);
        return -1;
    }

    if (options->start_id > 0x7FFU) {
        fprintf(stderr, "--start-id must be in range 0x000..0x7FF for standard frame\n");
        return -1;
    }

    return 0;
}

static void print_char_array(const CHAR *text, int length)
{
    int i = 0;
    for (i = 0; i < length; ++i) {
        if (text[i] == '\0') {
            break;
        }
        putchar((int)text[i]);
    }
}

static void print_board_info(const VCI_BOARD_INFO *board)
{
    printf(">>Serial_Num:");
    print_char_array(board->str_Serial_Num, 20);
    printf("\n");

    printf(">>hw_Type:");
    print_char_array(board->str_hw_Type, 40);
    printf("\n");

    printf(">>Firmware Version:V%x.%x%x\n",
           (board->fw_Version & 0xF00) >> 8,
           (board->fw_Version & 0xF0) >> 4,
           board->fw_Version & 0xF);
}

static int init_and_start_can_channel(DWORD channel_index, VCI_INIT_CONFIG *config)
{
    if (VCI_InitCAN(VCI_USBCAN2, 0, channel_index, config) != STATUS_OK) {
        fprintf(stderr, ">>Init CAN%u error\n", (unsigned int)(channel_index + 1));
        return 0;
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, channel_index) != STATUS_OK) {
        fprintf(stderr, ">>Start CAN%u error\n", (unsigned int)(channel_index + 1));
        return 0;
    }

    return 1;
}

static void print_payload(const VCI_CAN_OBJ *frame)
{
    int i = 0;
    for (i = 0; i < frame->DataLen; ++i) {
        printf(" %02X", frame->Data[i]);
    }
}

static void print_tx_frame(unsigned long index, const VCI_CAN_OBJ *frame)
{
    printf("Index:%04lu  CAN1 TX ID:0x%03X Standard Data DLC:0x%02X data:0x",
           index,
           frame->ID & 0x7FFU,
           frame->DataLen);
    print_payload(frame);
    printf("\n");
}

static void print_rx_frame(unsigned long index, const VCI_CAN_OBJ *frame)
{
    const char *format_text = frame->ExternFlag ? "Extend" : "Standard";
    const char *type_text = frame->RemoteFlag ? "Remote" : "Data";

    printf("Index:%04lu  CAN2 RX ID:0x%08X %s %s DLC:0x%02X data:0x",
           index,
           frame->ID,
           format_text,
           type_text,
           frame->DataLen);
    print_payload(frame);
    printf(" TimeStamp:0x%08X\n", frame->TimeStamp);
}

static void drain_can2_receive_queue(unsigned long *index)
{
    int i = 0;
    int received = 0;
    VCI_CAN_OBJ frames[3000];

    while ((received = (int)VCI_Receive(VCI_USBCAN2, 0, 1, frames, 3000, 0)) > 0) {
        for (i = 0; i < received; ++i) {
            print_rx_frame(*index, &frames[i]);
            *index = *index + 1;
        }
    }
}

int main(int argc, char **argv)
{
    int parse_status = 0;
    int device_opened = 0;
    int can1_started = 0;
    int can2_started = 0;
    int exit_code = 0;
    int i = 0;
    int remaining = 0;
    int found_count = 0;
    unsigned long index = 0;
    ProgramOptions options;
    VCI_BOARD_INFO found_devices[50];
    VCI_BOARD_INFO board_info;
    VCI_INIT_CONFIG config;
    VCI_CAN_OBJ tx_frame;

    parse_status = parse_args(argc, argv, &options);
    if (parse_status == 1) {
        return 0;
    }
    if (parse_status != 0) {
        print_usage(argv[0]);
        return 2;
    }

    memset(&board_info, 0, sizeof(board_info));
    memset(&config, 0, sizeof(config));
    memset(&tx_frame, 0, sizeof(tx_frame));
    memset(found_devices, 0, sizeof(found_devices));

    printf(">>CAN1 TX -> CAN2 RX test\n");
    if (options.count < 0) {
        printf(">>Mode: continuous (stop with Ctrl+C), interval=%dms\n", options.interval_ms);
    } else {
        printf(">>Mode: fixed count=%d, interval=%dms\n", options.count, options.interval_ms);
    }
    printf(">>Frame: standard data, start-id=0x%03X\n", options.start_id);

    found_count = (int)VCI_FindUsbDevice2(found_devices);
    printf(">>USBCAN DEVICE NUM:%d PCS\n", found_count);
    for (i = 0; i < found_count; ++i) {
        printf(">>Device:%d\n", i);
        print_board_info(&found_devices[i]);
    }

    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK) {
        fprintf(stderr, ">>open device error\n");
        return 1;
    }
    device_opened = 1;

    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &board_info) == STATUS_OK) {
        printf(">>Get VCI_ReadBoardInfo success!\n");
        print_board_info(&board_info);
    } else {
        fprintf(stderr, ">>Get VCI_ReadBoardInfo error\n");
    }

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = 0x03;
    config.Timing1 = 0x1C;
    config.Mode = 0;

    if (!init_and_start_can_channel(0, &config)) {
        exit_code = 1;
        goto cleanup;
    }
    can1_started = 1;

    if (!init_and_start_can_channel(1, &config)) {
        exit_code = 1;
        goto cleanup;
    }
    can2_started = 1;

    tx_frame.ID = options.start_id;
    tx_frame.SendType = 0;
    tx_frame.RemoteFlag = 0;
    tx_frame.ExternFlag = 0;
    tx_frame.DataLen = 8;
    for (i = 0; i < tx_frame.DataLen; ++i) {
        tx_frame.Data[i] = (BYTE)i;
    }

    {
        struct sigaction action;
        memset(&action, 0, sizeof(action));
        action.sa_handler = on_signal;
        sigaction(SIGINT, &action, NULL);
        sigaction(SIGTERM, &action, NULL);
    }

    remaining = options.count;

    while (!g_stop && remaining != 0) {
        if (VCI_Transmit(VCI_USBCAN2, 0, 0, &tx_frame, 1) != 1) {
            fprintf(stderr, ">>CAN1 transmit error at ID:0x%03X\n", tx_frame.ID & 0x7FFU);
            exit_code = 1;
            break;
        }

        print_tx_frame(index, &tx_frame);
        index = index + 1;

        drain_can2_receive_queue(&index);

        if (remaining > 0) {
            --remaining;
        }

        tx_frame.ID = (tx_frame.ID + 1U) & 0x7FFU;

        if (!g_stop && remaining != 0) {
            usleep((useconds_t)options.interval_ms * 1000U);
        }
    }

    if (g_stop) {
        printf(">>Stop signal received, shutting down...\n");
    }

    usleep(100000);
    drain_can2_receive_queue(&index);

cleanup:
    if (can1_started) {
        usleep(100000);
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);
    }
    if (can2_started) {
        usleep(100000);
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);
    }
    if (device_opened) {
        usleep(100000);
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    return exit_code;
}

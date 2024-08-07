#include "cli_ecu.h"
#include "main.h"



#define N_COMMANDS 7
#define HTIM_CLI htim6
#define CLI_UART huart2


cli_command_func_t help;
cli_command_func_t _sd_feedbacks;
cli_command_func_t _pedals;
cli_command_func_t _fsm_state;
cli_command_func_t _inv;
cli_command_func_t _cli_help;
cli_command_func_t _cli_sigterm;

char *command_names[N_COMMANDS] = {"help", "sd_feedbacks", "pedals" "fsm_state", "inv", "?", "\003"};

cli_command_func_t *commands[N_COMMANDS] = {&help, &_sd_feedbacks, &_pedals, &_fsm_state, &_inv, &_cli_help, &_cli_sigterm};

cli_t cli_ecu;
bool dmesg_ena = true;

void cli_ecu_init(){
    cli_ecu.uart           = &CLI_UART;
    cli_ecu.cmds.functions = commands;
    cli_ecu.cmds.names     = command_names;
    cli_ecu.cmds.count     = N_COMMANDS;

    // char logo[] = "\033[H"
        // "         E - A g l e   T r e n t o   R a c i n g   T e a m        \r\n"
        // "██████████████████████████████     ████  ███  ████████████████████\r\n"
        // "                           ████   ███                             \r\n"
        // "  ████████████  ████████  ██████ ███   ███  ███         ████████  \r\n"
        // "      ███      ███       ███  █████   ███  ███        ███         \r\n"
        // "     ███      ████████  ███    ███   ███  █████████  █████████    \r\n"
        // "                                                                  \r\n";
    char init[95];

    snprintf(
        init,
        95,
        "\r\n\n********* Fenice BMS *********\r\n"
        " build: %s @ %s\r\n\n type ? for commands\r\n\n",
        __DATE__,
        __TIME__);

    strcat(init, cli_ps);

    cli_init(&cli_ecu);
    cli_print(&cli_ecu, init, strlen(init));
}

void cli_ecu_debug(char *text) {
    if (dmesg_ena) {
        char out[256] = {'\0'};

        snprintf(out, 256, "[%.2f] %s\r\n> ", (float)HAL_GetTick() / 1000, text);

        cli_print(&cli_ecu, out, strlen(out));
    }
}

void help(uint16_t argc, char **argv, char *out) {
    snprintf(out, CLI_TX_BUF_LEN, "command list:\r\n");
    for (uint8_t i = 0; i < N_COMMANDS - 3; i++) {
        snprintf(out + strlen(out), CLI_TX_BUF_LEN - strlen(out), "- %s\r\n", cli_ecu.cmds.names[i]);
    }
}

void _sd_feedbacks(uint16_t argc, char **argv, char *out){
    snprintf(out,
    CLI_TX_BUF_LEN,
    "SD_FB0 (Cockpit Mushroom): %d\r\n"
    "SD_FB1: %d\r\n"
    "SD_FB2 (BOTS): %d\r\n"
    "SD_FB3 (Inertial Switch): %d\r\n"
    "SD_FB4: %d\r\n"
    "SD_IN: %d\r\n"
    "SD_OUT: %d\r\n"
    "SD CTRL PIN: %d\r\n",
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB0())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB1())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB2())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB3())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB4())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_IN())),
    ADC_is_closed(ADC_to_voltage(ADC_get_SD_OUT())),
    HAL_GPIO_ReadPin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin)
    );
}

void _pedals(uint16_t argc, char **argv, char *out){
    snprintf(out,
    CLI_TX_BUF_LEN,
    "APPS1: %" PRIu32 "\r\n"
    "APPS2: %" PRIu32 "\r\n"
    "BPPS1: %" PRIu32 "\r\n"
    "BPPS2: %" PRIu32 "\r\n"
    "BRKF: %" PRIu32 "\r\n"
    "BRKR: %" PRIu32 "\r\n"
    "ACC: %f\r\n"
    "BRK: %f\r\n",
    ADC_get_APPS1(),
    ADC_get_APPS2(),
    ADC_get_BPPS1(),
    ADC_get_BPPS2(),
    ADC_get_BRK_F(),
    ADC_get_BRK_R(),
    PED_get_accelerator_percent(),
    PED_get_brake_bar()
    );
}

void _fsm_state(uint16_t argc, char **argv, char *out){
    extern state_t current_state;
    snprintf(out,
    CLI_TX_BUF_LEN,
    "FSM State: %-6s\r\n",
    state_names[current_state]
    );
}

void _inv(uint16_t argc, char **argv, char *out){
    snprintf(out,
    CLI_TX_BUF_LEN,
    "Inverter");
}

void _cli_help(uint16_t argc, char **argv, char *out) {
    snprintf(out, CLI_TX_BUF_LEN, "command list:\r\n");
    for (uint8_t i = 0; i < N_COMMANDS - 3; i++) {
        snprintf(out + strlen(out), CLI_TX_BUF_LEN - strlen(out), "- %s\r\n", cli_ecu.cmds.names[i]);
    }
}

char watch_buf[BUF_SIZE]      = {'\0'};
uint8_t cli_watch_flush_tx    = 0;
uint8_t cli_watch_execute_cmd = 0;

void _cli_sigterm(uint16_t argc, char **argv, char *out) {
    if (HTIM_CLI.State == HAL_TIM_STATE_BUSY) {
        HAL_TIM_Base_Stop_IT(&HTIM_CLI);
        *watch_buf = '\0';
        snprintf(out, CLI_TX_BUF_LEN, "\r\n");
    } else {
        snprintf(out, CLI_TX_BUF_LEN, "^C\r\n");
    }
}

void _cli_watch(uint16_t argc, char **argv, char *out) {
    *watch_buf = '\0';
    if (!strcmp(argv[1], "stop")) {
        HAL_TIM_Base_Stop_IT(&HTIM_CLI);
    } else {
        uint16_t interval = atoi(argv[1]);
        if (interval == 0)
            interval = 500;
        for (uint8_t i = 2; i < argc; ++i) {
            snprintf(watch_buf + strlen(watch_buf), BUF_SIZE - strlen(out), "%s ", argv[i]);
        }
        cli_watch_execute_cmd            = 1;
        watch_buf[strlen(watch_buf) - 1] = '\0';
        __HAL_TIM_SetAutoreload(&HTIM_CLI, TIM_MS_TO_TICKS(&HTIM_CLI, interval));
        __HAL_TIM_CLEAR_IT(&HTIM_CLI, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&HTIM_CLI);
    }

    snprintf(out, CLI_TX_BUF_LEN, "\033[2J\033H");
}

void _cli_timer_handler(TIM_HandleTypeDef *htim) {
    if (cli_watch_flush_tx == 0)
        cli_watch_execute_cmd = 1;
    cli_watch_flush_tx = 1;
}

#define CLI_ECU_PRINT_BUF_LEN (CLI_TX_BUF_LEN / 4)

char tx_buf[CLI_TX_BUF_LEN]           = {'\0'};
char print_buf[CLI_ECU_PRINT_BUF_LEN] = {'\0'};
char *save_ptr                        = NULL;


void cli_watch_flush_handler() {
    char *argv[BUF_SIZE] = {NULL};
    uint16_t argc;
    char *to_print;

    if (cli_watch_flush_tx == 0)
        return;

    if (cli_watch_execute_cmd == 1) {
        snprintf(
            tx_buf,
            CLI_TX_BUF_LEN,
            "\033[HExecuting %s every %.0fms\033[K\r\n[%.2f]\033[K\r\n",
            watch_buf,
            TIM_TICKS_TO_MS(&HTIM_CLI, __HAL_TIM_GetAutoreload(&HTIM_CLI)),
            HAL_GetTick() / 1000.0);

        argc = _cli_get_args(watch_buf, argv);

        // Check which command corresponds with the buffer
        for (uint16_t i = 0; i < N_COMMANDS; i++) {
            //size_t len = strlen(cli->cmds.names[i]);

            if (strcmp(argv[0], command_names[i]) == 0) {
                commands[i](argc, argv, tx_buf + strlen(tx_buf));
                break;
            }

            if (i == N_COMMANDS - 1) {
                snprintf(tx_buf, CLI_TX_BUF_LEN, "Command not found\r\n");
                *watch_buf = '\0';
                HAL_TIM_Base_Stop_IT(&HTIM_CLI);
                HAL_UART_Transmit(&CLI_UART, (uint8_t *)tx_buf, strlen(tx_buf), 100);
                return;
            }
        }

        snprintf(tx_buf + strlen(tx_buf), CLI_TX_BUF_LEN - strlen(tx_buf), "\r\nPress CTRL+C to stop\r\n\033[J");

        // restore watch_buf after splitting it in _cli_get_args
        for (uint16_t i = 0; i < argc - 1; ++i) {
            watch_buf[strlen(watch_buf)] = ' ';
        }
        to_print = strtok_r(tx_buf, "\r\n", &save_ptr);

        snprintf(print_buf, sizeof(print_buf), "%s\r\n", to_print);

        HAL_UART_Transmit_IT(&CLI_UART, (uint8_t *)print_buf, strlen(print_buf));
        cli_watch_execute_cmd = 0;
    } else {
        if (CLI_UART.gState != HAL_UART_STATE_BUSY_TX) {
            to_print = strtok_r(NULL, "\r\n", &save_ptr);

            if (to_print != NULL) {
                snprintf(print_buf, sizeof(print_buf), "%s\033[K\r\n", to_print);
                HAL_UART_Transmit_IT(&CLI_UART, (uint8_t *)print_buf, strlen(print_buf));
            } else {
                cli_watch_flush_tx = 0;
            }
        }
    }
}
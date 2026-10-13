#ifndef APP_CONSOLE_CMD_H
#define APP_CONSOLE_CMD_H

#include "main.h"

void app_console_init(void);
void app_console_poll(void);
void app_console_execute_line(const char *line);
uint8_t app_console_handle_dac_command(const char *line);

#endif /* APP_CONSOLE_CMD_H */

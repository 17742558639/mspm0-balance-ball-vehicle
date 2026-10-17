/*
 * main.c - 入口. 仅调 system_init() + system_run(), 详见 system.h.
 */
#include "system.h"

int main(void)
{
    system_init();
    system_run();
    return 0;
}

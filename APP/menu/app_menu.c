/*
 * app_menu.c - H题任务菜单与OLED显示。
 *
 * 当前仅任务2（A Stop）具有完整控制器。任务3至任务6保留菜单入口，
 * 在对应控制器实现前统一进入安全停止状态，避免误启动任务2状态机。
 */
#include "app_menu.h"

#include <stdbool.h>

#include "Control.h"
#include "ball_target_task.h"
#include "menu_core.h"
#include "menu_text.h"
#include "mpu6050_vehicle.h"
#include "oled.h"
#include "track.h"


enum
{
    PAGE_MAIN,
    PAGE_TRACK_VIEW,
    PAGE_A_PASS,
    PAGE_COUNT
};

static const MenuPage pages[PAGE_COUNT];
static MenuContext g_menu;
static bool g_a_stop_view_active = false;
static bool g_ab_view_active = false;

static void action_a_stop(void)
{
    BallTargetTask_stop();
    g_a_stop_view_active = true;
    control_set_run(true);
}

static void action_ab(void)
{
    BallTargetTask_stop();
    g_ab_view_active = true;
    if (control_start_ab())
    {
        BallTargetTask_startCenterHold();
    }
}

static void action_full_lap(void)
{
    BallTargetTask_stop();
    control_set_run(true);
    if (start_run)
    {
        BallTargetTask_startCenterHold();
    }
}

static void action_ball_target(void)
{
    BallTargetTask_stop();
    control_set_run(false);
    BallTargetTask_startCenterHold();
}

static void action_ball_only(void)
{
    control_set_run(false);
    BallTargetTask_start();
}

static const MenuItem a_pass_items[] =
{
    {"AB",          MENU_ITEM_ACTION, {.action = action_ab}},
    {"Full Lap",    MENU_ITEM_ACTION, {.action = action_full_lap}},
    {"Ball Target", MENU_ITEM_ACTION, {.action = action_ball_target}},
};

static const MenuItem main_items[] =
{
    {"Track View", MENU_ITEM_SUBMENU, {.page = &pages[PAGE_TRACK_VIEW]}},
    {"A Stop",     MENU_ITEM_ACTION,  {.action = action_a_stop}},
    {"A Pass",     MENU_ITEM_SUBMENU, {.page = &pages[PAGE_A_PASS]}},
    {"Ball Only",  MENU_ITEM_ACTION,  {.action = action_ball_only}},
};

static const MenuPage pages[PAGE_COUNT] =
{
    [PAGE_MAIN]       = {"Track Mode", main_items,   MENU_ARRAY_COUNT(main_items)},
    [PAGE_TRACK_VIEW] = {"Track View", NULL,         0U},
    [PAGE_A_PASS]     = {"A Pass",     a_pass_items, MENU_ARRAY_COUNT(a_pass_items)},
};

static void a_stop_render(void)
{
    uint32_t elapsed = control_parking_elapsed_ms();
    uint32_t seconds = elapsed / 1000U;
    uint32_t millis = elapsed % 1000U;

    track_update();

    OLED_Clear();
    OLED_PrintfRC(0, 5, "A Stop");
    OLED_PrintfRC(1, 0, "Time:%lu.%03lus",
        (unsigned long)seconds, (unsigned long)millis);
    OLED_PrintfRC(2, 0, "IR:%u%u%u%u%u%u%u%u",
        (unsigned int)track_sensors[0], (unsigned int)track_sensors[1],
        (unsigned int)track_sensors[2], (unsigned int)track_sensors[3],
        (unsigned int)track_sensors[4], (unsigned int)track_sensors[5],
        (unsigned int)track_sensors[6], (unsigned int)track_sensors[7]);

    if (control_parking_result_ready())
    {
        OLED_PrintfRC(3, 0,
            control_parking_result_success() ? "PARK OK" : "PARK FAIL");
    }
    else if (start_run)
    {
        OLED_PrintfRC(3, 0, "RUNNING");
    }
    else
    {
        OLED_PrintfRC(3, 0, "START BLOCKED");
    }

    OLED_Update();
    menu_clear_dirty(&g_menu);
}

static void ab_render(void)
{
    uint32_t elapsed = control_ab_elapsed_ms();
    uint32_t seconds = elapsed / 1000U;
    uint32_t millis = elapsed % 1000U;
    uint32_t pulses = control_ab_distance_pulses();

    OLED_Clear();
    OLED_PrintfRC(0, 7, "AB");
    OLED_PrintfRC(1, 0, "Time:%lu.%03lus",
        (unsigned long)seconds, (unsigned long)millis);
    OLED_PrintfRC(2, 0, "Pulse:%lu", (unsigned long)pulses);

    if (start_run && control_active_task() == CONTROL_TASK_AB)
    {
        OLED_PrintfRC(3, 0, "RUNNING");
    }
    else
    {
        OLED_PrintfRC(3, 0, "START BLOCKED");
    }

    OLED_Update();
    menu_clear_dirty(&g_menu);
}

static void track_view_render(void)
{
    track_update();

    OLED_Clear();
    OLED_PrintfRC(0, 3, "Track View");
    OLED_PrintfRC(1, 0, "1 2 3 4 5 6 7 8");
    OLED_PrintfRC(2, 0, "%d %d %d %d %d %d %d %d",
        track_sensors[0], track_sensors[1], track_sensors[2], track_sensors[3],
        track_sensors[4], track_sensors[5], track_sensors[6], track_sensors[7]);
    OLED_Update();
    menu_clear_dirty(&g_menu);
}

void app_menu_init(void)
{
    g_a_stop_view_active = false;
    g_ab_view_active = false;
    (void)mpu6050_vehicle_monitor_init();
    (void)menu_init(&g_menu, &pages[PAGE_MAIN], 3U);
}

void app_menu_invalidate(void)
{
    (void)mpu6050_vehicle_monitor_update();
    menu_invalidate(&g_menu);
}

void app_menu_refresh(void)
{
    char line[17];
    uint8_t rows;
    uint8_t row;

    if (!menu_is_dirty(&g_menu))
    {
        return;
    }

    if (g_a_stop_view_active)
    {
        a_stop_render();
        return;
    }

    if (g_ab_view_active)
    {
        ab_render();
        return;
    }

    if (menu_current_page(&g_menu) == &pages[PAGE_TRACK_VIEW])
    {
        track_view_render();
        return;
    }

    OLED_Clear();
    rows = menu_visible_rows(&g_menu);
    for (row = 0U; row <= rows; row++)
    {
        if (menu_text_get_line(&g_menu, row, line, sizeof(line), 16U))
        {
            OLED_ShowString(0, (uint8_t)(row * 16U), line, OLED_FONT_16);
        }
    }
    OLED_Update();
    menu_clear_dirty(&g_menu);
}

void app_menu_on_key(uint8_t id, Key_Event event)
{
    MenuEvent menu_event;

    if (g_a_stop_view_active)
    {
        if (id == 3U && event == KEY_EVENT_SHORT_PRESS)
        {
            control_set_run(false);
            g_a_stop_view_active = false;
            menu_invalidate(&g_menu);
        }
        return;
    }


    if (g_ab_view_active)
    {
        if (id == 3U && event == KEY_EVENT_SHORT_PRESS)
        {
            control_set_run(false);
            BallTargetTask_stop();
            g_ab_view_active = false;
            menu_invalidate(&g_menu);
        }
        return;
    }

    switch (id)
    {
    case 0:
        if (event != KEY_EVENT_SHORT_PRESS && event != KEY_EVENT_LONG_PRESS &&
            event != KEY_EVENT_LONG_REPEAT)
        {
            return;
        }
        menu_event = MENU_EVENT_DOWN;
        break;

    case 1:
        if (event != KEY_EVENT_SHORT_PRESS && event != KEY_EVENT_LONG_PRESS &&
            event != KEY_EVENT_LONG_REPEAT)
        {
            return;
        }
        menu_event = MENU_EVENT_UP;
        break;

    case 2:
        if (event != KEY_EVENT_SHORT_PRESS)
        {
            return;
        }
        menu_event = MENU_EVENT_ENTER;
        break;

    case 3:
        if (event != KEY_EVENT_SHORT_PRESS)
        {
            return;
        }
        control_set_run(false);
        BallTargetTask_stop();
        menu_event = MENU_EVENT_BACK;
        break;

    default:
        return;
    }

    (void)menu_handle_event(&g_menu, menu_event);
}

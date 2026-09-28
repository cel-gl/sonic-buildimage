/*
 * $Id: custom_led.c$
 *
 * $Copyright: (c) 2025 Broadcom.
 * Broadcom Proprietary and Confidential. All rights reserved.$
 *
 * File:        custom_led.c
 * Purpose:     Customer CMICx LED bit pattern composer.
 * Requires:
 */

/******************************************************************************
 *
 * The CMICx LED interface has two RAM Banks as shown below, Bank0
 * (Accumulation RAM) for accumulation of status from ports and Bank1
 * (Pattern RAM) for writing LED pattern. Each row is representing
 * one port.
 *
 *           Accumulation RAM (Bank 0)        Pattern RAM (Bank1)
 *          15                       0     15                       0
 *          ----------------------------   ------------------------------
 * Row 0   |  led_uc_port 0 status    |   | led_uc_port 0 LED Pattern   |
 *          ----------------------------   ------------------------------
 * Row 1   |  led_uc_port 1 status    |   | led_uc_port 1 LED Pattern   |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 * Row 127 |  led_uc_port 128 status  |   | led_uc_port 128 LED Pattern |
 *          ----------------------------   ------------------------------
 * Row 128 |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 * Row x   |  led_uc_port (x+1) status|   | led_uc_port(x+1) LED Pattern|
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 *         |                            |   |                            |
 *          ----------------------------   ------------------------------
 * Row 1022|  led_uc_port 1022 status |   | led_uc_port 1022 LED Pattern|
 *          ----------------------------   ------------------------------
 * Row 1023|  led_uc_port 1023 status |   | led_uc_port 1023 LED Pattern|
 *          ----------------------------   ------------------------------
 *
 * Format of Accumulation RAM:
 *
 * Bits   15:9       8        7        6        5      4:3     2    1    0
 *    ------------------------------------------------------------------------
 *    | Reserved | Link  | Link-up |  Flow  | Duplex | Speed | Col | Tx | Rx |
 *    |          | Enable| Status  | Control|        |       |     |    |    |
 *    ------------------------------------------------------------------------
 *
 * Where Speed 00 - 10 Mbps
 *             01 - 100 Mbps
 *             10 - 1 Gbps
 *             11 - Above 1 Gbps
 *
 * The customer handler in this file should read the port status from
 * the HW Accumulation RAM or "led_control_data" array, then form the required
 * LED bit pattern in the Pattern RAM at the corresponding location.
 *
 * "led_control data" is an array (1024 bytes for SDK6, 4096 bytes for HSDK).
 * The application can access the "led_control data" area through the
 * BCM LED API to exchange information with the LED firmware.
 *
 * Typically, led_uc_port = physical port number - constant.
 * The constant is 1 for ESW chips, 0 for DNX/DNXF chips and 2 for Firelight.
 * For those ports that do not meet the above rule, they will be listed in
 * "include/shared/cmicfw/cmicx_led_public.h".
 *
 * There are five LED interfaces in CMICx-based devices, and although
 * a single interface can be used to output LED patterns for all
 * ports, it is possible to use more than one interface, e.g. the LEDs
 * for some ports are connected to LED interface-0, while the rest of
 * the ports are connected to LED interface-1. Accordingly, the custom
 * handler MUST fill in start-port, end-port and pattern-width in the
 * soc_led_custom_handler_ctrl_t structure passed to the custom
 * handler.
 *
 * The example custom handler provided in this file has reference code
 * for forming two different LED patterns. Please refer to these
 * patterns before writing your own custom handler code.
 *
 * The led_customer_t structure definition is available in
 * include/shared/cmicfw/cmicx_led_public.h.
 *
 ******************************************************************************/

#include <shared/cmicfw/cmicx_led_public.h>

/* ============================================================
 * 1. MACROS & CONSTANTS
 * ============================================================ */

/* 2-bit Color Definitions */
#define LED_BIT_GREEN       0b00
#define LED_BIT_AMBER       0b01
#define LED_BIT_OFF         0b11

/* Blink Flags (Internal Use) */
#define LED_BLINK_FLAG      0xF0
#define LED_BLINK_GREEN     (LED_BLINK_FLAG | LED_BIT_GREEN)

#define IS_BLINK_MODE(c)    ((c) & LED_BLINK_FLAG)
#define GET_BASE_COLOR(c)   ((c) & 0x0F)
#define LED_LINK_UP(accu)   ((accu) & 0x0100) /* LED_HW_LINK */
#define LED_ACTIVITY(accu)  ((accu) & 0x0003) /* RX | TX */

/* Constants */
#define NUM_LOGICAL_PORTS   32
#define NUM_MGMT_PORTS      2

/* Compact Port Definition */
typedef struct { uint8 base; uint8 count; } port_map_t;

static const port_map_t fp_ports[NUM_LOGICAL_PORTS] = {
    /* Interface 0 (Ports 1-16) */
    {1, 8}, {9, 8}, {25, 8}, {33, 8},
    {17, 4}, {21, 4}, {41, 4}, {45, 4},
    {49, 4}, {53, 4}, {57, 4}, {61, 4},
    {65, 4}, {69, 4}, {73, 4}, {77, 4},
    /* Interface 1 (Ports 17-32) */
    {81, 4}, {85, 4}, {89, 4}, {93, 4},
    {97, 4}, {101, 4}, {105, 4}, {109, 4},
    {113, 4}, {117, 4}, {137, 4}, {141, 4},
    {121, 8}, {129, 8}, {145, 8}, {153, 8}
};

static const uint8 mgmt_phys[NUM_MGMT_PORTS] = { 160, 162 };

/* ============================================================
 * 2. MAIN HANDLER
 * ============================================================ */

/*!
 * \brief Function for LED bit pattern generator.
 *
 * Customer can compose the LED bit pattern to control serial LED
 * according to link/traffic information.
 *
 * \param [in,out] ctrl Data structure indicating the locations of the
 *                      port status and serial LED bit pattern RAM.
 * \param [in] cnt 30Hz counter.
 *
 */
void customer_led_handler(soc_led_custom_handler_ctrl_t *ctrl, uint32 cnt)
{
    int i, lane;
    uint16 accu_val, phys_port, pat_val;
    uint8 final_leds[2]; /* [0]=LED1, [1]=LED2 */
    
    /* Approx 1Hz blink rate from 30Hz counter */
    uint8 blink_off_tick = (cnt & 0x1); 

    /* --------------------------------------------------------
     * STEP 1: Process Front Panel Ports (1-32)
     * -------------------------------------------------------- */
    for (i = 0; i < NUM_LOGICAL_PORTS; i++) {
        uint8 base_phys = fp_ports[i].base;
        uint8 max_lanes = fp_ports[i].count;
        
        int split_lane = max_lanes / 2;
        if (split_lane == 0) split_lane = 1;

        uint8 led1 = LED_BIT_OFF;
        uint8 led2 = LED_BIT_OFF;

        /* --- A. Spatial Aggregation --- */
        for (lane = 0; lane < max_lanes; lane++) {
            phys_port = base_phys + lane - 1;
            accu_val = LED_HW_RAM_READ16(ctrl->accu_ram_base, phys_port);

            if (LED_LINK_UP(accu_val)) {
                uint8 color = LED_BIT_GREEN;

                if (LED_ACTIVITY(accu_val)) {
                    color |= LED_BLINK_FLAG;
                }

                if (lane < split_lane) {
                    if (led1 == LED_BIT_OFF) {
                        led1 = color;
                    } else {
                        uint8 f = (IS_BLINK_MODE(led1) || IS_BLINK_MODE(color)) ? LED_BLINK_FLAG : 0;
                        led1 = LED_BIT_GREEN | f;
                    }
                } else {
                    if (led2 == LED_BIT_OFF) {
                        led2 = color;
                    } else {
                        uint8 f = (IS_BLINK_MODE(led2) || IS_BLINK_MODE(color)) ? LED_BLINK_FLAG : 0;
                        led2 = LED_BIT_GREEN | f;
                    }
                }
            }
        }

        final_leds[0] = led1;
        final_leds[1] = led2;

        /* --- B. Apply Blink Timer & Write --- */
        for (int led_idx = 0; led_idx < 2; led_idx++) {
            if (IS_BLINK_MODE(final_leds[led_idx])) {
                if (blink_off_tick) final_leds[led_idx] = LED_BIT_OFF;
                else final_leds[led_idx] = GET_BASE_COLOR(final_leds[led_idx]);
            }
        }

        /* Pack and Write (Port i) */
        uint8 packed = (final_leds[1] << 2) | (final_leds[0] & 0x3);
        pat_val = LED_HW_RAM_READ16(ctrl->pat_ram_base, i);
        pat_val = (pat_val & ~0x0F) | packed;
        LED_HW_RAM_WRITE16(ctrl->pat_ram_base, i, pat_val);
    }

    /* --------------------------------------------------------
     * STEP 2: Management Ports
     * -------------------------------------------------------- */
    final_leds[0] = LED_BIT_OFF;
    final_leds[1] = LED_BIT_OFF;

    for (i = 0; i < NUM_MGMT_PORTS; i++) {
        phys_port = mgmt_phys[i];
        
        /* Direct physical port lookup for management ports */
        accu_val = LED_HW_RAM_READ16(ctrl->accu_ram_base, phys_port);

        if (LED_LINK_UP(accu_val)) {
            uint8 color = LED_BIT_GREEN;

            if (LED_ACTIVITY(accu_val)) {
                if (!blink_off_tick) final_leds[i] = color;
            } else {
                final_leds[i] = color;
            }
        }
    }

    /* Write Mgmt (Index 32) */
    uint16 mgmt_packed = (final_leds[1] << 2) | (final_leds[0] & 0x3);
    pat_val = LED_HW_RAM_READ16(ctrl->pat_ram_base, 32);
    pat_val = (pat_val & ~0x0F) | mgmt_packed;
    
    /* Write modified pat_val to preserve bitmask */
    LED_HW_RAM_WRITE16(ctrl->pat_ram_base, 32, pat_val);

    /* --------------------------------------------------------
     * STEP 3: Config Init
     * -------------------------------------------------------- */
    if (!ctrl->intf_ctrl[0].valid) {
        ctrl->intf_ctrl[0].valid = 1;
        ctrl->intf_ctrl[0].start_row = 0;
        ctrl->intf_ctrl[0].end_row = 15;
        ctrl->intf_ctrl[0].pat_width = 4;

        ctrl->intf_ctrl[1].valid = 1;
        ctrl->intf_ctrl[1].start_row = 16;
        ctrl->intf_ctrl[1].end_row = 32;
        ctrl->intf_ctrl[1].pat_width = 4;
    }
}

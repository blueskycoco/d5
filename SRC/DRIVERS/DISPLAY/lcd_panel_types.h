//*********************************************************************
//* Software that is described herein is for illustrative purposes only  
//* which provides customers with programming information regarding the  
//* products. This software is supplied "AS IS" without any warranties.  
//* NXP Semiconductors assumes no responsibility or liability for the 
//* use of the software, conveys no license or title under any patent, 
//* copyright, or mask work right to the product. NXP Semiconductors 
//* reserves the right to make changes in the software without 
//* notification. NXP Semiconductors also make no representation or 
//* warranty that such application will be suitable for the specified 
//* use without further testing or modification. 
//*
//* Copyright NXP Semiconductors
//*********************************************************************
//
// lcd_panel_types.c
//
// List of supported panel configurations
//

#pragma once

#include <windows.h>
#include "lpc32xx_clcdc.h"

//------------------------------------------------------------------------------
typedef enum {
    TFT = 0,      // Panel type is standard TFT
    MONO_4BIT,    // Panel type is 4-bit mono
    MONO_8BIT,    // Panel type is 8-bit mono
    CSTN          // Panel type is color STN
} LCD_PANEL_T;

// Structure containing the parameters for the LCD panel
typedef struct {
	WCHAR           panel_name[128];
    UNS_8           h_back_porch;         /* Horizontal back porch in
                                             clocks (minimum of 1) */
    UNS_8           h_front_porch;        /* Horizontal front porch in
                                             clocks (minimum of 1) */
    UNS_8           h_sync_pulse_width;   /* HSYNC pulse width in
                                             clocks (minimum of 1) */
    UNS_16          pixels_per_line;      /* Pixels per line (horizontal
                                             resolution) */
    UNS_8           v_back_porch;         /* Vertical back porch in
                                             clocks */
    UNS_8           v_front_porch;        /* Vertical front porch in
                                             clocks */
    UNS_8           v_sync_pulse_width;   /* VSYNC pulse width in
                                             clocks (minimum 1 clock) */
    UNS_16          lines_per_panel;      /* Lines per panel (vertical
                                             resolution) */
    UNS_8           invert_output_enable; /* Invert output enable, 1 =
                                             invert*/
    UNS_8           invert_panel_clock;   /* Invert panel clock, 1 =
                                             invert*/
    UNS_8           invert_hsync;         /* Invert HSYNC, 1 = invert */
    UNS_8           invert_vsync;         /* Invert VSYNC, 1 = invert */
    UNS_8           ac_bias_frequency;    /* AC bias frequency in
                                             clocks (minimum 1) */
    UNS_8           bits_per_pixel;       /* Maximum bits per pixel the
                                             HARDWARE supports */
    UNS_32          optimal_clock;        /* Optimal clock rate (Hz) */
    LCD_PANEL_T     lcd_panel_type;       /* LCD panel type */
    UNS_8           dual_panel;           /* Dual panel, 1 = dual panel
                                             display */
    ULONG           m_BitMasks[3];
} LCD_PARAM_T;

//------------------------------------------------------------------------------
#define LCD_MAX_PANELS 1
extern LCD_PARAM_T LCDPanelList[LCD_MAX_PANELS];

/**
 ******************************************************************************
 * @file    circle_limitation.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Circle Limitation component of the Motor Control SDK.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 * @ingroup CircleLimitation
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIRCLELIMITATION_H
#define __CIRCLELIMITATION_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bsp_global.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup CircleLimitation
 * @{
 */

#if MAX_MODULATION_99_PER_CENT
#define START_INDEX 62
#define MAX_MODULE  32439 // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*99%
#define MMITABLE    {                                                     \
    32635, 32375, 32121, 31873, 31631, 31394, 31162, 30935, 30714, 30497, \
    30284, 30076, 29872, 29672, 29574, 29380, 29190, 29003, 28820, 28641, \
    28464, 28291, 28122, 27955, 27791, 27630, 27471, 27316, 27163, 27012, \
    26864, 26718, 26575, 26434, 26295, 26159, 26024, 25892, 25761, 25633, \
    25569, 25444, 25320, 25198, 25078, 24959, 24842, 24727, 24613, 24501, \
    24391, 24281, 24174, 24067, 23963, 23859, 23757, 23656, 23556, 23458, \
    23361, 23265, 23170, 23077, 22984, 22893                              \
}

#elif MAX_MODULATION_96_PER_CENT
#define START_INDEX 58
#define MAX_MODULE  31456 // root(Vd^2+Vq^2) <= MAX_MODULE = 31456*96%
#define MMITABLE    {                                                     \
    32619, 32472, 32184, 31904, 31631, 31365, 31106, 30853, 30728, 30484, \
    30246, 30013, 29785, 29563, 29345, 29238, 29028, 28822, 28620, 28423, \
    28229, 28134, 27946, 27762, 27582, 27405, 27231, 27061, 26977, 26811, \
    26649, 26489, 26332, 26178, 26027, 25952, 25804, 25659, 25517, 25376, \
    25238, 25103, 25035, 24903, 24772, 24644, 24518, 24393, 24270, 24210, \
    24090, 23972, 23855, 23741, 23627, 23516, 23461, 23352, 23244, 23138, \
    23033, 22930, 22828, 22777, 22677, 22579, 22481, 22385, 22290, 22196  \
}

#elif MAX_MODULATION_95_PER_CENT
#define START_INDEX 57
#define MAX_MODULE  31128 // root(Vd^2+Vq^2) <= MAX_MODULE = 31128*95%
#define MMITABLE    {                                                     \
    32613, 32310, 32016, 31872, 31589, 31314, 31046, 30784, 30529, 30404, \
    30158, 29919, 29684, 29456, 29343, 29122, 28906, 28695, 28488, 28285, \
    28186, 27990, 27798, 27610, 27425, 27245, 27155, 26980, 26808, 26639, \
    26473, 26392, 26230, 26072, 25917, 25764, 25614, 25540, 25394, 25250, \
    25109, 24970, 24901, 24766, 24633, 24501, 24372, 24245, 24182, 24058, \
    23936, 23816, 23697, 23580, 23522, 23408, 23295, 23184, 23075, 23021, \
    22913, 22808, 22703, 22600, 22499, 22449, 22349, 22251, 22154, 22059, \
    21964                                                                 \
}

#elif MAX_MODULATION_94_PER_CENT
#define START_INDEX 56
#define MAX_MODULE  30800 // root(Vd^2+Vq^2) <= MAX_MODULE = 30800*94%
#define MMITABLE    {                                                     \
    32607, 32293, 31988, 31691, 31546, 31261, 30984, 30714, 30451, 30322, \
    30069, 29822, 29581, 29346, 29231, 29004, 28782, 28565, 28353, 28249, \
    28044, 27843, 27647, 27455, 27360, 27174, 26991, 26812, 26724, 26550, \
    26380, 26213, 26049, 25968, 25808, 25652, 25498, 25347, 25272, 25125, \
    24981, 24839, 24699, 24630, 24494, 24360, 24228, 24098, 24034, 23908, \
    23783, 23660, 23600, 23480, 23361, 23245, 23131, 23074, 22962, 22851, \
    22742, 22635, 22582, 22477, 22373, 22271, 22170, 22120, 22021, 21924, \
    21827, 21732                                                          \
}

#endif

/**
 * @brief  CircleLimitation component parameters definition
 */
typedef struct
{
    uint16_t MaxModule;              /**<  Circle limitation maximum allowed
                                          module */
    uint16_t Circle_limit_table[87]; /**<  Circle limitation table */
    uint8_t Start_index;             /**<  Circle limitation table indexing
                                          start */
} CircleLimitation_Handle_t;

/* Exported functions ------------------------------------------------------- */

Volt_Components Circle_Limitation(CircleLimitation_Handle_t *pHandle, Volt_Components Vqd);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __Circle Limitation_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

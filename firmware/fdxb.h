//------------------------------------------------------------------------------
//       Filename: fdxb.h
//------------------------------------------------------------------------------
//       Bogdan Ionescu (c) 2025
//------------------------------------------------------------------------------
//       Purpose : Defines the FDX-B module interface
//------------------------------------------------------------------------------
//       Notes : None
//       FDX-B Decoding: https://www.priority1design.com.au/fdx-b_animal_identification_protocol.html
//------------------------------------------------------------------------------
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Module includes
//------------------------------------------------------------------------------
#include <funconfig.h>
#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Module exported defines
//------------------------------------------------------------------------------
#define CONFIG_RFID_FREQ_125kHz 386
#define CONFIG_RFID_FREQ_134kHz 360

#ifndef CONFIG_RFID_FREQ_VAL
#define CONFIG_RFID_FREQ_VAL CONFIG_RFID_FREQ_134kHz
#endif

//------------------------------------------------------------------------------
// Module exported type definitions
//------------------------------------------------------------------------------
typedef struct
{
    uint64_t id;
    uint16_t country;
    union
    {
        struct
        {
            uint16_t dataBlock : 1;
            uint16_t undefined : 14;
            uint16_t animal : 1;
        };
        uint16_t flags; // 2 flag bits, 14 reserved bits
    };
    uint16_t checksum; // crc16

#if CONFIG_READ_EXTRA_DATA
    uint8_t data[3];
#endif

    bool valid;
} FDX_B_t;

//------------------------------------------------------------------------------
// Module exported functions
//------------------------------------------------------------------------------

/**
 * @brief  Initialize the FDX-B module
 * @param  None
 * @return None
 */
void FDX_B_Init(void);

/**
 * @brief  Check if a tag is available to read
 * @param  None
 * @return true if a tag is available
 */
bool FDX_B_Available(void);

/**
 * @brief  Read the tag
 * @param[out] tag - The tag to read into
 * @return None
 */
void FDX_B_Read(FDX_B_t *tag);

/**
 * @brief  Convert the ID to a string
 * @param[in] tag - The tag to convert
 * @param[out] str - The string to write to
 * @return None
 */
void FDX_B_ID_ToString(const FDX_B_t *tag, char str[16]);

//------------------------------------------------------------------------------
// Module exported variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

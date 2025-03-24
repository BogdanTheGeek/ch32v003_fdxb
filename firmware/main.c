//------------------------------------------------------------------------------
//       Filename: main.c
//------------------------------------------------------------------------------
//       Bogdan Ionescu (c) 2025
//------------------------------------------------------------------------------
//       Purpose : Application entry point
//------------------------------------------------------------------------------
//       Notes :
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module includes
//------------------------------------------------------------------------------
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ch32fun.h>

#include "fdxb.h"
#include "log.h"

//------------------------------------------------------------------------------
// Module constant defines
//------------------------------------------------------------------------------
#define TAG "main"

//------------------------------------------------------------------------------
// External variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// External functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module type definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module static variables
//------------------------------------------------------------------------------
static volatile uint32_t s_systickCount = 0;

//------------------------------------------------------------------------------
// Module static function prototypes
//------------------------------------------------------------------------------
static void SysTick_Init(void);
static void WDT_Init(uint16_t reload_val, uint8_t prescaler);
static void WDT_Pet(void);

//------------------------------------------------------------------------------
// Module externally exported functions
//------------------------------------------------------------------------------

/**
 * @brief  Application entry point
 * @param  None
 * @return None
 */
int main(void)
{
    SystemInit();
    SysTick_Init();

    const bool debuggerAttached = !WaitForDebuggerToAttach(1000);
    if (debuggerAttached)
    {
        LOG_Init(eLOG_LEVEL_INFO, (uint32_t *)&s_systickCount);
    }
    else
    {
        LOG_Init(eLOG_LEVEL_NONE, (uint32_t *)&s_systickCount);
    }

    WDT_Init(0x0FFF, IWDG_Prescaler_128);

    FDX_B_Init();

    uint32_t lastTick = s_systickCount;
    while (1)
    {

        if (FDX_B_Available())
        {
            FDX_B_t tag;
            FDX_B_Read(&tag);

            if (!tag.valid)
            {
                LOGE(TAG, "Invalid Tag");
            }
            else
            {
                char idStr[16];
                FDX_B_ID_ToString(&tag, idStr);
                puts("ID:");
                puts(idStr);
                puts("Animal:");
                puts(tag.animal ? "Yes" : "No");
            }
        }

        // Pet the watchdog every second
        if (s_systickCount - lastTick > 1000)
        {
            lastTick = s_systickCount;
            WDT_Pet();
            LOGI(TAG, "Looking for tags");
        }
    }
}

//------------------------------------------------------------------------------
// Module static functions
//------------------------------------------------------------------------------

/**
 * @brief  Enable the SysTick module
 * @param  None
 * @return None
 */
static void SysTick_Init(void)
{
    // Disable default SysTick behavior
    SysTick->CTLR = 0;

    // Enable the SysTick IRQ
    NVIC_EnableIRQ(SysTicK_IRQn);

    // Set the tick interval to 1ms for normal op
    SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK / 1000) - 1;

    // Start at zero
    SysTick->CNT = 0;
    s_systickCount = 0;

    // Enable SysTick counter, IRQ, HCLK/1
    SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
}

/**
 * @brief  Initialize the watchdog timer
 * @param reload_val - the value to reload the counter with
 * @param prescaler - the prescaler to use
 * @return None
 */
static void WDT_Init(uint16_t reload_val, uint8_t prescaler)
{
    IWDG->CTLR = 0x5555;
    IWDG->PSCR = prescaler;

    IWDG->CTLR = 0x5555;
    IWDG->RLDR = reload_val & 0xfff;

    IWDG->CTLR = 0xCCCC;
}

/**
 * @brief  Pet the watchdog timer
 * @param  None
 * @return None
 */
static void WDT_Pet(void)
{
    IWDG->CTLR = 0xAAAA;
}

/**
 * @brief  SysTick interrupt handler
 * @param  None
 * @return None
 * @note   __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
    // Set the next interrupt to be in 1/1000th of a second
    SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK / 1000);

    // Clear IRQ
    SysTick->SR = 0;

    // Update counter
    s_systickCount++;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

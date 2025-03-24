//------------------------------------------------------------------------------
//       Filename: fdxb.c
//------------------------------------------------------------------------------
//       Bogdan Ionescu (c) 2025
//------------------------------------------------------------------------------
//       Purpose : Implements the FDX-B module interface
//------------------------------------------------------------------------------
//       Notes : None
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Module includes
//------------------------------------------------------------------------------
#include <ch32fun.h>

#include "fdxb.h"
#include "log.h"

//------------------------------------------------------------------------------
// Module constant defines
//------------------------------------------------------------------------------
#define TAG "FDX-B"

#ifndef US_TO_TICKS
#define US_TO_TICKS(us) ((us) * (FUNCONF_SYSTEM_CORE_CLOCK / 1000000))
#endif

#ifndef BITMASK
#define BITMASK(x) ((1 << (x)) - 1)
#endif

#if CONFIG_INLINE_RIGHT_SHIFT
#define RIGHT_SHIFT(data, shift)                                                             \
    do                                                                                       \
    {                                                                                        \
        (data)[0] = ((data)[0] >> (shift)) | ((data)[1] & BITMASK(shift)) << (32 - (shift)); \
        (data)[1] = ((data)[1] >> (shift)) | ((data)[2] & BITMASK(shift)) << (32 - (shift)); \
        (data)[2] = ((data)[2] >> (shift)) | ((data)[3] & BITMASK(shift)) << (32 - (shift)); \
        (data)[3] = ((data)[3] >> (shift));                                                  \
    } while (0)
#else
static void RIGHT_SHIFT(uint32_t data[4], size_t shift)
{
    data[0] = (data[0] >> shift) | (data[1] & BITMASK(shift)) << (32 - shift);
    data[1] = (data[1] >> shift) | (data[2] & BITMASK(shift)) << (32 - shift);
    data[2] = (data[2] >> shift) | (data[3] & BITMASK(shift)) << (32 - shift);
    data[3] = (data[3] >> shift);
}
#endif

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
static volatile uint32_t s_buffer[2][4] = {0};
static volatile int s_activeBuffer = 0;
static volatile bool s_tagAvailable = 0;

static volatile size_t s_bitCount = 0;
static volatile uint32_t s_bitMask = 0;
static volatile size_t s_bitCountMax = 0;
static volatile uint32_t s_lastPinChange = 0;
static volatile bool s_lastLastWasShort = false;

// Bits I want to check
static const uint32_t c_checkBitsMask[4] = {
    0b00100000000100000000111111111111,
    0b00000001000000001000000001000000,
    0b00010000000010000000010000000010,
    0b00000000100000000100000000100000,
};

// Values for bits to check
static const uint32_t c_validBits[4] = {
    0b00100000000100000000100000000001,
    0b00000001000000001000000001000000,
    0b00010000000010000000010000000010,
    0b00000000100000000100000000100000,
};

//------------------------------------------------------------------------------
// Module static function prototypes
//------------------------------------------------------------------------------
static void SetDeadTime(TIM_TypeDef *timer, uint16_t deadtime);
static void PWM_Init(void);
static void InputCapture_Init(void);
static void SetupOpAmp(int channel);
static uint16_t CRC16K(uint16_t crc, const uint8_t val);
static char mod10(uint64_t *u);
static void Load(FDX_B_t *tag, uint32_t raw[4]);

//------------------------------------------------------------------------------
// Module externally exported functions
//------------------------------------------------------------------------------

/**
 * @brief  Initialize the FDX-B module
 * @param  None
 * @return None
 */
void FDX_B_Init(void)
{
    PWM_Init();
    Delay_Ms(100);
    InputCapture_Init();
}

/**
 * @brief  Check if a tag is available to read
 * @param  None
 * @return true if a tag is available
 */
bool FDX_B_Available(void)
{
    return s_tagAvailable;
}

/**
 * @brief  Read the tag
 * @param[out] tag - The tag to read into
 * @return None
 */
void FDX_B_Read(FDX_B_t *tag)
{
    // Disable the interrupt while we read the tag
    NVIC_DisableIRQ(EXTI7_0_IRQn);

    // Load from the inactive buffer
    Load(tag, (uint32_t *)s_buffer[s_activeBuffer ^ 1]);
    s_tagAvailable = false;

    NVIC_EnableIRQ(EXTI7_0_IRQn);
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void EXTI7_0_IRQHandler(void)
{
    const uint32_t time = SysTick->CNT;
    const uint32_t delta = time - s_lastPinChange;
    s_lastPinChange = time;

    // ignore weird values
    if (delta > US_TO_TICKS(400) || delta < US_TO_TICKS(50))
    {
        goto done;
    }

    // Biphase decoding: long pulse = 1, 2 short pulses = 0
    bool bit = true;
    if (delta < US_TO_TICKS(175))
    {
        if (s_lastLastWasShort)
        {
            bit = false;
            s_lastLastWasShort = false;
        }
        else
        {
            s_lastLastWasShort = true;
            goto done;
        }
    }

    // Equivalent to word = s_bitCount / 32, but does not require division
    const size_t word = s_bitCount >> 5;

    // Insert the bit into the buffer
    // Equivalent to bit |= bit << (s_bitCount % 32)
    s_buffer[s_activeBuffer][word] |= bit << (s_bitCount & BITMASK(5));

    // Reset the bit mask if we have a full word
    if (s_bitMask == 0xFFFFFFFF)
    {
        s_bitMask = 0;
    }

    // Add another bit to the mask
    s_bitMask |= 1 << (s_bitCount & BITMASK(5));
    ++s_bitCount;

    // Example Data:
    // bit: 1
    // word: 0
    // s_bitCount: 14
    // s_bitMask:             00000000000000000011111111111111 <- only check ignested bits
    // c_checkBitsMask[word]: 00100000000100000000111111111111 <- these are the bits that need to match
    // c_validBits[word]:     00100000000100000000100000000001 <- these are the values they need to be
    // s_buffer[word]:        00000000000000000011100000000001 <- this is the current buffer
    // checkBits:             00000000000000000000100000000001 <- this is what we expect to see
    // maskedBits:            00000000000000000000100000000001 <- this is what we have

    // We only need to check the latest word
    const uint32_t checkBits = (c_validBits[word] & c_checkBitsMask[word]) & s_bitMask;
    const uint32_t maskedBits = (s_buffer[s_activeBuffer][word] & c_checkBitsMask[word]) & s_bitMask;

    if (maskedBits != checkBits)
    {
        // Make last ingested bit the first bit in the buffer
        s_bitCount = 1;
        s_bitMask = 1;
        s_buffer[s_activeBuffer][0] = bit;
        goto done;
    }

    // We have a full tag
    if (s_bitCount == 128)
    {
        // Reset the buffer
        s_bitCount = 0;
        s_bitMask = 0;
        // Swap the buffers
        s_activeBuffer ^= 1;
        s_tagAvailable = true;
    }

done:
    // Acknowledge the interrupt
    EXTI->INTFR = EXTI_Line0;
}

/**
 * @brief  Convert the ID to a string
 * @param[in] tag - The tag to convert
 * @param[out] str - The string to write to
 * @return None
 */
void FDX_B_ID_ToString(const FDX_B_t *tag, char str[16])
{
    uint64_t country = tag->country;
    for (int i = 0; i < 3; ++i)
    {
        if (country >= 10)
        {
            str[2 - i] = mod10(&country) + '0';
        }
        else if (country == 0)
        {
            str[2 - i] = '0';
        }
        else
        {
            str[2 - i] = country + '0';
            break;
        }
    }

    size_t cursor = 0;
    uint64_t id = tag->id;
    while (id >= 10)
    {
        str[15 - (++cursor)] = mod10(&id) + '0';
    }
    str[15 - (++cursor)] = mod10(&id) + '0';

    // padding
    while (cursor < 12)
    {
        str[15 - (++cursor)] = '0';
    }

    str[15] = '\0';
}

//------------------------------------------------------------------------------
// Module static functions
//------------------------------------------------------------------------------

/**
 * @brief Set deadtime
 * @param  timer - The timer to set the deadtime for
 * @param  deadtime - The deadtime value
 * @return None
 * @note
   Deadband setting bits that define the duration of the deadband between complementary outputs.
   Assume that DT denotes its duration.
   DTG[7:5]=0xx=>DT=DTG[7:0]*TDTS；
   DTG[7:5]=10x=>DT=(64+DTG[5:0])*2*TDTS;
   DTG[7:5]=110=>DT=(32+DTG[4:0])*8*TDTS;
   DTG[7:5]=111=>DT=(32+DTG[4:0])*16*TDTS.
 */
static void SetDeadTime(TIM_TypeDef *timer, uint16_t deadtime)
{
    timer->BDTR = (timer->BDTR & ~TIM_DTG) | (deadtime & TIM_DTG);
}

/**
 * @brief  Initialize PWM
 * @param  None
 * @return None
 */
static void PWM_Init(void)
{

    RCC->APB2PCENR |= RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA;

    /*AFIO->PCFR1 |= GPIO_PartialRemap1_TIM1;*/

    // PA1 is T1CH2, 10MHz Output alt func, push-pull
    funPinMode(PA1, GPIO_CFGLR_OUT_2Mhz_AF_PP);
    // PA2 is T1CH2N, 10MHz Output alt func, push-pull
    funPinMode(PA2, GPIO_CFGLR_OUT_2Mhz_AF_PP);

    // Reset TIM1 to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // CTLR1: default is up, events generated, edge align
    // SMCFGR: default clk input is CK_INT

    // Prescaler
    TIM1->PSC = 0x0000;

    // Auto Reload - sets period
    TIM1->ATRLR = CONFIG_RFID_FREQ_VAL;

    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;

    // Enable CH2 output, normal polarity and CH2N output, inverted polarity
    TIM1->CCER |= TIM_CC2E | TIM_CC2NE | TIM_CC2NP;

    // CH2 Mode is output, PWM1 (CC3S = 00, OC3M = 110)
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

    // Set the Capture Compare Register value to off
    TIM1->CH2CVR = TIM1->ATRLR >> 1; // 50% duty cycle

    // Enable TIM1 outputs
    TIM1->BDTR |= TIM_MOE;

    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;

    SetDeadTime(TIM1, 0x4);
}

/**
 * @brief  Set up the Op-Amp for current sensing
 * @param  channel - The channel to set up
 * @return None
 */
static void SetupOpAmp(int channel)
{
    RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA;

    // Set the Op-Amp Output to Floating
    funPinMode(PD4, GPIO_CFGLR_IN_ANALOG);

    // Set the Op-Amp Input Positive and Negative to Floating
    if (channel == 0)
    {
        funPinMode(PA1, GPIO_CFGLR_IN_ANALOG);
        funPinMode(PA2, GPIO_CFGLR_IN_ANALOG);

        // Set Op-Amp pins to OPP0 and OPN0
        EXTEN->EXTEN_CTR &= ~(EXTEN_OPA_NSEL | EXTEN_OPA_PSEL);
    }
    else
    {
        funPinMode(PD0, GPIO_CFGLR_IN_ANALOG);
        funPinMode(PD7, GPIO_CFGLR_IN_ANALOG);

        // Set Op-Amp pins to OPP1 and OPN1
        EXTEN->EXTEN_CTR |= EXTEN_OPA_PSEL | EXTEN_OPA_NSEL;
    }

    EXTEN->EXTEN_CTR |= EXTEN_OPA_EN;
}

/**
 * @brief  Initialize Input Capture
 * @param  None
 * @return None
 */
static void InputCapture_Init(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    // PC0 is TIM2_CH3
    funPinMode(PC0, GPIO_CFGLR_IN_FLOAT);

    // Configure the IO as an interrupt.
    AFIO->EXTICR = AFIO_EXTICR1_EXTI0_PC;
    EXTI->INTENR = EXTI_INTENR_MR0; // Enable EXT0
    EXTI->RTENR = EXTI_RTENR_TR0;   // Rising edge trigger
    EXTI->FTENR = EXTI_FTENR_TR0;   // Falling edge trigger

    // enable interrupt
    NVIC_EnableIRQ(EXTI7_0_IRQn);
}

/**
 * @brief  Calculate the CRC16-K checksum
 * @param[in] crc = The current CRC16-K checksum
 * @param[in] val = The value to add to the checksum
 * @return The new CRC16-K checksum
 */
static uint16_t CRC16K(uint16_t crc, const uint8_t val)
{
    crc ^= val;
    for (uint8_t k = 0; k < 8; k++)
    {
        crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }

    return crc;
}

/**
 * @brief  Modulo 10
 * @param[inout] u - The number to mod 10
 * @return The remainder
 */
static char mod10(uint64_t *u)
{
    uint64_t dividend = *u;
    uint64_t quo = 0;
    uint64_t div = 0xA000000000000000; // largest shifted 10
    while (div >= 10)
    {
        quo <<= 1; // quo *= 2;
        if (dividend >= div)
        {
            dividend -= div;
            quo++;
        }
        div >>= 1;
    }
    *u = quo;
    return (char)dividend;
}

/**
 * @brief  Load the FDX-B tag from the raw data
 * @param[out] tag - The tag to load
 * @param[in] raw - The raw data to load from
 * @return None
 */
static void Load(FDX_B_t *tag, uint32_t raw[4])
{
    uint16_t crc = 0;
    tag->id = 0;
    tag->valid = false;

    RIGHT_SHIFT(raw, 11 + 1); // shift out header + control bit
    for (int i = 0; i < 4; ++i)
    {
        crc = CRC16K(crc, raw[0] & 0xff);
        tag->id |= (uint64_t)(raw[0] & 0xff) << (i * 8);
        RIGHT_SHIFT(raw, 8 + 1);
    }
    crc = CRC16K(crc, raw[0] & 0xff);
    tag->id |= (uint64_t)(raw[0] & BITMASK(6)) << (4 * 8);
    RIGHT_SHIFT(raw, 6);

    tag->country = (raw[0] & BITMASK(2));
    RIGHT_SHIFT(raw, 2 + 1);
    crc = CRC16K(crc, raw[0] & 0xff);
    tag->country |= (raw[0] & BITMASK(8)) << 2;
    RIGHT_SHIFT(raw, 8 + 1);

    crc = CRC16K(crc, raw[0] & 0xff);
    tag->flags = (raw[0] & BITMASK(8));
    RIGHT_SHIFT(raw, 8 + 1);
    crc = CRC16K(crc, raw[0] & 0xff);
    tag->flags |= (raw[0] & BITMASK(8)) << 8;
    RIGHT_SHIFT(raw, 8 + 1);

    tag->checksum = (raw[0] & BITMASK(8));
    RIGHT_SHIFT(raw, 8 + 1);
    tag->checksum |= (raw[0] & BITMASK(8)) << 8;
    RIGHT_SHIFT(raw, 8 + 1);

#if CONFIG_READ_EXTRA_DATA
    for (int i = 0; i < 3; ++i)
    {
        tag->data[i] = (raw[0] & BITMASK(8));
        RIGHT_SHIFT(raw, 8 + 1);
    }
#endif

    if (crc != tag->checksum)
    {
        LOGE(TAG, "Checksum error %04X != %04X", crc, tag->checksum);
        return;
    }

    tag->valid = true;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

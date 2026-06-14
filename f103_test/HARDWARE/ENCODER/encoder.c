#include "encoder.h"
#include "tim.h"

static volatile int g_encoder_a_count = 0;
static volatile int g_encoder_b_count = 0;
static uint8_t g_encoder_a_state = 0;
static uint8_t g_encoder_b_state = 0;

static uint8_t encoder_read_state(GPIO_TypeDef *a_port, uint16_t a_pin,
                                  GPIO_TypeDef *b_port, uint16_t b_pin)
{
    uint8_t a = (HAL_GPIO_ReadPin(a_port, a_pin) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(b_port, b_pin) == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((a << 1) | b);
}

static int8_t encoder_step(uint8_t old_state, uint8_t new_state)
{
    static const int8_t table[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    return table[(old_state << 2) | new_state];
}

void encoder_exti_callback(uint16_t GPIO_Pin)
{
    uint8_t new_state;

    if ((GPIO_Pin == ENC_A1_Pin) || (GPIO_Pin == ENC_A2_Pin))
    {
        new_state = encoder_read_state(ENC_A1_GPIO_Port, ENC_A1_Pin,
                                       ENC_A2_GPIO_Port, ENC_A2_Pin);
        g_encoder_a_count += encoder_step(g_encoder_a_state, new_state);
        g_encoder_a_state = new_state;
    }
    else if ((GPIO_Pin == ENC_B1_Pin) || (GPIO_Pin == ENC_B2_Pin))
    {
        new_state = encoder_read_state(ENC_B1_GPIO_Port, ENC_B1_Pin,
                                       ENC_B2_GPIO_Port, ENC_B2_Pin);
        g_encoder_b_count += encoder_step(g_encoder_b_state, new_state);
        g_encoder_b_state = new_state;
    }
}

int read_pluse(TIM_HandleTypeDef *htim)
{
    int pluse = 0;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (htim == &htim2)
    {
        pluse = g_encoder_a_count;
        g_encoder_a_count = 0;
    }
    else if (htim == &htim4)
    {
        pluse = g_encoder_b_count;
        g_encoder_b_count = 0;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return pluse;
}

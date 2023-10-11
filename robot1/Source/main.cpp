/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"

//LineFollower_s LineFollower;

constexpr auto MOTOR_MAX_RPM = 2800; //2800, 4300
constexpr auto ENCODER_REDUCTION = 5.4;//1:5.4, 4.4
constexpr auto ENCODER_PPR_HAL = 11;//11, 1? 13?
constexpr auto ENCODER_WHEEL_DIAMETER_MM = 25;
constexpr auto ENCODER_WHEEL_DIAMETER = 0.026;
#define TIMER_PRESCALE      250     // 8-bit value
#define TIMER_WRAP          125000  // 17-bit value
#define SAMPLE_FREQ         (125000000 / (TIMER_PRESCALE*TIMER_WRAP))
 
/* DRV 1
 * MOTOR     A - 15
 * MOTOR     B -  9
 * MOTOR  PWMA - PWM7B
 * MOTOR  PWMB - PWM4B
 * ENC       A - 23
 * ENC       B - 24
 * ENC PWM CNT - PWM3
 * ENC PWM TIM - PWM2
 */
constexpr uint DRIVE1_MOTOR_PINA = 15;
constexpr uint DRIVE1_MOTOR_PINB = 9;
constexpr uint DRIVE1_MOTOR_PWMA = 7;
constexpr uint DRIVE1_MOTOR_PWMB = 4;
constexpr uint DRIVE1_ENCODER_PINA = 23;
constexpr uint DRIVE1_ENCODER_PINB = 24;
constexpr uint DRIVE1_ENCODER_PWM = 3;
/* DRV 2
 * MOTOR     A - 14
 * MOTOR     B -  8
 * MOTOR  PWMA - PWM7A
 * MOTOR  PWMB - PWM4A
 * ENC       A -  1
 * ENC       B -  3
 * ENC PWM CNT - PWM0
 * ENC PWM TIM - PWM1
 */
constexpr uint DRIVE2_MOTOR_PINA = 14;
constexpr uint DRIVE2_MOTOR_PINB = 8;
constexpr uint DRIVE2_MOTOR_PWMA = 7;
constexpr uint DRIVE2_MOTOR_PWMB = 4;
constexpr uint DRIVE2_ENCODER_PINA = 1;
constexpr uint DRIVE2_ENCODER_PINB = 3;
constexpr uint DRIVE2_ENCODER_PWM = 0;

void motor_init(uint pinA, uint pinB, uint pwmA, uint pwmB)
{
    gpio_set_function(pinA, GPIO_FUNC_PWM);
    gpio_set_function(pinB, GPIO_FUNC_PWM);
    pwm_set_wrap(pwmA, 0xFFFF);
    pwm_set_wrap(pwmB, 0xFFFF);
    pwm_set_clkdiv(pwmA, 1.0f);
    pwm_set_clkdiv(pwmB, 1.0f);
    pwm_set_enabled(pwmA, true);
    pwm_set_enabled(pwmB, true);
}

void setDuty(float _Duty, uint pinA, uint pinB)
{
    int32_t pwm = _Duty * 0xFFFF;
    int32_t max = 0xFFFF;

    uint val1;
    uint val2;

    if (pwm > 0) {
        
        val1 = max;
        val2 = pwm > max ? 0 : max - pwm;
    }
    else if (pwm < 0) {
        val1 = -pwm > max ? 0 : max + pwm;
        val2 = max;
    }
    else {
        val1 = max;
        val2 = max;
    }

    pwm_set_gpio_level(pinA, val1);
    pwm_set_gpio_level(pinB, val2);
}





int main()
{
    //LineFollower.init();

    // while (true)
    // {
    //     //LineFollower.run();
    // }
    int new_value0, new_value1, delta0, delta1, old_value0 = 0, old_value1 = 0, per1 = 0, per2 = 0;

    stdio_init_all();
    //pio_add_program(pio1, &quadrature_encoder_program);
    //quadrature_encoder_program_init(pio1, 0, 23, 2, 0);
    //quadrature_encoder_program_init(pio1, 1, 1, 3, 0);

    motor_init(DRIVE1_MOTOR_PINA, DRIVE1_MOTOR_PINB, DRIVE1_MOTOR_PWMA, DRIVE1_MOTOR_PWMB);
    motor_init(DRIVE2_MOTOR_PINA, DRIVE2_MOTOR_PINB, DRIVE2_MOTOR_PWMA, DRIVE2_MOTOR_PWMB);

    //encoder_init(DRIVE1_ENCODER_PINA, DRIVE1_ENCODER_PWM);
    //encoder_init(DRIVE2_ENCODER_PINA, DRIVE2_ENCODER_PWM);

    setDuty(1.0f, DRIVE1_MOTOR_PINA, DRIVE1_MOTOR_PINB);
    setDuty(1.0f, DRIVE2_MOTOR_PINA, DRIVE2_MOTOR_PINB);

    //encoder_start(DRIVE1_ENCODER_PWM);
    //encoder_start(DRIVE2_ENCODER_PWM);
    absolute_time_t upTime = get_absolute_time();
    
    while (1) {
        absolute_time_t TimeNow = get_absolute_time();
        if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(upTime))
        {
            //per1 = encoder_read(DRIVE1_ENCODER_PWM);
            //per2 = encoder_read(DRIVE2_ENCODER_PWM);


            //new_value0 = quadrature_encoder_get_count(pio1, 0);
            //new_value1 = quadrature_encoder_get_count(pio1, 1);
            delta0 = new_value0 - old_value0;
            old_value0 = new_value0;
            delta1 = new_value1 - old_value1;
            old_value1 = new_value1;

            printf("p0 %8d, d0 %6d, p1 %8d, d1 %6d, per1%8d, per2%8d\n", new_value0, delta0, new_value1, delta1, per1, per2);
            upTime = make_timeout_time_ms(250);
        }
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        
        //sleep_ms(100);
    }

    return 0;
}



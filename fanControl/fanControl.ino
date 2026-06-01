void CH32V003_SERVO_Init(void)
{
    SystemCoreClockUpdate();
    if (s_initialized) return;
    s_initialized = 1;

    /* ----- Enable peripheral clocks ----- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);

    /* ----- GPIO: PC4 (TIM1_CH4) as AF push-pull ----- */
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_4;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* ----- TIM1 time-base: 50 Hz, 1 us resolution ----- */
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = SERVO_ARR;
    tb.TIM_Prescaler     = SERVO_PSC;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    /* ----- CH4 (PC4): PWM mode 2, edge-aligned ----- */
    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = SERVO_DEFAULT;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);

    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_Cmd(TIM1, ENABLE);    
}

void CH32V003_SERVO_WriteCH4(uint16_t us)
{
    TIM_SetCompare4(TIM1, clamp_us(us));
}
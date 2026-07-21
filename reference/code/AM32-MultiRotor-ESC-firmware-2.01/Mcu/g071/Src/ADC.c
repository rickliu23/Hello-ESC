/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
#include "ADC.h"

#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
uint16_t ADCDataDMA[3];
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 64)

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables

#ifdef USE_ADC_INPUT
  ADC_raw_temp = ADCDataDMA[3];
  ADC_raw_volts = ADCDataDMA[1] / 2;
  ADC_raw_current = ADCDataDMA[2];
  ADC_raw_input = ADCDataDMA[0];

#else
  ADC_raw_temp = ADCDataDMA[2];
  ADC_raw_volts = ADCDataDMA[1];
  ADC_raw_current = ADCDataDMA[0];
#endif
}

// 配置并使能 ADC1 的 DMA 传输
// 设置 DMA 源地址、目标地址、传输长度、中断，并启动 DMA 通道
void enableADC_DMA()
{
  // 配置 DMA 中断优先级并使能中断
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  // 配置 DMA 传输地址：ADC1 规则数据寄存器 → ADCDataDMA 数组
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_2,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&ADCDataDMA,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  // 根据是否使用 ADC 输入，设置 DMA 传输数据长度
#ifdef USE_ADC_INPUT
  // ADC 输入模式：4 个数据（输入 + 电流 + 电压 + 温度）
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_2,
                       4);
#else
  // 普通模式：3 个数据（电流 + 电压 + 温度）
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_2,
                       3);

#endif
  // 使能 DMA 传输完成中断
  LL_DMA_EnableIT_TC(DMA1,
                     LL_DMA_CHANNEL_2);

  // 使能 DMA 传输错误中断
  LL_DMA_EnableIT_TE(DMA1,
                     LL_DMA_CHANNEL_2);

  // 启动 DMA1 通道 2 传输
  LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_2);
}

// 启动 ADC1：使能内部稳压器、校准、使能 ADC，并开启温度传感器
void activateADC(void)
{
  __IO uint32_t wait_loop_index = 0U;
  __IO uint32_t backup_setting_adc_dma_transfer = 0U;

  // 如果 ADC 尚未使能，则执行启动流程
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    // 使能 ADC 内部稳压器，等待稳定
    LL_ADC_EnableInternalRegulator(ADC1);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    // 校准时需要暂时关闭 DMA 传输请求，校准完成后再恢复
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

    // 启动 ADC 自校准
    LL_ADC_StartCalibration(ADC1);

    // 等待校准完成
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    }

    // 恢复之前的 DMA 传输设置
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

    // 校准结束到使能 ADC 之间需要延时
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    // 使能 ADC
    LL_ADC_Enable(ADC1);

    // 等待 ADC 就绪
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    }

    // 使能内部温度传感器（通过 ADC 通用控制寄存器）
    ADC->CCR |= ADC_CCR_TSEN;
  }
}

// 初始化 ADC1：配置电流、电压、温度三个采样通道，使用 DMA 循环传输
void ADC_Init(void)
{
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};  // ADC 规则组初始化结构体
  LL_ADC_InitTypeDef ADC_InitStruct = {0};          // ADC 全局初始化结构体

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};        // GPIO 初始化结构体

  /* 使能 ADC 和 GPIOA 时钟 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* 配置电压检测引脚为模拟输入 */
  GPIO_InitStruct.Pin = VOLTAGE_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 配置电流检测引脚为模拟输入 */
  GPIO_InitStruct.Pin = CURRENT_ADC_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA 配置：DMA1 通道 2 */
  // 选择 DMA 请求源为 ADC1
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_ADC1);
  // 传输方向：外设（ADC 数据寄存器）→ 内存（ADC 缓冲区）
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  // DMA 通道优先级：高
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
  // 循环模式：转换完成后自动从头开始，持续更新 ADC 缓冲区
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
  // 外设地址不递增（始终指向 ADC DR）
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  // 内存地址递增（依次填充 ADC 缓冲区数组）
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  // 外设数据宽度：32 位（读取 ADC_DR）
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_WORD);
  // 内存数据宽度：16 位（半字，匹配 12 位 ADC 结果）
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

  /* 使能芯片内部温度传感器通道 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CHANNEL_TEMPSENSOR);

  /* 配置 ADC 规则组：软件触发，扫描 3 个通道，单次转换模式 */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;              // 软件触发启动转换
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;   // 扫描序列包含 3 个通道
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;     // 禁用间断模式
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;               // 单次扫描模式（非连续）
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;         // 每次触发产生一次 DMA 请求
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;               // 数据溢出时保留旧数据
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_FALLING);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);                    // 禁用过采样
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_LOW);         // 低触发频率模式
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);   // 序列可配置
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_ASYNC_DIV4);                           // ADC 异步时钟 4 分频
  // 通用采样时间 1：19.5 个 ADC 时钟，用于电流/电压（输入阻抗较低，快速采样）
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  // 通用采样时间 2：160.5 个 ADC 时钟，用于温度传感器（内部信号需要更长时间建立）
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);  // 禁用转换结束中断（用 DMA，不需要中断）
  LL_ADC_DisableIT_EOS(ADC1);  // 禁用序列结束中断

  /* ADC 基本参数：12 位分辨率，右对齐，不启用低功耗模式 */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* 配置扫描序列：Rank1=电流，Rank2=电压，Rank3=温度传感器 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, CURRENT_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC1, CURRENT_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_COMMON_1);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, VOLTAGE_ADC_CHANNEL);
  LL_ADC_SetChannelSamplingTime(ADC1, VOLTAGE_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_COMMON_1);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_COMMON_2);
}

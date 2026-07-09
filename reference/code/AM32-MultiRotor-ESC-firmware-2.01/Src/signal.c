/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "targets.h"
#include "signal.h"
#include "IO.h"
#include "dshot.h"
#include "serial_telemetry.h"
#include "functions.h"
#include "sounds.h"
#include "common.h"

int max_servo_deviation = 200; // 速度变化限制：加速/减速的变化速率不能过快，保护电机和电调
int servorawinput;

uint8_t enter_calibration_count = 0;
uint8_t calibration_required = 0; // 当用户长时间将油门推到最大（>1500 且持续超过 50 次采样），ESC 认为用户在请求"行程校准"，于是把 calibration_required 置 1
uint8_t high_calibration_counts = 0;
uint8_t high_calibration_set = 0; // 高位行程校准标志位，为1时代表校准完毕
uint16_t last_high_threshold = 0;
uint8_t low_calibration_counts = 0;
uint16_t last_input = 0;

void computeMSInput()
{

	int lastnumber = dma_buffer[0];
	for (int j = 1; j < 2; j++)
	{

		if (((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0))
		{ // blank space

			newinput = map((dma_buffer[j] - lastnumber), 243, 1200, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

// 计算输入脉宽
void computeServoInput()
{
	// 判断输入范围
	// 正常范围时1000~2000，判断时放宽
	if (((dma_buffer[1] - dma_buffer[0]) > 800) && ((dma_buffer[1] - dma_buffer[0]) < 2200))
	{
		if (calibration_required) // 如果在校准状态
		{
			if (!high_calibration_set) // 高位还没校准
			{
				if (high_calibration_counts == 0)
				{
					// 记录初始值
					last_high_threshold = dma_buffer[1] - dma_buffer[0];
				}

				// 累加，表示连续采样次数
				high_calibration_counts++;

				// 油门变化还比较大，数据无效
				if (getAbsDif(last_high_threshold, servo_high_threshold) > 50)
				{
					calibration_required = 0;
				}
				else
				{
					// 一阶低通滤波：新值 = (旧值*7 + 新采样) / 8
					servo_high_threshold = ((7 * servo_high_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);

					// 且稳定次数比较多
					if (high_calibration_counts > 50)
					{
						// 留一定余量
						// 最后的25范围都认为是最大油门
						// 即使有抖动或者摇杆异常顶不到最大值，也可以稳定输出最大值
						servo_high_threshold = servo_high_threshold - 25;

						// 实际校准时，这个值一般在 1750 ~ 2250 µs 之间。
						// 减去 1750：把起始点移到 0，在0~500之间
						// 除以 2：把精度降到 2 µs / LSB，同时最大值 500 / 2 = 250
						// 这样就可以用一个字节来保存数据
						// 后续加载数据时，做一遍反向运算
						eepromBuffer[33] = (servo_high_threshold - 1750) / 2; // 保存最大油门

						high_calibration_set = 1; // 高位校准ok

						playDefaultTone(); // 播放音效
					}
				}
				last_high_threshold = servo_high_threshold;
			}

			if (high_calibration_set) // 高位已经校准
			{
				// 正常范围时1000~2000，判断时放宽
				// 此处检测低油门
				if (dma_buffer[1] - dma_buffer[0] < 1250)
				{
					low_calibration_counts++;

					// 滤波
					servo_low_threshold = ((7 * servo_low_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
				}

				if (low_calibration_counts > 75)
				{
					// 参考满油门校准
					servo_low_threshold = servo_low_threshold + 25;
					eepromBuffer[32] = (servo_low_threshold - 750) / 2;
					calibration_required = 0;
					saveEEpromSettings();
					low_calibration_counts = 0;
					playChangedTone();
				}
			}

			signaltimeout = 0;
		}
		else
		{
			if (bi_direction)
			{
				if (dma_buffer[1] - dma_buffer[0] <= servo_neutral)
				{
					servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_neutral, 0, 1000);
				}
				else
				{
					servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_neutral + 1, servo_high_threshold, 1001, 2000);
				}
			}
			else
			{
				servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_high_threshold, 47, 2047);
				if (servorawinput == 47)
				{
					servorawinput = 0;
				}
			}

			signaltimeout = 0;
		}
	}
	else
	{
		// 否则直接认为输入信号无效，不计入零输入
		// 需要确认一下如果是正常运行过程中，是否会到这里
		zero_input_count = 0;
	}

#ifdef SLOW_RAMP_DOWN
	// 只有F051用到了，忽略
	if (forward)
	{
		if ((servorawinput - newinput) > max_servo_deviation)
		{
			newinput += max_servo_deviation;
		}
		else if ((newinput - servorawinput) > (max_servo_deviation >> 2))
		{
			newinput -= (max_servo_deviation >> 2);
		}
		else
		{
			newinput = servorawinput;
		}
	}
	else
	{
		if ((servorawinput - newinput) > max_servo_deviation >> 2)
		{
			newinput += max_servo_deviation >> 2;
		}
		else if ((newinput - servorawinput) > (max_servo_deviation))
		{
			newinput -= (max_servo_deviation);
		}
		else
		{
			newinput = servorawinput;
		}
	}
#else
	// 计算油门
	if ((servorawinput - newinput) > max_servo_deviation)
	{
		newinput += max_servo_deviation;
	}
	else if ((newinput - servorawinput) > max_servo_deviation)
	{
		newinput -= max_servo_deviation;
	}
	else
	{
		newinput = servorawinput;
	}
#endif
}

// 外部输入信号捕获完成
void transfercomplete()
{
	if (armed && dshot_telemetry) // 已解锁且双向
	{
		if (out_put)
		{
			// 发送完毕，切换回接受
			receiveDshotDma();

			return;
		}
		else
		{
			// 先发送上一帧数据，再准备当前帧数据？
			sendDshotDma();
			make_dshot_package();

			computeDshotDMA(); // 解码飞控发过来的 DShot 命令

			return; // 直接返回
		}
	}

	if (inputSet == 0) // 还不知道是什么协议格式
	{
		detectInput();	   // 分析协议
		receiveDshotDma(); // 启动下一次DMA接收
		return;
	}

	if (inputSet == 1) // 已经知道是什么协议
	{
		if (dshot_telemetry) // 双向协议
		{
			if (out_put) // 当前是输出模式
			{
				//    	TIM17->CNT = 0;
				make_dshot_package(); // this takes around 10us !!
				computeDshotDMA();	  // this is slow too..
				receiveDshotDma();	  // holy smokes.. reverse the line and set up dma again
				return;
			}
			else
			{
				sendDshotDma();
				return;
			}
		}
		else // 单向协议
		{
			if (dshot == 1) // DShot协议
			{
				computeDshotDMA(); // 解码飞控发过来的 DShot 命令
				if (send_telemetry)
				{
					// done in 10khz routine
				}
				receiveDshotDma(); // 继续接收
			}

			if (servoPwm == 1) // servoPwm协议
			{
				// 确保当前 PWM 脉冲已经结束，避免在引脚还是高电平时重新配置定时器，导致捕获错误
				while ((INPUT_PIN_PORT->IDR & INPUT_PIN))
				{ // if the pin is high wait
				}

				// 计算脉宽
				computeServoInput();

				// 重置捕获极性为上升沿
				LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_RISING); // setup rising pin trigger.

				// 重新启动DMA接收
				receiveDshotDma();

				// 使能半传输中断
				// 第一次捕获到上升沿时，DMA 搬完 1 个数据，触发半传输中断。
				// 中断里把极性切换成 下降沿触发。
				// 第二次捕获到下降沿时，DMA 搬完第 2 个数据，触发传输完成中断，进入 transfercomplete()。
				LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
			}
		}

		if (!armed) // 还没解锁
		{
			if (adjusted_input < 0)
			{
				adjusted_input = 0; // 油门值大于等于0
			}

			// 油门为0 && 没有处于校准状态
			if (adjusted_input == 0 && calibration_required == 0)
			{
				// 累计，累计到达阈值之后解锁
				zero_input_count++;
			}
			else
			{
				// 存在非零油门信号，直接清零，防止一上电直接失控
				zero_input_count = 0;

				if (adjusted_input > 1500)
				{
					if (getAbsDif(adjusted_input, last_input) > 50) // 油门还在大幅度变化，不稳定
					{
						enter_calibration_count = 0;
					}
					else
					{
						// 此时油门稳定，一般是在最大值时才能做到稳定
						enter_calibration_count++;
					}

					// 达到校准结束的条件
					if (enter_calibration_count > 50 && (!high_calibration_set))
					{
						playBeaconTune3();		  // 播放校准完成影月
						calibration_required = 1; // 置为1，进入 Servo PWM 行程校准模式，其他代码中会去保存当前油门信息

						enter_calibration_count = 0; // 退出校准状态
					}
					last_input = adjusted_input;
				}
			}
		}
	}
}

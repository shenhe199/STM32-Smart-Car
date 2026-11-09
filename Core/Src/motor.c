#include "main.h"
#include "oled.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include <stdlib.h>

// ================= 外部全局变量声明 =================
extern TIM_HandleTypeDef htim2;        // 电机 PWM 定时器
extern TIM_HandleTypeDef htim3;        // 左轮编码器定时器
extern TIM_HandleTypeDef htim4;        // 右轮编码器定时器
extern volatile float current_angle;   // 当前角度
extern float gz_offset;                // 角速度的零点偏移量
// ====================================================

//停车函数
void Car_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

//前进函数
void Car_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

//左转弯函数
void Car_TurnLeft(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

//右转弯函数
void Car_TurnRight(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
}

//前进
void Car_Forward_Distance(float distance_cm) {
	
		int16_t current_pulse = 0;//定义平均脉冲
    int target_pulse = (int)(distance_cm * ENCODER_PER_CM);//目标脉冲数
    int accel_pulse = target_pulse * ACCEL_RATIO;//加速脉冲数
    int decel_pulse = target_pulse * DECEL_RATIO;//减速脉冲数
	
    __HAL_TIM_SET_COUNTER(&htim3, 0);//左轮脉冲调零
		__HAL_TIM_SET_COUNTER(&htim4, 0);//右轮脉冲调零
	
		int refresh_count = 0;//OLED屏幕刷新率
		uint32_t last_time = HAL_GetTick();//读取初始时间戳
		float target_yaw = current_angle;//初始方向
	
		float current_speed = TURN_MIN_SPEED;//定义当前速度
		Car_Forward(TURN_MIN_SPEED);
	
    while(1) {
			
        int16_t left_pulse = __HAL_TIM_GET_COUNTER(&htim3);//读取左轮脉冲
				int16_t right_pulse = __HAL_TIM_GET_COUNTER(&htim4);//读取右轮脉冲
        if (left_pulse < 0) left_pulse = -left_pulse; //绝对值
				if (right_pulse < 0) right_pulse = -right_pulse;//绝对值
			
				//退出循环条件
				current_pulse = (left_pulse + right_pulse) / 2;//取平均值
        if (current_pulse >= target_pulse) {
            break;
        }
				
				//================MPU6050防跑偏（读取）==================
				
				MPU6050_Read_All();//读取MPU6050数据
				float real_gz = (gz - gz_offset) / GYRO_SENSITIVITY;//真实角速度
				if (real_gz > -GYRO_DEADBAND && real_gz < GYRO_DEADBAND) real_gz = 0.0f;//死区过滤
	
				uint32_t current_time = HAL_GetTick();//读取当前时间戳
				float dt = (current_time - last_time) / 1000.0f;//真实时间（s）
				last_time = current_time; // 更新时间戳
        
				current_angle += real_gz * dt; //用真实时间进行积分
				
        // ===================梯形加减速=========================
				
        int remaining_pulse = target_pulse - current_pulse;
        //加速
        if (current_pulse < accel_pulse) {
            current_speed = MOTOR_MIN_SPEED + (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED) * current_pulse / accel_pulse;
        }
				//匀速
        else if (remaining_pulse < decel_pulse) {
            current_speed = MOTOR_MIN_SPEED + (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED) * remaining_pulse / decel_pulse;
        } 
				//减速
        else {
            current_speed = MOTOR_MAX_SPEED;
        }
        
				/*
        //（左轮功率偏低）硬件补偿防跑偏
        uint16_t left_speed =(int)( current_speed * MOTOR_K); 
				if(left_speed > 999) left_speed = 999;
    
				//分别设置占空比
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_speed);
				*/
				
				//================MPU6050防跑偏（纠正）==================
				
				float yaw_error = target_yaw - current_angle;//算偏角
        int correction = 0;
				
				//死区控制
				if (yaw_error > STRAIGHT_DEADBAND) {
            correction = STRAIGHT_CORRECT;
        } else if (yaw_error < -STRAIGHT_DEADBAND) {
            correction = -STRAIGHT_CORRECT;
        }

        // 把基础速度加上纠偏量
        int left_speed = current_speed - correction;
        int right_speed = current_speed + correction;
        
        // 限幅保护
        if (left_speed > 990) left_speed = 990;
        if (left_speed < 100) left_speed = 100;
        if (right_speed > 990) right_speed = 990;
        if (right_speed < 100) right_speed = 100;

        // 分别设置速度（占空比）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed); 
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_speed);
				
        // ========================================================

        // OLED显示
        refresh_count++;
        if (refresh_count >= 10) {
            OLED_ShowString(0, 4, "PULSE:");
            OLED_ShowNum(42, 4, current_pulse, 5);
            OLED_ShowString(0, 5, "SPD:");
            OLED_ShowNum(42, 5, current_speed, 4);
						OLED_ShowString(0, 6, "ANG:");
						OLED_ShowFloat(30, 6, current_angle);
            refresh_count = 0;
        }
        
        HAL_Delay(5);
    }
    
    Car_Stop();//停车指令
		
		//===============================航向对齐========================================
		
		uint32_t stop_time = HAL_GetTick(); // 记录刹车的时间戳
		
		while(HAL_GetTick() - stop_time < 800) {
			
		//————
		MPU6050_Read_All();//读取MPU6050数据
    float real_gz = (gz - gz_offset) / GYRO_SENSITIVITY;//真实角速度
		if (real_gz > -GYRO_DEADBAND && real_gz < GYRO_DEADBAND) real_gz = 0.0f;//死区过滤
		uint32_t current_time = HAL_GetTick();//读取当前时间戳
    float dt = (current_time - last_time) / 1000.0f;//真实时间（s）
		last_time = current_time; // 更新时间戳
		current_angle += real_gz * dt;//用真实时间进行积分
		//————
			
		float yaw_error = target_yaw - current_angle;//算偏角	
			
		//死区控制
		if (yaw_error > ALIGN_DEADBAND) {//偏右
           Car_TurnLeft(MIN_SPEED);
        } else if (yaw_error < -ALIGN_DEADBAND) {//偏左
           Car_TurnRight(MIN_SPEED);
        }else{
					 Car_Stop();
				}
			
		HAL_Delay(5);
		}
		
		OLED_ShowString(0, 6, "ANG:");
    OLED_ShowFloat(30, 6, current_angle);
		
		//===============================================================================
		
		int16_t left_pulse = __HAL_TIM_GET_COUNTER(&htim3);//读取左轮脉冲
		int16_t right_pulse = __HAL_TIM_GET_COUNTER(&htim4);//读取右轮脉冲   
    if (left_pulse < 0) left_pulse = -left_pulse; //绝对值
		if (right_pulse < 0) right_pulse = -right_pulse;//绝对值
		current_pulse = (left_pulse + right_pulse) / 2;//取平均值
		OLED_ShowString(0, 4, "PULSE:");
    OLED_ShowNum(42, 4, current_pulse, 5);
		
}

//转弯
void Turn_Left_90(void) {
	
    float start_angle = current_angle; //定义初始角度
    float target_angle = start_angle + TURN_TARGET_ANGLE;//定义目标角度
		float current_speed = TURN_MIN_SPEED;//定义当前速度
    
    int refresh_count = 0;//OLED屏幕刷新率
		uint32_t last_time = HAL_GetTick();//读取初始时间戳

    while (1) {
			
        MPU6050_Read_All();//读取MPU6050数据
        float real_gz = (gz - gz_offset) / GYRO_SENSITIVITY;//真实角速度
				if (real_gz > -GYRO_DEADBAND && real_gz < GYRO_DEADBAND) real_gz = 0.0f;//死区过滤
			
        /*current_angle += real_gz * ANGLE_INTEGRAL_K; //微元法积分（时间系数）*/
			
				//================进阶微元法积分=================
				uint32_t current_time = HAL_GetTick();//读取当前时间戳
        float dt = (current_time - last_time) / 1000.0f;//真实时间（s）
        last_time = current_time; // 更新时间戳
        
        current_angle += real_gz * dt; //用真实时间进行积分
			
			  //===============================================
			
				//退出循环条件
				if(current_angle >= target_angle){
					break;
				}
        
        // =================梯形加减速====================
			
        float passed_angle = current_angle - start_angle;//已走角度
        float remaining_angle = target_angle - current_angle;//剩余角度
        
				//加速
        if (passed_angle < TURN_ACCEL_ANGLE) {
            current_speed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * (passed_angle / TURN_ACCEL_ANGLE);
        } 
				//减速
        else if (remaining_angle < TURN_DECEL_ANGLE) {
            current_speed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * (remaining_angle / TURN_DECEL_ANGLE);
        } 
				//匀速
        else {
            current_speed = TURN_MAX_SPEED;
        }
        
        //设置速度
        Car_TurnLeft((uint16_t)current_speed);
				
        // =================================================
        
        // OLED显示
        refresh_count++;
        if (refresh_count >= 10) {
						OLED_ShowString(0, 5, "SPD:");
            OLED_ShowNum(42, 5, current_speed, 4);
            OLED_ShowString(0, 6, "ANG:");
            OLED_ShowFloat(30, 6, current_angle);
            refresh_count = 0;
        }
        
        HAL_Delay(5); 
    }
		
    Car_Stop();//停车指令 
		
		//===============================航向对齐========================================
		uint32_t stop_time = HAL_GetTick(); // 记录刹车的时间戳
		
		while(HAL_GetTick() - stop_time < 800) {

		//————
		MPU6050_Read_All();//读取MPU6050数据
    float real_gz = (gz - gz_offset) / GYRO_SENSITIVITY;//真实角速度
		if (real_gz > -GYRO_DEADBAND && real_gz < GYRO_DEADBAND) real_gz = 0.0f;//死区过滤
		uint32_t current_time = HAL_GetTick();//读取当前时间戳
    float dt = (current_time - last_time) / 1000.0f;//真实时间（s）
		last_time = current_time; // 更新时间戳
		current_angle += real_gz * dt;//用真实时间进行积分
		//————
			
		float yaw_error = start_angle + 90.0f - current_angle;//算偏角	
		//死区控制
		if (yaw_error > ALIGN_DEADBAND) {//偏右
           Car_TurnLeft(MIN_SPEED);
        } else if (yaw_error < -ALIGN_DEADBAND) {//偏左
           Car_TurnRight(MIN_SPEED);
        }else{
					 Car_Stop();
				}
			
		HAL_Delay(5);
		}
		
		OLED_ShowString(0, 6, "ANG:");
    OLED_ShowFloat(30, 6, current_angle);
		
}

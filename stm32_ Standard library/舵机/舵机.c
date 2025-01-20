/*----------------------------------时钟的初始化---------------------------------*/
void TIM4_CH1_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCInitTypeStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//要开启复用功能的时钟才能重映射
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  ,ENABLE); 
	
	//TIM3部分重映射
	/*
	*查看数据手册，引脚的定时器通道是完全映射，还是部分映射
	*二者调用参数不相同
	*完全映射 ：GPIO_FullRemap_TIM4
	*部分映射 ：GPIO_PartialRemap_TIM4
	*/
	
	//设置该引脚为复用输出功能
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	//初始化TIM4
	TIM_TimeBaseStruct.TIM_Period = arr;//重装载值 
	TIM_TimeBaseStruct.TIM_Prescaler = psc;//预分频值 
	TIM_TimeBaseStruct.TIM_ClockDivision = 0; //时钟分频1、2、4分频	
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;//设置计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStruct);
	
	//初始化输出比较参数
	TIM_OCInitTypeStruct.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式
	TIM_OCInitTypeStruct.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitTypeStruct.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性
	TIM_OC1Init(TIM4,&TIM_OCInitTypeStruct); //选择定时器输出通道 TIM4_CH1
		TIM_OC2Init(TIM4,&TIM_OCInitTypeStruct); //选择定时器输出通道 TIM4_CH1
	//使能预装载寄存器
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
		TIM_OC2Init(TIM4,&TIM_OCInitTypeStruct); //选择定时器输出通道 TIM4_CH1
	
	//使能定时器
	TIM_Cmd(TIM4,ENABLE);

}



/*-------------------------------------主函数----------------------------------------*/
//初始化  
	TIM4_CH1_PWM_Init(1999,719);  
  //ccr的值  1750-2000，这个范围内转满180
//这里可以直接debug进去调试，两个通达控制两个舵机，PB6，PB7
		while(1){
switch(Num){
//		//-90度
	case 1:	TIM_SetCompare1(TIM4,1750);//占空比（2000-1750）/2000*20ms=2.5ms
break;
//		//45度
	case 2:		TIM_SetCompare1(TIM4,1800);//占空比（2000-1800）/2000*20ms=2ms
break;
//		//0度
	case 3:	TIM_SetCompare1(TIM4,1850);//占空比（2000-1850）/2000*20ms=1.5ms
break;
	//-45度
case 4:	TIM_SetCompare1(TIM4,1900);//占空比（2000-1900）/2000*20ms=1ms
//Delay_ms(1000);
break;
//		//-90度
	case 5:		TIM_SetCompare1(TIM4,1945);//占空比（2000-1945）/2000*20ms=0.5ms  	
//		Delay_ms(1000);
	break;
		case 6:		TIM_SetCompare1(TIM4,vel);//占空比（2000-1945）/2000*20ms=0.5ms  	
//		Delay_ms(1000);
		break;
			case 7:		TIM_SetCompare2(TIM4,vel);//占空比（2000-1945）/2000*20ms=0.5ms  	
//		Delay_ms(1000);
		break;
		default :
			break;
	
	}

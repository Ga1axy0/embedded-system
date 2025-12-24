#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "stm32f10x_spi.h"
#include "stm32f10x_exti.h"

 
//LCD的画笔颜色和背景色	   
u16 POINT_COLOR=0x0000;	//画笔颜色
u16 BACK_COLOR=0xeeee;  //背景色 

//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;
	 		

void delay_us(int us)
{
		int i ;
		while(us > 0 )
		{
			for ( i = us; i < 100;i ++ ); 
			us -- ;
		}
}

void delay_ms(int ms)
{
	int i,j ;
		while(ms > 0 )
		{
			
			delay_us(1000);
			ms -- ;
		}
}


void LCD_Configuration(void)
{
	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	/* Enable the GPIO_LED Clock */
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO , ENABLE); 	
////	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); //Disable jtag	,Enable SWD
//			
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_7 |GPIO_Pin_11|GPIO_Pin_12 ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);

//	GPIO_SetBits(GPIOD, GPIO_Pin_12);  //reset pin
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
	
	/*****************寄存器版**********************/
	RCC->APB2ENR|=7<<4;      //使能PORTC,D,E时钟	
	GPIOD->CRL&=0X0F00FFFF;
	GPIOD->CRL|=0X30330000;
	GPIOD->CRH&=0XFFF00FFF;
	GPIOD->CRH|=0X00033000;  //PD4 5 7 11 12推挽输出  
	GPIOD->ODR|=0X1000;      //PD12置高
	
	GPIOC->CRL&=0XFFFFFFF0; 
	GPIOC->CRL|=0X00000003;      	 //PC0推挽输出 
}

void LCD_Config_DIN(void)
{	
	/*GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = 0xC703;////GPIO_Pin_14 | GPIO_Pin_15 |GPIO_Pin_1|GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,0xC703);
	GPIO_InitStructure.GPIO_Pin = 0xff80 ; //PE7-15
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	GPIO_ResetBits(GPIOE,0xff80);*/
	
	/*****************寄存器版**********************/
	//GPIOD->CRL&=0XFFFFFF00;
	//GPIOD->CRL|=0X00000088;
	//GPIOD->CRH&=0X00FFF000;
	//GPIOD->CRH|=0X88000888; 
	
	GPIOD->CRL=0x34334488;
	GPIOD->CRH=0x88433888;
	GPIOD->BRR=(uint16_t)0xC703;
	
	//GPIOE->CRL&=0X0FFFFFFF; 
	//GPIOE->CRL|=0X80000000;
	//GPIOE->CRH&=0XFFFFFFFF; 
	//GPIOE->CRH|=0X88888888;
	GPIOE->CRL=0X84444444;
	GPIOE->CRH=0X88888888;
	GPIOE->BRR=(uint16_t)0xff80;
}	

void LCD_Config_DOUT(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin = 0xC703;////GPIO_Pin_14 | GPIO_Pin_15 |GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
	GPIO_InitStructure.GPIO_Pin = 0xff80 ; //PE7-15
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
//	
	/*GPIOD->CRL&=0XFFFFFF00;
	GPIOD->CRL|=0X00000033;
	GPIOD->CRH&=0X00FFF000;
	GPIOD->CRH|=0X33000333; 
	
	GPIOE->CRL&=0XFFFFFFF0; 
	GPIOE->CRL|=0X00000003;
	GPIOE->CRH&=0XFFFFFFFF;
	GPIOE->CRH|=0X33333333;*/
	
	//GPIOD->CRL=0x34334433;
	//GPIOD->CRH=0x33333333;
	//GPIOE->CRL=0X34444444;
	//GPIOE->CRH=0X33333333;
	
	
}	

//
void DATAOUT(u16 x)//数据输出
{
	LCD_Config_DOUT();
	
	GPIOD->BRR = (0x03 << 14);  // LCD_D0,D1; PD14,PD15
	GPIOD->BSRR= ( x & 0x0003) << 14 ;
	
	GPIOD->BRR = (0x03 << 0);  // LCD_D2,D3; PD0,PD1
	GPIOD->BSRR= ( x & 0x000c) >> 2 ;
	
	GPIOE->BRR = (0x1ff << 7);  // LCD_D4-12; PE7-15
	GPIOE->BSRR= ( x & 0x1ff0) << 3 ;

	GPIOD->BRR = (0x07 << 8);  // LCD_D13,D14,D15; PD8-10
	GPIOD->BSRR= ( x & 0xE000) >> 5 ;
	
	
}	


u16 DATAIN()   //数据输入	
{
		u16 tmp = 0 ;
		LCD_Config_DIN();
		tmp = GPIOD->IDR >> 14; 
		tmp |= ( (GPIOD->IDR & 0x3) << 2);
		tmp |= ((GPIOE->IDR & 0xff80) >> 3);
		tmp |= ((GPIOD->IDR & 0x0700) << 5);
		return tmp;
}



//写寄存器函数
//data:寄存器值
void LCD_WR_REG(u16 data)
{ 
	LCD_RS_CLR;//写地址  
 	LCD_CS_CLR; 
	DATAOUT(data); 
	LCD_WR_CLR; 
	LCD_WR_SET; 
 	LCD_CS_SET;   
}
//写数据函数
//可以替代LCD_WR_DATAX宏,拿时间换空间.
//data:寄存器值
void LCD_WR_DATAX(u16 data)
{
	LCD_RS_SET;
	LCD_CS_CLR;
	DATAOUT(data);
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
}
//读LCD数据
//返回值:读到的值
u16 LCD_RD_DATA(void)
{										   
	u16 t;

	DATAIN();

	LCD_RS_SET;
	LCD_CS_CLR;
	//读取数据(读寄存器时,并不需要读2次)
	LCD_RD_CLR;
 
	t=DATAIN();  
	LCD_RD_SET;
	LCD_CS_SET; 


	DATAOUT(0xFFFF);
	return t;  
}
//写寄存器
//LCD_Reg:寄存器编号
//LCD_RegValue:要写入的值
void LCD_WriteReg(u16 LCD_Reg,u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}   
//读寄存器
//LCD_Reg:寄存器编号
//返回值:读到的值
u16 LCD_ReadReg(u16 LCD_Reg)
{										   
 	LCD_WR_REG(LCD_Reg);  //写入要读的寄存器号  
	return LCD_RD_DATA(); 
} 
//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
} 
//LCD写GRAM
//RGB_Code:颜色值
void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD_WR_DATA(RGB_Code);//写十六位GRAM
}

//当mdk -O1时间优化时需要设置
//延时i
void opt_delay(u8 i)
{
	while(i--);
}
//读取个某点的颜色值	 
//x,y:坐标
//返回值:此点的颜色
u16 LCD_ReadPoint(u16 x,u16 y)
{
 	u16 r,g,b;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回		   
	LCD_SetCursor(x,y);
	if(lcddev.id==0X9341)LCD_WR_REG(0X2E);//9341/6804/3510/1963 发送读GRAM指令
	DATAIN();//
	
	LCD_RS_SET;
	LCD_CS_CLR;	    
	//读取数据(读GRAM时,第一次为假读)	
	LCD_RD_CLR;		   
	opt_delay(2);									//延时		
 	r=DATAIN();  										//实际坐标颜色 
	LCD_RD_SET;

 	//dummy READ
	LCD_RD_CLR;					   
	opt_delay(2);//延时					   
 	r=DATAIN();  	//实际坐标颜色
	LCD_RD_SET;
 	if(lcddev.id==0X9341)	//9341/NT35310/NT35510要分2次读出
	{	 
		LCD_RD_CLR;					   
		opt_delay(2);//延时			   
		b=DATAIN();//读取蓝色值  	  
	 	LCD_RD_SET;
		g=r&0XFF;//对于9341,第一次读取的是RG的值,R在前,G在后,各占8位
		g<<=8;
	}	 
	LCD_CS_SET;
	DATAOUT(0xFFFF);  //全部输出高
	
	if(lcddev.id==0X9341) 
		return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));//ILI9341/NT35310/NT35510需要公式转换一下

}		 
//LCD开启显示
void LCD_DisplayOn(void)
{					   
	if(lcddev.id==0X9341)LCD_WR_REG(0X29);	//开启显示

}	 
//LCD关闭显示
void LCD_DisplayOff(void)
{	   
	if(lcddev.id==0X9341)LCD_WR_REG(0X28);	//关闭显示

}   
//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	 
 	if(lcddev.id==0X9341)
	{		    
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(Xpos>>8);LCD_WR_DATA(Xpos&0XFF); 			 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8);LCD_WR_DATA(Ypos&0XFF); 		
	} 
} 		 
//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
//所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向(具体定义见lcd.h)
//9320/9325/9328/4531/4535/1505/b505/5408/9341/5310/5510/1963等IC已经实际测试	   	   
void LCD_Scan_Dir(u8 dir)
{
	u16 regval=0;
	u16 dirreg=0;
	u16 temp;  

	if(lcddev.id==0x9341)//9341/6804/5310/5510/1963,特殊处理
	{
		switch(dir)
		{
			case L2R_U2D://从左到右,从上到下
				regval|=(0<<7)|(0<<6)|(0<<5); 
				break;
			case L2R_D2U://从左到右,从下到上
				regval|=(1<<7)|(0<<6)|(0<<5); 
				break;
			case R2L_U2D://从右到左,从上到下
				regval|=(0<<7)|(1<<6)|(0<<5); 
				break;
			case R2L_D2U://从右到左,从下到上
				regval|=(1<<7)|(1<<6)|(0<<5); 
				break;	 
			case U2D_L2R://从上到下,从左到右
				regval|=(0<<7)|(0<<6)|(1<<5); 
				break;
			case U2D_R2L://从上到下,从右到左
				regval|=(0<<7)|(1<<6)|(1<<5); 
				break;
			case D2U_L2R://从下到上,从左到右
				regval|=(1<<7)|(0<<6)|(1<<5); 
				break;
			case D2U_R2L://从下到上,从右到左
				regval|=(1<<7)|(1<<6)|(1<<5); 
				break;	 
		}
		dirreg=0X36;
 		LCD_WriteReg(dirreg,regval);

  }
}     
//画点
//x,y:坐标
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);		//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	LCD_WR_DATA(POINT_COLOR); 
}	 
//快速画点
//x,y:坐标
//color:颜色
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)  //不使用设置位置、写GRAM函数，多一个颜色参数，其它跟LCD_DrawPoint时一样的
{	   
	if(lcddev.id==0X9341)
	{
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(x>>8);LCD_WR_DATA(x&0XFF);  			 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(y>>8);LCD_WR_DATA(y&0XFF); 		 	 
	}
	LCD_RS_CLR;
 	LCD_CS_CLR; 
	DATAOUT(lcddev.wramcmd);//写指令  
	LCD_WR_CLR; 
	LCD_WR_SET; 
 	LCD_CS_SET; 
	LCD_WR_DATA(color);		//写数据
}
//SSD1963 背光设置
//pwm:背光等级,0~100.越大越亮.
void LCD_SSD_BackLightSet(u8 pwm)
{	
	LCD_WR_REG(0xBE);	//配置PWM输出
	LCD_WR_DATA(0x05);	//1设置PWM频率
	LCD_WR_DATA(pwm*2.55);//2设置PWM占空比
	LCD_WR_DATA(0x01);	//3设置C
	LCD_WR_DATA(0xFF);	//4设置D
	LCD_WR_DATA(0x00);	//5设置E
	LCD_WR_DATA(0x00);	//6设置F
}
//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(u8 dir)
{
	if(dir==0)			//竖屏
	{
		lcddev.dir=0;	//竖屏
		lcddev.width=240;
		lcddev.height=320;
		if(lcddev.id==0X9341)
		{
			lcddev.wramcmd=0X2C;
	 		lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B;  	 
		}
	}else 				//横屏
	{	  				
		lcddev.dir=1;	//横屏
		lcddev.width=320;
		lcddev.height=240;
		if(lcddev.id==0X9341)
		{
			lcddev.wramcmd=0X2C;
	 		lcddev.setxcmd=0X2A;
			lcddev.setycmd=0X2B;  	 
		}
	} 
	LCD_Scan_Dir(DFT_SCAN_DIR);	//默认扫描方向
}	 
//设置窗口,并自动设置画点坐标到窗口左上角(sx,sy).
//sx,sy:窗口起始坐标(左上角)
//width,height:窗口宽度和高度,必须大于0!!
//窗体大小:width*height. 
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{    
	u8 hsareg,heareg,vsareg,veareg;
	u16 hsaval,heaval,vsaval,veaval; 
	u16 twidth,theight;
	twidth=sx+width-1;
	theight=sy+height-1;
	if(lcddev.id==0X9341)
	{
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(sx>>8); 
		LCD_WR_DATA(sx&0XFF);	 
		LCD_WR_DATA(twidth>>8); 
		LCD_WR_DATA(twidth&0XFF);  
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(sy>>8); 
		LCD_WR_DATA(sy&0XFF); 
		LCD_WR_DATA(theight>>8); 
		LCD_WR_DATA(theight&0XFF); 
	}
}
//初始化lcd
//该初始化函数可以初始化各种ALIENTEK出品的LCD液晶屏
//本函数占用较大flash,用户可以根据自己的实际情况,删掉未用到的LCD初始化代码.以节省空间.
void LCD_Init(void)
{ 
	delay_ms(5); // delay 50 ms 
	LCD_WriteReg(0x0000,0x0001);
	delay_ms(5); // delay 50 ms 
  lcddev.id = LCD_ReadReg(0x0000);

 		//尝试9341 ID的读取		
		LCD_WR_REG(0XD3);				   
		LCD_RD_DATA(); 				//dummy read 	
 		LCD_RD_DATA();   	    	//读到0X00
  	lcddev.id=LCD_RD_DATA();   	//读取93								   
 		lcddev.id<<=8;
		lcddev.id|=LCD_RD_DATA();  	//读取41 	   			   

		if(lcddev.id==0X9341)	//9341初始化
		{	 
			LCD_WR_REG(0xCF);  
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0xC1); 
			LCD_WR_DATAX(0X30); 
			LCD_WR_REG(0xED);  
			LCD_WR_DATAX(0x64); 
			LCD_WR_DATAX(0x03); 
			LCD_WR_DATAX(0X12); 
			LCD_WR_DATAX(0X81); 
			LCD_WR_REG(0xE8);  
			LCD_WR_DATAX(0x85); 
			LCD_WR_DATAX(0x10); 
			LCD_WR_DATAX(0x7A); 
			LCD_WR_REG(0xCB);  
			LCD_WR_DATAX(0x39); 
			LCD_WR_DATAX(0x2C); 
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x34); 
			LCD_WR_DATAX(0x02); 
			LCD_WR_REG(0xF7);  
			LCD_WR_DATAX(0x20); 
			LCD_WR_REG(0xEA);  
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x00); 
			LCD_WR_REG(0xC0);    //Power control 
			LCD_WR_DATAX(0x1B);   //VRH[5:0] 
			LCD_WR_REG(0xC1);    //Power control 
			LCD_WR_DATAX(0x01);   //SAP[2:0];BT[3:0] 
			LCD_WR_REG(0xC5);    //VCM control 
			LCD_WR_DATAX(0x30); 	 //3F
			LCD_WR_DATAX(0x30); 	 //3C
			LCD_WR_REG(0xC7);    //VCM control2 
			LCD_WR_DATAX(0XB7); 
			LCD_WR_REG(0x36);    // Memory Access Control 
			LCD_WR_DATAX(0x48); 
			LCD_WR_REG(0x3A);   
			LCD_WR_DATAX(0x55); 
			LCD_WR_REG(0xB1);   
			LCD_WR_DATAX(0x00);   
			LCD_WR_DATAX(0x1A); 
			LCD_WR_REG(0xB6);    // Display Function Control 
			LCD_WR_DATAX(0x0A); 
			LCD_WR_DATAX(0xA2); 
			LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
			LCD_WR_DATAX(0x00); 
			LCD_WR_REG(0x26);    //Gamma curve selected 
			LCD_WR_DATAX(0x01); 
			LCD_WR_REG(0xE0);    //Set Gamma 
			LCD_WR_DATAX(0x0F); 
			LCD_WR_DATAX(0x2A); 
			LCD_WR_DATAX(0x28); 
			LCD_WR_DATAX(0x08); 
			LCD_WR_DATAX(0x0E); 
			LCD_WR_DATAX(0x08); 
			LCD_WR_DATAX(0x54); 
			LCD_WR_DATAX(0XA9); 
			LCD_WR_DATAX(0x43); 
			LCD_WR_DATAX(0x0A); 
			LCD_WR_DATAX(0x0F); 
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x00); 		 
			LCD_WR_REG(0XE1);    //Set Gamma 
			LCD_WR_DATAX(0x00); 
			LCD_WR_DATAX(0x15); 
			LCD_WR_DATAX(0x17); 
			LCD_WR_DATAX(0x07); 
			LCD_WR_DATAX(0x11); 
			LCD_WR_DATAX(0x06); 
			LCD_WR_DATAX(0x2B); 
			LCD_WR_DATAX(0x56); 
			LCD_WR_DATAX(0x3C); 
			LCD_WR_DATAX(0x05); 
			LCD_WR_DATAX(0x10); 
			LCD_WR_DATAX(0x0F); 
			LCD_WR_DATAX(0x3F); 
			LCD_WR_DATAX(0x3F); 
			LCD_WR_DATAX(0x0F); 
			LCD_WR_REG(0x2B); 
			LCD_WR_DATAX(0x00);
			LCD_WR_DATAX(0x00);
			LCD_WR_DATAX(0x01);
			LCD_WR_DATAX(0x3f);
			LCD_WR_REG(0x2A); 
			LCD_WR_DATAX(0x00);
			LCD_WR_DATAX(0x00);
			LCD_WR_DATAX(0x00);
			LCD_WR_DATAX(0xef);	 
			LCD_WR_REG(0x11); //Exit Sleep
			delay_ms(10);
			LCD_WR_REG(0x29); //display on	
		}

	LCD_Display_Dir(0);		 	//默认为竖屏
	LCD_LED=1;					//点亮背光
	LCD_Clear(WHITE);
}  		  
  
//清屏函数
//color:要清屏的填充色
void LCD_Clear(u16 color)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//得到总点数
  LCD_SetCursor(0x00,0x0000);	//设置光标位置 
	LCD_WriteRAM_Prepare();     		//开始写入GRAM	  	  
	for(index=0;index<totalpoint;index++)LCD_WR_DATA(color);	
}  
//在指定区域内填充指定颜色
//区域大小:(xend-xsta+1)*(yend-ysta+1)
//xsta
//color:要填充的颜色
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{          
	u16 i,j;
	u16 xlen=0;
	u16 temp;
	{
		xlen=ex-sx+1;	 
		for(i=sy;i<=ey;i++)
		{
		 	LCD_SetCursor(sx,i);      				//设置光标位置 
			LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
			for(j=0;j<xlen;j++)LCD_WR_DATA(color);	//设置光标位置 	    
		}
	}
}  
//在指定区域内填充指定颜色块			 
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{  
	u16 height,width;
	u16 i,j;
	width=ex-sx+1; 			//得到填充的宽度
	height=ey-sy+1;			//高度
 	for(i=0;i<height;i++)
	{
 		LCD_SetCursor(sx,sy+i);   	//设置光标位置 
		LCD_WriteRAM_Prepare();     //开始写入GRAM
		for(j=0;j<width;j++)LCD_WR_DATA(color[i*width+j]);//写入数据 
	}	  
} 
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    
//画矩形	  
//(x1,y1),(x2,y2):矩形的对角坐标
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  		LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  
//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16/24
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数	
 	num=num-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[num][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[num][t];	//调用2412字体
		else return;								//没有的字库
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//超区域了
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//超区域了
				break;
			}
		}  	 
	}  	    	   	 	  
}   
//m^n函数
//返回值:m^n次方.
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
//显示数字,高位为0,则不显示
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//color:颜色 
//num:数值(0~4294967295);	 
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 
//显示数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);	 
//len:长度(即要显示的位数)
//size:字体大小
//mode:
//[7]:0,不填充;1,填充0.
//[6:1]:保留
//[0]:0,非叠加显示;1,叠加显示.
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{  
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);  
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);  
 				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01); 
	}
} 
//显示字符串
//x,y:起点坐标
//width,height:区域大小  
//size:字体大小
//*p:字符串起始地址		  
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}

const unsigned char CN_DataBuf[][16] = {
/*--  大  --*/
/*--  ??12;  ??????????:?x?=16x16   --*/
0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,
0x02,0x80,0x02,0x80,0x04,0x40,0x04,0x40,0x08,0x20,0x10,0x10,0x20,0x08,0xC0,0x06,

/*--  小  --*/
/*--  ??12;  ??????????:?x?=16x16   --*/
0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x11,0x10,0x11,0x08,0x11,0x04,
0x21,0x04,0x21,0x02,0x41,0x02,0x81,0x02,0x01,0x00,0x01,0x00,0x05,0x00,0x02,0x00
};



void  TOUCH_SCREEN_INIT() { 
	
	   GPIO_InitTypeDef GPIO_InitStructure;
	   SPI_InitTypeDef  SPI_InitStructure;
  /* Enable GPIOB, GPIOC and AFIO clock */
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO , ENABLE); 	
		 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); //Disable jtag	,Enable SWD

  /* SPI pins configuration */
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15| GPIO_Pin_5;
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
	
     GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
     GPIO_Init(GPIOB, &GPIO_InitStructure);// MOSI
	
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
     GPIO_Init(GPIOA, &GPIO_InitStructure);// MISO   


/*
		 SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI??????????
     SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//????SPI
     SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//??SPI?????:SPI????8????
     SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//??????????:?????
     SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//???????????
     SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//(SPI_NSS_Soft)??NSS?????????GPIO??????
     SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//Fclk/2
     SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Initialize the SPI_FirstBit member 
     SPI_InitStructure.SPI_CRCPolynomial=7;
     SPI_Init(SPI1, &SPI_InitStructure);
     SPI_Cmd(SPI1, ENABLE);
*/
}

#define  TOUCH_READ_TIMES  4 //点数个数
u16 x_addata[TOUCH_READ_TIMES],z_addata[TOUCH_READ_TIMES],y_addata[TOUCH_READ_TIMES];
u32 Temp ,r,sx,sy= 0;
int xScreenAD, yScreenAD;
int xScreen, yScreen;
u16 PointX_ADmin=0;
u16 PointY_ADmin=0;
u16 PointX_ADmax=0;
u16 PointY_ADmax=0;

#define QPINTERVAL 48
#define QZSIZE 20
#define QPSIZEX 240  //分辨率
#define QPSIZEY 320
#define QPSTARTX 0
#define QPSTARTY 0




#define	T_CS_SET  GPIOA->BSRR=1<<15    //片选端口  		PA15
#define	T_WR_SET	GPIOB->BSRR=1<<5    //写数据MOSI			PB5
#define	T_SCK_SET	GPIOA->BSRR=1<<5    //SCK			PA5
								    
#define	T_CS_CLR  GPIOA->BRR=1<<15     //片选端口  		PA15
#define	T_WR_CLR	GPIOB->BRR=1<<5     //写数据	MOSI		PB5
#define	T_SCK_CLR	GPIOA->BRR=1<<5     //SCK			PA5  

#define	T_IN_STATUE	((GPIOA->IDR & 0X40)?1:0)    //数据MISO PA6	   
 
//spi读写数据
u8 SPI_SndRecv(u8 data)
{
	u8 tmp=0,i;

	delay_us(1);
	for( i = 0 ; i < 8 ; i ++ )
	{
		T_SCK_CLR;

		if(data & 0x80)
			T_WR_SET;
		else
			T_WR_CLR;
		data = data << 1;
		delay_us(1);	
		
		T_SCK_SET;
		tmp = tmp << 1;
		tmp |= T_IN_STATUE;		
		delay_us(1);

	}

	return tmp ;
	
}


u32 SPI_X(void) { 
		 u16		i,j,k;
	   for(i=0;i<TOUCH_READ_TIMES;i++)					//采样4次.
	     {
					T_CS_CLR;		//cs
					SPI_SndRecv(0xD0);
 
					x_addata[i] = SPI_SndRecv(0x00);
				  x_addata[i]<<=8;	//拼接2个字节的数据(x坐标)
				 
					x_addata[i] |= SPI_SndRecv(0x00);
				  x_addata[i]>>=3; 
				 
					T_CS_SET;

			}  
 
	 Temp=(x_addata[2] + x_addata[3]) >> 1;
	 return Temp ; 
			 

}

u32 SPI_Y(void) { 
	u16 i,j,k;
	for(i=0;i<TOUCH_READ_TIMES;i++) {					//采样4次. 
			T_CS_CLR;		//cs
			SPI_SndRecv(0x90);
			y_addata[i] = SPI_SndRecv(0x00);
			y_addata[i]<<=8;	
			y_addata[i] |= SPI_SndRecv(0x00);
			y_addata[i]>>=3;
			T_CS_SET;
	}

	Temp=(y_addata[2] + y_addata[3]) >> 1;
	return Temp;

}  
 
static void TOUCH_INT_config(void) {
     GPIO_InitTypeDef GPIO_InitStructure;
     /* Enable GPIOB, GPIOC and AFIO clock */
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);  
     /* LEDs pins configuration */
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
     //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void TOUCH_INT_EXIT_Init(void) {
     EXTI_InitTypeDef EXTI_InitStructure;
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //AFIO??
    /* Connect Button EXTI Line to Button GPIO Pin */
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2);
    /* Configure Button EXTI line */
     EXTI_InitStructure.EXTI_Line = EXTI_Line2;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	   EXTI_ClearITPendingBit(EXTI_Line2);	
     EXTI_Init(&EXTI_InitStructure);

	

	}
static void TOUCH_InterruptConfig(void) { 
    NVIC_InitTypeDef NVIC_InitStructure;
  /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
 /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}

//根据采集到的坐标值找寻最近的网格点上
static void getPos(u16 xScreen, u16 yScreen) {
	u16 x = xScreen, y = yScreen;
	int i, j;
	int dist = 0;
	for(i = QPSTARTX+QPINTERVAL/2, dist = 9999; i <= QPSTARTX + QPSIZEX; i += QPINTERVAL) {
		if(abs(i-x) <= dist) {
			xScreen = i;
			dist = abs(i-x);
		} 
	}
	for(j = QPSTARTY+QPINTERVAL/2, dist = 9999; j <= QPSTARTY + QPSIZEY; j += QPINTERVAL) {
		if(abs(j-y) <= dist) {
			yScreen = j;
			dist = abs(j-y);
		}
	}
//		play(xScreen/QPINTERVAL, yScreen/QPINTERVAL);
}
int PressFlag = 0;



void TouchScreen() {
//	static u16 sDataX,sDataY;
	delay_ms(1);
	xScreenAD = SPI_X();
  yScreenAD = SPI_Y();
	 if (xScreenAD < 50 ||  yScreenAD < 50 || xScreenAD > 4050 || yScreenAD > 4050 )
	 {} //无效数据
	 else
	 {
			PressFlag ++ ;
		  if ( PointX_ADmax > PointX_ADmin && PointY_ADmax > PointY_ADmin ) //已经经过校准了
			{
					xScreen = (float)(xScreenAD-PointX_ADmin) * 160/(PointX_ADmax-PointX_ADmin) + 40 ;
				  yScreen = (float)(yScreenAD-PointY_ADmin) * 240/(PointY_ADmax-PointY_ADmin) + 40 ;
			}
	 }
//	if((xScreen>1)&&(yScreen>1)&&(xScreen<240-1)&&(yScreen<320-1)) {
//		if(!(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)) ) {
//			if((xScreen < QPSIZEX/QPINTERVAL*QPINTERVAL-1)&&(yScreen <= QPSIZEY/QPINTERVAL*QPINTERVAL-1)) getPos(xScreen,yScreen);
//		}

//  }
}

void EXTI2_IRQHandler (void) {
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
			
     TouchScreen();
     EXTI_ClearITPendingBit(EXTI_Line2);
  }
}


void Touch_Init()
{
	TOUCH_SCREEN_INIT();
	TOUCH_INT_config();
	TOUCH_INT_EXIT_Init();
	TOUCH_InterruptConfig();	
	

//	while(1);
}


//显示采集到的点位数据
char showstr[32];


void Touch_Check()
{
	int status = 0 ;
	LCD_ShowString(45,45,245,325,12,"1");

	LCD_ShowString(205,285,245,325,12,"2");

	
	LCD_DrawLine(0,40,240,40);
	LCD_DrawLine(0,280,240,280);  //交叉点在x=(40 , 200),y=(40,280)
	LCD_ShowString(40-2,40-5,240,12,12,"+");
	LCD_DrawLine(40,0,40,320);
	LCD_DrawLine(200,0,200,320);
	
	while(status == 0)
	{
			LCD_ShowString(45,0,240,12,12,"Press No.1 Point");
			if ( PressFlag > 0)  //已经触摸过了
			{
					PointX_ADmin = xScreenAD;
				  PointY_ADmin = yScreenAD ;
					sprintf(showstr,"TouchAD, x:%4d y%4d   ",xScreenAD,yScreenAD);
					LCD_ShowString(45,12,240,12,12,showstr);
					status = 1;
					PressFlag = 0 ;
			}
	}
	while(status == 1)
	{
			LCD_ShowString(45,0,240,12,12,"Press No.2 Point");
			if ( PressFlag > 0)  //已经触摸过了
			{
					PointX_ADmax = xScreenAD;
				  PointY_ADmax = yScreenAD ;
					sprintf(showstr,"TouchAD, x:%4d y%4d   ",xScreenAD,yScreenAD);
					LCD_ShowString(45,24,240,12,12,showstr);
					status = 2;
					PressFlag = 0 ;
					status = 3 ;
			}			
	}
	
}


void showTouch()
{

		sprintf(showstr,"Press at:%4d y%4d   ",xScreen,yScreen);
	  LCD_ShowString(45,0,240,12,12,showstr); 
		LCD_ShowString(xScreen-2,yScreen-5,240,12,12,"+"); //+中心点对坐标的偏移为2，5
}











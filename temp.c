#include <stdio.h>

#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
//返回 0：初始化成功
unsigned char Init_QT18B20(void)
{
	unsigned char dat;

	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
	MAP_UtilsDelay(367); 					//恢复时间：>1us。
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
	MAP_UtilsDelay(17120);  				//单片机拉低总线:>480us。642us
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
	MAP_UtilsDelay(1600);				//释放总线，在中间位置读取是否有器件响应。60us
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_IN);
	MAP_UtilsDelay(5600);  				//210us
	dat = MAP_GPIOPinRead(GPIOA1_BASE, 0x10); //读低电平
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
	return dat;
}








//读1字节温度数据
unsigned char ReadOneChar(void)
{
	unsigned char i;
	unsigned char date = 0;
	for(i=8;i>0;i--)
	{
		MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
		MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
		MAP_UtilsDelay(134);  //5us
		MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
		MAP_UtilsDelay(134);  //5us
		MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_IN);
		date >>= 1;
		if(MAP_GPIOPinRead(GPIOA1_BASE, 0x10))
		{
			date |= 0x80;
		}
		MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
		MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
		MAP_UtilsDelay(1200);  //45us
	}
	return date;
}

//写1字节温度数据
void WriteOneChar(unsigned char date)
{
	unsigned char i;
	unsigned char textb;
	for(i = 8;i > 0;i--)
	{
		textb = date&0x01;
		date = date >> 1;
		if(textb)
		{
			MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
			MAP_UtilsDelay(30);	//大于1us
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
			MAP_UtilsDelay(2400);	//至少60us,90us
		}
		else
		{
			MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
			MAP_UtilsDelay(2400);	//至少60us,90us
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
			MAP_UtilsDelay(30); //大于1us
		}
	}
}

float ReadTemp(void)
{
	unsigned int temp = 0;
	unsigned int templ;
	unsigned int temph;
	float i,j;
	Init_QT18B20();
	MAP_UtilsDelay(300); //大于10us
	WriteOneChar(0xcc);
	WriteOneChar(0x44);

	Init_QT18B20();
	WriteOneChar(0xcc);
	WriteOneChar(0xbe);
	templ = ReadOneChar();
	temph = ReadOneChar();
	temp = temph;
	temp <<= 8;
	temp |= templ;
	i = temp;
	j = 0.0625*i;
	return j;
}


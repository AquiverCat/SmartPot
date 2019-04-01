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
//���� 0����ʼ���ɹ�
unsigned char Init_QT18B20(void)
{
	unsigned char dat;

	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
	MAP_UtilsDelay(367); 					//�ָ�ʱ�䣺>1us��
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
	MAP_UtilsDelay(17120);  				//��Ƭ����������:>480us��642us
	MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
	MAP_UtilsDelay(1600);				//�ͷ����ߣ����м�λ�ö�ȡ�Ƿ���������Ӧ��60us
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_IN);
	MAP_UtilsDelay(5600);  				//210us
	dat = MAP_GPIOPinRead(GPIOA1_BASE, 0x10); //���͵�ƽ
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
	return dat;
}








//��1�ֽ��¶�����
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

//д1�ֽ��¶�����
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
			MAP_UtilsDelay(30);	//����1us
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
			MAP_UtilsDelay(2400);	//����60us,90us
		}
		else
		{
			MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0);
			MAP_UtilsDelay(2400);	//����60us,90us
			MAP_GPIOPinWrite(GPIOA1_BASE, 0x10,0x10);
			MAP_UtilsDelay(30); //����1us
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
	MAP_UtilsDelay(300); //����10us
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


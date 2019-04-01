#ifndef __TEMP_H_
#define __TEMP_H_

unsigned char Init_QT18B20(void);
unsigned char ReadOneChar(void);
void WriteOneChar(unsigned char dat);
float ReadTemp(void);
#endif

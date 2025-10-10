#ifndef FX_FXTYPE_H_
#define FX_FXTYPE_H_
/*********************************************************************************
// 应用时候应该根据平台和编译环境确认放开下列宏
// _FX_CPL_UBUNTU_XENOMAI_J1900_  J1900 控制器 ubuntu12.04       gcc4.8.4
// _FX_CPL_WIN_32_CASE_1_   intel CPU i7  Windows 11 VS2022 x86 V143 C++14
// _FX_CPL_WIN_64_CASE_1_   intel CPU i7  Windows 11 VS2022 x64 V143 C++14
//
//
//
//
// 新应用环境应该进行类型测试,用以下代码测试，并在以上添加新的宏
/////////////////////////////////////////////////////////////////////////
#include "stdio.h"
#include "stdlib.h"
int main()
{
	printf("int %d\n", sizeof(int));
	printf("short %d\n", sizeof(short));
	printf("long %d\n", sizeof(long));
	printf("long long %d\n", sizeof(long long));//可能需要修改


	printf("unsigned int %d\n", sizeof(unsigned int));
	printf("unsignedshort %d\n", sizeof(unsigned short));
	printf("unsignedlong %d\n", sizeof(unsigned long));
	printf("unsignedlong long %d\n", sizeof(unsigned long long));//可能需要修改

	printf("double %d\n", sizeof(double));
	printf("flot %d\n", sizeof(float));
	return 0;

}
/////////////////////////////////////////////////////////////////////////

*********************************************************************************/  
#define _FX_CPL_UBUNTU_XENOMAI_J1900_
//#define _FX_CPL_WIN_32_CASE_1_
//#define _FX_CPL_WIN_64_CASE_1_


#ifdef  _FX_CPL_UBUNTU_XENOMAI_J1900_
#define FX_VOID     void

#define FX_BOOL  	unsigned char
#define FX_TRUE  	1
#define FX_FALSE 	0

#define FX_CHAR   	char
#define FX_UCHAR  	unsigned char
#define FX_INT8   	char
#define FX_INT16 	short
#define FX_INT32 	int
#define FX_INT32L 	long
#define FX_INT64 	long long

#define FX_UINT8  	unsigned char
#define FX_UINT16 	unsigned short
#define FX_UINT32 	unsigned int
#define FX_UINT32L 	unsigned long
#define FX_UINT64 	unsigned long long

#define FX_FLOAT 	float
#define FX_DOUBLE 	double

#endif


#ifdef  _FX_CPL_WIN_32_CASE_1_
#define FX_VOID     void

#define FX_BOOL  	unsigned char
#define FX_TRUE  	1
#define FX_FALSE 	0

#define FX_CHAR   	char
#define FX_UCHAR  	unsigned char
#define FX_INT8   	char
#define FX_INT16 	short
#define FX_INT32 	int
#define FX_INT32L 	long
#define FX_INT64 	long long

#define FX_UINT8  	unsigned char
#define FX_UINT16 	unsigned short
#define FX_UINT32 	unsigned int
#define FX_UINT32L 	unsigned long
#define FX_UINT64 	unsigned long long

#define FX_FLOAT 	float
#define FX_DOUBLE 	double

#endif


#ifdef  _FX_CPL_WIN_64_CASE_1_
#define FX_VOID     void

#define FX_BOOL  	unsigned char
#define FX_TRUE  	1
#define FX_FALSE 	0

#define FX_CHAR   	char
#define FX_UCHAR  	unsigned char
#define FX_INT8   	char
#define FX_INT16 	short
#define FX_INT32 	int
#define FX_INT32L 	long
#define FX_INT64 	long long

#define FX_UINT8  	unsigned char
#define FX_UINT16 	unsigned short
#define FX_UINT32 	unsigned int
#define FX_UINT32L 	unsigned long
#define FX_UINT64 	unsigned long long

#define FX_FLOAT 	float
#define FX_DOUBLE 	double

#endif


#endif



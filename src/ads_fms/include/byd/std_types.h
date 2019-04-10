/*****************************************************************************
  Copyright (c) 2018, ���ǵ�����
  All rights reserved.

  �ļ�����: std_types.h
  ժ Ҫ:    �������� 

  ����:
  �����б�:

  ��ǰ�汾: 1.0
  �� ��:
  �޸�����:
  �������: 2018��05��17��
*****************************************************************************/
#ifndef _BYDSTD_TYPES_H_
#define _BYDSTD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

typedef signed   char   	 	int8_t;
typedef unsigned char    		uint8_t;

typedef signed   short   		int16_t;
typedef unsigned short  	 	uint16_t;

typedef signed   int     		int32_t;
typedef unsigned int    	 	uint32_t;

#ifndef int64_t
typedef signed   long int     	int64_t;
#endif

#ifndef uint64_t
typedef unsigned long int     	uint64_t;
#endif
/*
#ifndef bool
typedef uint32_t bool;
#endif*/
/**
 *  \ I-KEY��VIN��Ϣ���ݽṹ 
 */
typedef struct
{
	uint8_t vininfo[17];
}VinInfo;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
 

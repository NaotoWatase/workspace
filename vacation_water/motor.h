/*
    Header file for wro 2022 robomission junior program
    motor.h
    Written by Naoto Watase and Kaita Suzuki.
    Copyright by NK Cells 2022. All rights reserved.
    This source code is a property of NK Cells. Nobody can use, copy, modify, transmit, redistibute and 
    sell without permission of NK Cells. 
*/

/*
	アプリケーションの定数の定義
    例：
#define MSEC (1000)
*/

/*
    肩の宣言

例：
typedef enum object {  
    PERSON = 10,
    CHILD = 3,
    ADULT = 2,
    FIRE = 5,
    CHEMICAL = 1,
    NOTHING = 0
} object_t ;

*/

/*
	アプリケーションのグローバル変数の宣言
    注意：
    変数の実体を定義するファイルで、このヘッダーファイルをインクルードする前に
    #define _MOTOR_H_
    と書いておく。(MOTORはそのファイル名)
*/
#ifndef _MOTOR_C_ 
// グローバル変数をここでextern 宣言する。
// extern int test;

#endif

/*
 *  関数のプロトタイプ宣言
 */
extern void steering(int power, int cm, int steering);




//EOF
#if !defined(INCLUDE_APP_H)
#define INCLUDE_APP_H

#include <ev3api.h>

#ifndef STACK_SIZE
#define STACK_SIZE      4096        /* タスクのスタックサイズ */
#endif /* STACK_SIZE */

/*
 *  関数のプロトタイプ宣言
 */
void    main_task(intptr_t exinf);

/*
 * 変数の宣言
 */
/* なし */

#endif /* INCLUDE_APP_H */

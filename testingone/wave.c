
#define _WAVE_H_
/**
 * EV3RT wave playback sample.
 *
 *
 */

#define _MOTOR_C_

#include "ev3api.h"
#include "app.h"
#include "wave.h"
#include "stdlib.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif



/* 
    再生する waveファイルの登録
*/

#define NUM_WAVE (26)       // waveの数

/* wave ファイル名のリスト */
const char *fname[] = {
    "voice0.wav", "voice1.wav", "voice2.wav", "voice3.wav", "voice4.wav", "voice5.wav", "voice6.wav", "voice7.wav", 
    "voice8.wav", "voice9.wav", "voice10.wav", "voice11.wav", "voice12.wav", "voice13.wav", "voice14.wav", "voice15.wav",
    "voice16.wav", "voice17.wav", 
    "voiceNashi.wav", "voiceKuro.wav", "voiceAo.wav", "voiceMidori.wav", "voiceKiiro.wav", "voiceAka.wav",  "voiceShiro.wav", "voiceChairo.wav",     
};

/* ファイルを読み込む構造体のメモリをファイル数分用意する */
static memfile_t memfile[NUM_WAVE];

/* 
    wave をメモリーにロードする 
    最初に一回だけ呼ぶ
*/

void init_wave() {
    for (int i = 0; i < NUM_WAVE; i++) {
        char buf[100];
        sprintf(buf, "/ev3rt/res/%s", fname[i]);
        ev3_memfile_load(buf, &memfile[i]);       //SDカード内の"test.wav"を紐づけ
    }
}


/* 
    wave を再生する
    wave : wave番号 wave.hを参照 
*/

int play_wave(wave_t wave) 
{
    if (wave < 0 || wave >= NUM_WAVE) return 0;
    ev3_speaker_set_volume(10);                                 // 音量10に設定  ** 必須、このレベルでないと音が歪む 
    ev3_speaker_play_file(&memfile[wave], SOUND_MANUAL_STOP);   // 音声を再生
    return 1;
}

// EOF
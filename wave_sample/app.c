#include "ev3api.h"
#include "app.h"
#include "wave.h"

/*
        メインタスク　- 駆動輪のモーターを制御して全身する。
    */
void main_task(intptr_t unused) {

    /*
    ev3_speaker_set_volume(50);//音量50に設定
    ev3_speaker_play_tone(NOTE_C4,1000);
    dly_tsk(1000U * 200U);
    ev3_speaker_stop();

    memfile_t mem;//ファイル構造体を作成
    ev3_memfile_load("/ev3rt/res/voice0.wav", &mem);//SDカード内の"test.wav"を紐づけ
    ev3_speaker_set_volume(10);//音量10に設定
    ev3_speaker_play_file(&mem, SOUND_MANUAL_STOP);//音声を再生
    */

    init_wave();

    dly_tsk(1000U * 1000U);
    play_wave(WAVE_0);
    dly_tsk(1000U * 1000U);
    play_wave(WAVE_11);
    dly_tsk(1000U * 1000U);
    play_wave(WAVE_Aka);
    dly_tsk(1000U * 1000U);
    play_wave(WAVE_Kuro);
    dly_tsk(1000U * 1000U);
    play_wave(WAVE_Nashi);

    dly_tsk(1000U * 1000U);

    for (int i = 0; i < 26; i++) {
        dly_tsk(1000U * 1000U);
        play_wave((wave_t)i);
    }



}

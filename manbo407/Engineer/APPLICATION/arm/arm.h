
#ifndef ARM_H
#define ARM_H

void ArmInit(void);
void Arm_bsp();
void ArmTask(void);
void state_control();
void key_state();
void armmotorbsp();
void Emotion_task();
void GimbalVisionFollow(float dt);
void song_task();
#endif // ! ARM_H

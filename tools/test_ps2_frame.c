/* Catches digital/config frames being interpreted as analog throttle. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ps2_gamepad.h"
int main(void){
 uint8_t frame[9]={0xff,0x73,0x5a,0xff,0xff,10,20,30,128};
 ps2_state_t p={0};
 assert(ps2_decode_frame(frame,&p)==PS2_FRAME_OK);
 assert(p.ly==128 && p.lx==30 && p.rx==10 && p.ry==20 && p.buttons==65535);
 frame[1]=0x41;frame[8]=0;ps2_state_t before=p;
 assert(ps2_decode_frame(frame,&p)==PS2_FRAME_BAD);
 assert(memcmp(&p,&before,sizeof p)==0);
 frame[1]=0xf3;assert(ps2_decode_frame(frame,&p)==PS2_FRAME_BAD);
 frame[1]=0x79;frame[8]=255;assert(ps2_decode_frame(frame,&p)==PS2_FRAME_OK && p.ly==255);
 frame[2]=0;assert(ps2_decode_frame(frame,&p)==PS2_FRAME_BAD);
 assert(ps2_decode_frame(frame,0)==PS2_FRAME_BAD);
 puts("PS2 analog frame validation OK");return 0;
}

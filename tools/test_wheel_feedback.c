#include <assert.h>
#include <stdio.h>
#include "wheel_feedback.h"
int main(void){
 wheel_feedback_t w;
 wheel_feedback_init(&w,65000U);
 wheel_feedback_update(&w,464U); /* +1000 across 65535 -> 0 */
 assert(w.counts==1000);
 assert(wheel_feedback_milliturns(&w,2000U)==500);
 wheel_feedback_update(&w,1464U);
 assert(w.counts==2000 && wheel_feedback_milliturns(&w,2000U)==1000);
 wheel_feedback_update(&w,464U);
 assert(w.counts==1000 && wheel_feedback_milliturns(&w,2000U)==500);
 wheel_feedback_update(&w,65000U);
 assert(w.counts==0);
 wheel_feedback_update(&w,64000U);
 assert(w.counts==-1000 && wheel_feedback_milliturns(&w,2000U)==-500);
 assert(wheel_feedback_milliturns(&w,0U)==0);
 wheel_feedback_init(&w,0);
 for(int i=1;i<=1000;i++)wheel_feedback_update(&w,(uint16_t)(i*1000U));
 assert(w.counts==1000000);
 assert(wheel_feedback_milliturns(&w,2000U)==500000);
 wheel_feedback_init(&w,1234U);
 assert(w.counts==0 && w.previous==1234U);
 puts("wheel feedback wrap / half turn / full turn / reverse / reset OK");return 0;
}

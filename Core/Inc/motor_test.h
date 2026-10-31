#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

/* 单轮自检的最小净计数阈值：低于它视为“没有转动证据”，±1 抖动必须被挡掉。 */
#define MOTOR_TEST_MIN_COUNTS 32

/* 原因掩码：区分前向、后向缺反馈和采样异常，不用一个 bool 概括。 */
#define MOTOR_TEST_REASON_SKIPPED 0x0001U
#define MOTOR_TEST_REASON_FORWARD 0x0002U
#define MOTOR_TEST_REASON_REVERSE 0x0004U
#define MOTOR_TEST_REASON_SAMPLE 0x0008U

typedef struct
{
  uint8_t tested;      /* 跳过时为 0：跳过不等于通过 */
  uint8_t forward_ok;
  uint8_t reverse_ok;
  uint16_t reason_mask;
  int32_t forward_counts;  /* 前向段净计数，回绕已在采集时抵消 */
  int32_t reverse_counts;  /* 后向段净计数 */
} motor_test_result_t;

void motor_test_reset(motor_test_result_t *result);
void motor_test_skip(motor_test_result_t *result);
void motor_test_record(motor_test_result_t *result, int32_t forward_counts,
                       int32_t reverse_counts);
void motor_test_mark_sample_invalid(motor_test_result_t *result);
uint8_t motor_test_passed(const motor_test_result_t *result);

/* OLED 短文本：OK / SKIP / NO FB / FWD ERR / REV ERR / FB ERR */
const char *motor_test_reason_text(const motor_test_result_t *result);

/* 汇总：四轮都测过且都通过才是 ALL OK，否则 CHECK END */
const char *motor_test_summary_text(const motor_test_result_t *results,
                                    uint8_t count);

#endif

/* 逐轮自检结果：每轮必须留下前后两段净计数，不能只留一个汇总 bool。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../Core/Src/motor_test.c"

static int text_is(const motor_test_result_t *r, const char *want)
{
  return strcmp(motor_test_reason_text(r), want) == 0;
}

int main(void)
{
  motor_test_result_t r;

  /* 1. 两向都有反馈：通过，且两段净计数分别保存 */
  motor_test_reset(&r);
  motor_test_record(&r, 1500, -1400);
  assert(motor_test_passed(&r) == 1U);
  assert(r.tested == 1U);
  assert(r.forward_counts == 1500);
  assert(r.reverse_counts == -1400);
  assert(r.reason_mask == 0U);
  assert(text_is(&r, "OK"));

  /* 2. 前向缺反馈 */
  motor_test_reset(&r);
  motor_test_record(&r, 0, -1400);
  assert(motor_test_passed(&r) == 0U);
  assert(r.reverse_counts == -1400);
  assert(r.forward_ok == 0U);
  assert(r.reverse_ok == 1U);
  assert((r.reason_mask & MOTOR_TEST_REASON_FORWARD) != 0U);
  assert((r.reason_mask & MOTOR_TEST_REASON_REVERSE) == 0U);
  assert(text_is(&r, "FWD ERR"));

  /* 3. 后向缺反馈 */
  motor_test_reset(&r);
  motor_test_record(&r, 1200, 0);
  assert(motor_test_passed(&r) == 0U);
  assert(text_is(&r, "REV ERR"));

  /* 4. 双向都缺反馈 */
  motor_test_reset(&r);
  motor_test_record(&r, 0, 0);
  assert(motor_test_passed(&r) == 0U);
  assert((r.reason_mask & MOTOR_TEST_REASON_FORWARD) != 0U);
  assert((r.reason_mask & MOTOR_TEST_REASON_REVERSE) != 0U);
  assert(text_is(&r, "NO FB"));

  /* 5. ±1 抖动不算转动 */
  motor_test_reset(&r);
  motor_test_record(&r, 1, -1);
  assert(motor_test_passed(&r) == 0U);
  assert(text_is(&r, "NO FB"));
  motor_test_reset(&r);
  motor_test_record(&r, MOTOR_TEST_MIN_COUNTS - 1, -(MOTOR_TEST_MIN_COUNTS - 1));
  assert(motor_test_passed(&r) == 0U);
  /* 阈值本身算通过，方向取绝对值 */
  motor_test_reset(&r);
  motor_test_record(&r, -MOTOR_TEST_MIN_COUNTS, MOTOR_TEST_MIN_COUNTS);
  assert(motor_test_passed(&r) == 1U);

  /* 6. 主动跳过：不得显示 PASS */
  motor_test_reset(&r);
  motor_test_skip(&r);
  assert(r.tested == 0U);
  assert(motor_test_passed(&r) == 0U);
  assert((r.reason_mask & MOTOR_TEST_REASON_SKIPPED) != 0U);
  assert(text_is(&r, "SKIP"));

  /* 7. 采样异常单独成码，不伪装成“已确认离线” */
  motor_test_reset(&r);
  motor_test_record(&r, 900, -900);
  motor_test_mark_sample_invalid(&r);
  assert(motor_test_passed(&r) == 0U);
  assert((r.reason_mask & MOTOR_TEST_REASON_SAMPLE) != 0U);
  assert(text_is(&r, "FB ERR"));

  /* 8. 汇总文案：全部通过才 ALL OK */
  {
    motor_test_result_t all[4];
    uint8_t i;
    for (i = 0U; i < 4U; i++)
    {
      motor_test_reset(&all[i]);
      motor_test_record(&all[i], 1000, -1000);
      assert(motor_test_passed(&all[i]) == 1U);
    }
    assert(strcmp(motor_test_summary_text(all, 4U), "ALL OK") == 0);
    motor_test_record(&all[2], 1000, 0);
    assert(strcmp(motor_test_summary_text(all, 4U), "CHECK END") == 0);

    /* 跳过不等于通过，汇总也不能是 ALL OK */
    motor_test_reset(&all[2]);
    motor_test_record(&all[2], 1000, -1000);
    motor_test_reset(&all[3]);
    motor_test_skip(&all[3]);
    assert(strcmp(motor_test_summary_text(all, 4U), "CHECK END") == 0);
  }

  puts("motor test per-wheel result / reason mask / summary OK");
  return 0;
}

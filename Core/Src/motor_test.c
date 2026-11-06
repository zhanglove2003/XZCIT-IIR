#include "motor_test.h"

static int32_t magnitude(int32_t value)
{
  return value < 0 ? -value : value;
}

void motor_test_reset(motor_test_result_t *result)
{
  if (result == 0)
  {
    return;
  }
  result->tested = 0U;
  result->forward_ok = 0U;
  result->reverse_ok = 0U;
  result->reason_mask = 0U;
  result->forward_counts = 0;
  result->reverse_counts = 0;
}

void motor_test_skip(motor_test_result_t *result)
{
  motor_test_reset(result);
  if (result == 0)
  {
    return;
  }
  /* 跳过：没测过就没有通过，也不能冒充“已确认离线”以外的判断。 */
  result->reason_mask = MOTOR_TEST_REASON_SKIPPED;
}

void motor_test_record(motor_test_result_t *result, int32_t forward_counts,
                       int32_t reverse_counts)
{
  uint16_t mask;

  if (result == 0)
  {
    return;
  }

  result->tested = 1U;
  result->forward_counts = forward_counts;
  result->reverse_counts = reverse_counts;
  result->forward_ok =
      magnitude(forward_counts) >= (int32_t)MOTOR_TEST_MIN_COUNTS ? 1U : 0U;
  result->reverse_ok =
      magnitude(reverse_counts) >= (int32_t)MOTOR_TEST_MIN_COUNTS ? 1U : 0U;

  mask = (uint16_t)(result->reason_mask & MOTOR_TEST_REASON_SAMPLE);
  if (result->forward_ok == 0U)
  {
    mask = (uint16_t)(mask | MOTOR_TEST_REASON_FORWARD);
  }
  if (result->reverse_ok == 0U)
  {
    mask = (uint16_t)(mask | MOTOR_TEST_REASON_REVERSE);
  }
  result->reason_mask = mask;
}

void motor_test_mark_sample_invalid(motor_test_result_t *result)
{
  if (result == 0)
  {
    return;
  }
  result->reason_mask =
      (uint16_t)(result->reason_mask | MOTOR_TEST_REASON_SAMPLE);
}

uint8_t motor_test_passed(const motor_test_result_t *result)
{
  if (result == 0)
  {
    return 0U;
  }
  if (result->tested == 0U)
  {
    return 0U;
  }
  if (result->reason_mask != 0U)
  {
    return 0U;
  }
  return (uint8_t)(result->forward_ok != 0U && result->reverse_ok != 0U);
}

const char *motor_test_reason_text(const motor_test_result_t *result)
{
  if (result == 0)
  {
    return "SKIP";
  }
  if (result->tested == 0U)
  {
    return "SKIP";
  }
  if ((result->reason_mask & MOTOR_TEST_REASON_SAMPLE) != 0U)
  {
    return "FB ERR";
  }
  if (result->forward_ok == 0U && result->reverse_ok == 0U)
  {
    return "NO FB";
  }
  if (result->forward_ok == 0U)
  {
    return "FWD ERR";
  }
  if (result->reverse_ok == 0U)
  {
    return "REV ERR";
  }
  return "OK";
}

const char *motor_test_summary_text(const motor_test_result_t *results,
                                    uint8_t count)
{
  uint8_t i;

  if (results == 0 || count == 0U)
  {
    return "CHECK END";
  }
  for (i = 0U; i < count; i++)
  {
    if (motor_test_passed(&results[i]) == 0U)
    {
      return "CHECK END";
    }
  }
  return "ALL OK";
}

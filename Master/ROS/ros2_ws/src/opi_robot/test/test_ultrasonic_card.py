"""真实浏览器超声卡片渲染测试。

加载完整 console_page.html（Chrome headless --dump-dom），在页内调用实际
applyStatus(m)，读取 us_big / us_gate / us_barfill 的真实渲染结果。

任务书：docs/superpowers/plans/2026-11-19-ultrasonic-ui-map-stale.md 任务 2。

修复前 bug：us_gate 读取 m.safety.ultrasonic（该路径无 ultrasonic 字段），
导致有底盘距离时安全状态恒显示"无数据"且误用绿色。修复目标：安全状态改读
m.drive_safety.ultrasonic 且仅 drive_safety.state === 'fresh' 时解释；
距离仍取底盘状态新鲜度（标注"底盘距离"）；drive_safety.state 为 stale 时
后端仍携带旧 ultrasonic 对象（web_debug drive_safety_view），不得当新鲜数据
显示；无新鲜安全状态时色条不得用正常色或强调色。
"""

import json
from pathlib import Path
import shutil
import subprocess

import pytest

PAGE = Path(__file__).parents[1] / 'opi_robot/web_node/console_page.html'

CHASSIS_FRESH = {'state': 'fresh', 'ultrasonic_m': 2.07}
CLEAR = {'valid': True, 'fault': False, 'stop': False, 'deny': False,
         'range_m': 2.07, 'stop_distance': 0.4, 'release_distance': 0.5}

# (用例名, drive_safety, chassis, 期望距离, 期望 gate 文本,
#  期望 gate class（'ok'/'bad'/'neutral'）, 期望色条（'acc'/'bad'/'neutral'）)
CASES = [
    ('fresh_clear',
     {'state': 'fresh', 'ultrasonic': CLEAR}, CHASSIS_FRESH,
     '2.07', '超声未触发停车', 'ok', 'acc'),
    ('fresh_gate_stale_fault',
     {'state': 'fresh',
      'ultrasonic': dict(CLEAR, valid=False, fault=True, range_m=None,
                         fault_reason='stale')}, CHASSIS_FRESH,
     '2.07', '超声故障（stale）', 'bad', 'bad'),
    ('fresh_near_stop',
     {'state': 'fresh', 'ultrasonic': dict(CLEAR, stop=True, deny=True)},
     CHASSIS_FRESH,
     '2.07', '超声停车', 'bad', 'bad'),
    ('stale_carries_old_ultra',
     {'state': 'stale', 'ultrasonic': CLEAR}, CHASSIS_FRESH,
     '2.07', '状态已过期', 'neutral', 'neutral'),
    ('unknown_state',
     {'state': 'unknown'}, CHASSIS_FRESH,
     '2.07', '状态无数据', 'neutral', 'neutral'),
    ('fresh_without_ultra',
     {'state': 'fresh'}, CHASSIS_FRESH,
     '2.07', '状态无数据', 'neutral', 'neutral'),
    ('fresh_incomplete_ultra',
     {'state': 'fresh', 'ultrasonic': {'valid': True}}, CHASSIS_FRESH,
     '2.07', '状态无数据', 'neutral', 'neutral'),
    # 底盘过期/缺失：距离显示 --，即使融合状态新鲜也不能把旧底盘距离当实时值
    ('chassis_stale_hides_distance',
     {'state': 'fresh', 'ultrasonic': CLEAR},
     {'state': 'stale', 'ultrasonic_m': 2.07},
     '--', '超声未触发停车', 'ok', 'acc'),
    ('chassis_missing_hides_distance',
     {'state': 'fresh', 'ultrasonic': CLEAR}, {},
     '--', '超声未触发停车', 'ok', 'acc'),
]

INJECT_TEMPLATE = r'''<script>
(function () {
  const CASES = [
    __CASES__
  ];
  const out = [];
  for (const c of CASES) {
    try {
      const m = Object.assign(
        {t: 'status', mode: 'map', scan: null, scan_hz: null,
         scan_valid: null, imu: {}, lidar_diag: {}, image: {},
         patrol: {}, nav: {}, boot: {}, network: {}, nodes: [],
         battery: {}, odom: {}, map: null, safety: {}},
        {chassis: c.chassis, drive_safety: c.ds});
      window.applyStatus(m);
      const bar = document.getElementById('us_barfill');
      out.push([c.name,
        document.getElementById('us_big').textContent,
        document.getElementById('us_gate').textContent,
        document.getElementById('us_gate').className,
        bar.style.width, bar.style.background].join('|'));
    } catch (err) {
      out.push(c.name + '|error:' + err.message);
    }
  }
  document.body.dataset.result = out.join(';;');
})();
</script>'''


def _run_chrome_dump(tmp_path, html):
    test_page = tmp_path / 'ultrasonic_card_test.html'
    test_page.write_text(html, encoding='utf-8')
    chrome = shutil.which('google-chrome')
    result = subprocess.run(
        [chrome, '--headless', '--no-sandbox', '--disable-gpu',
         '--disable-background-networking', '--disable-extensions',
         '--no-first-run', '--force-device-scale-factor=1',
         f'--user-data-dir={tmp_path / "chrome"}', '--dump-dom',
         test_page.as_uri()],
        capture_output=True, text=True, timeout=60, check=True,
    )
    marker = 'data-result="'
    assert marker in result.stdout, '页面未产生测试结果: %s' % result.stderr[-800:]
    start = result.stdout.index(marker) + len(marker)
    end = result.stdout.index('"', start)
    return result.stdout[start:end]


def _render_rows(raw):
    rows = {}
    for row in raw.split(';;'):
        if not row:
            continue
        parts = row.split('|')
        name = parts[0]
        if len(parts) >= 6:
            rows[name] = parts[1:6]
        else:
            rows[name] = None          # error:xxx 等异常行
    return rows


def test_ultrasonic_card_states(tmp_path):
    """超声卡片：距离=底盘新鲜度；安全状态=drive_safety（仅 fresh 解释）。"""
    if shutil.which('google-chrome') is None:
        pytest.skip('需要 Google Chrome 验证真实 DOM 行为')
    page_src = PAGE.read_text(encoding='utf-8')
    cases_js = ',\n    '.join(
        '{name: %s, ds: %s, chassis: %s}' % (
            json.dumps(name), json.dumps(ds), json.dumps(ch))
        for name, ds, ch, *_ in CASES)
    inject = INJECT_TEMPLATE.replace('__CASES__', cases_js)
    assert '</body>' in page_src
    html = page_src.replace('</body>', inject + '\n</body>')
    raw = _run_chrome_dump(tmp_path, html)
    rows = _render_rows(raw)

    failed = []
    for name, _ds, _ch, exp_dist, exp_text, exp_class, exp_bar in CASES:
        got = rows.get(name)
        if got is None:
            failed.append('%s: 页内执行出错或无输出: %r' % (name, raw))
            continue
        dist, text, cls, bar_w, bar_bg = got
        errs = []
        if dist != exp_dist:
            errs.append('距离 %r != 期望 %r' % (dist, exp_dist))
        if text != exp_text:
            errs.append('状态文本 %r != 期望 %r' % (text, exp_text))
        tokens = cls.split()
        if exp_class == 'neutral':
            if 'ok' in tokens or 'bad' in tokens:
                errs.append('状态 class %r 应为中性（不得含 ok/bad）' % cls)
        elif exp_class not in tokens:
            errs.append('状态 class %r 应含 %r' % (cls, exp_class))
        if exp_bar == 'neutral':
            neutral_bar = bar_w == '0%' or (
                '--acc' not in bar_bg and '--bad' not in bar_bg)
            if not neutral_bar:
                errs.append('色条 (width=%r, bg=%r) 应为中性'
                            % (bar_w, bar_bg))
        elif exp_bar not in bar_bg:
            errs.append('色条 bg %r 应含 %r' % (bar_bg, exp_bar))
        if errs:
            failed.append('%s: %s' % (name, '; '.join(errs)))
    assert not failed, '失败用例:\n' + '\n'.join(failed)

"""真实浏览器交互反馈测试：toast、按钮 :active/busy、us_rearm 未登录提示。

任务书外用户追加需求（2026-11-19）：
1. 点击按钮无任何反馈，用户误以为页面卡死 → 要求全局按钮 :active 反馈
   与动作按钮 busy 态。
2. us_rearm 未登录时提示写在巡航页签的 patrol_message 里，地图页看不到
   → 要求全局可见提示并联动登录；服务端 ultrasonic_rearm 成功回执
   （t:'ultrasonic_status'）此前前端无人处理，同样零反馈。

测试加载完整 console_page.html（Chrome headless --dump-dom），在页内直接
驱动真实交互：调用 showToast、点击 us_rearm、以桩 ws 发送 patrolSend，
读取 toast_host / classList / 发送帧的真实结果。
"""

import json
from pathlib import Path
import shutil
import subprocess

import pytest

PAGE = Path(__file__).parents[1] / 'opi_robot/web_node/console_page.html'

INJECT_TEMPLATE = r'''<script>
(function () {
  const out = [];
  const push = (name, ok, detail) =>
    out.push([name, ok ? 'PASS' : 'FAIL', detail || ''].join('|'));
  try {
    const host = document.getElementById('toast_host');
    push('toast_host_exists', !!host);
    push('toast_fn_exists', typeof window.showToast === 'function');
    if (host && typeof window.showToast === 'function') {
      window.__login_opened = false;
      window.showToast('交互测试消息', 'bad',
        {label: '去登录', fn: function () { window.__login_opened = true; }});
      const first = host.firstElementChild;
      push('toast_shows', !!first && (first.className || '').indexOf('bad') >= 0
        && first.textContent.indexOf('交互测试消息') >= 0,
        first ? first.className + ' / ' + first.textContent : 'no node');
      const act = first.querySelector('button');
      push('toast_action_label', !!act && act.textContent === '去登录');
      if (act) act.click();
      push('toast_action_fn', window.__login_opened === true);
    }
    let hasActive = false, hasBusy = false;
    for (const sheet of document.styleSheets) {
      let rules;
      try { rules = sheet.cssRules; } catch (e) { continue; }
      for (const r of rules) {
        const sel = r.selectorText || '';
        if (sel.indexOf('button:active') >= 0) hasActive = true;
        if (sel.indexOf('.busy') >= 0) hasBusy = true;
      }
    }
    push('css_active_rule', hasActive);
    push('css_busy_rule', hasBusy);
    push('auth_initial_false', manualAuthenticated === false,
        'manualAuthenticated=' + manualAuthenticated);
    document.getElementById('us_rearm').click();
    let loginToast = null;
    for (const el of host.children) {
      if (el.textContent.indexOf('请先登录') >= 0) loginToast = el;
    }
    push('rearm_login_toast', !!loginToast,
        loginToast ? loginToast.textContent : 'no toast');
    host.innerHTML = '';
    manualAuthenticated = true;
    patrolSend({t: 'ultrasonic_rearm'});
    let offlineToast = null;
    for (const el of host.children) {
      if (el.textContent.indexOf('连接已断开') >= 0) offlineToast = el;
    }
    push('offline_toast', !!offlineToast);
    host.innerHTML = '';
    ws = {readyState: 1, send: function (s) {
      (window.__sent = window.__sent || []).push(s);
    }};
    const btn = document.getElementById('us_rearm');
    btn.click();
    push('rearm_sent', (window.__sent || []).length === 2,
        'sent=' + (window.__sent || []).length);
    push('rearm_busy', btn.classList.contains('busy'));
    let sentToast = null;
    for (const el of host.children) {
      if (el.textContent.indexOf('已发送') >= 0) sentToast = el;
    }
    push('rearm_sent_toast', !!sentToast);
  } catch (err) {
    push('exception', false, err.message);
  }
  document.body.dataset.result = out.join(';;');
})();
</script>'''


def test_web_interactions(tmp_path):
    if shutil.which('google-chrome') is None:
        pytest.skip('需要 Google Chrome 验证真实 DOM 行为')
    page_src = PAGE.read_text(encoding='utf-8')
    inject = INJECT_TEMPLATE
    assert '</body>' in page_src
    html = page_src.replace('</body>', inject + '\n</body>')
    test_page = tmp_path / 'interaction_test.html'
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
    rows = result.stdout[start:end].split(';;')
    failed = []
    for row in rows:
        if not row:
            continue
        parts = row.split('|')
        if len(parts) >= 2 and parts[1] != 'PASS':
            failed.append('%s: %s' % (parts[0], parts[2] if len(parts) > 2
                                      else parts[1]))
    assert rows and not failed, '交互测试失败:\n' + '\n'.join(failed)
    assert json.dumps(len(rows)) and len(rows) >= 13, '用例数量不足: %r' % rows

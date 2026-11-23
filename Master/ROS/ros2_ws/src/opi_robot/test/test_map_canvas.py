"""真实 Canvas 绘制：栅格必须与地图坐标和车辆位置一致。"""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_occupancy_rows_match_world_coordinates(tmp_path):
    chrome = shutil.which('google-chrome')
    if chrome is None:
        pytest.skip('需要 Google Chrome 验证 Canvas 像素')

    page = (Path(__file__).parents[1] / 'opi_robot/web_node/console_page.html')
    source = page.read_text(encoding='utf-8')
    start = source.index('function renderMap() {')
    end = source.index("\n$('map_zoom').oninput", start)
    render_map = source[start:end]
    html = r'''<!doctype html><html><body>
<canvas id="mapcanvas" style="width:120px;height:120px"></canvas>
<input id="map_zoom" value="12">
<span id="map_state"></span><span id="mp_map_size"></span>
<span id="mp_map_age"></span><span id="pose_x"></span>
<span id="pose_y"></span><span id="pose_th"></span><span id="pose_src"></span>
<script>
const $ = id => document.getElementById(id);
const mapOff = document.createElement('canvas');
const MAP_VIEW_ROTATION = Math.PI / 2;
const mapGrid = {w: 3, h: 2, res: 1, ox: 0, oy: 0, age: 0,
                 data: new Uint8Array([100, 0, 0, 0, 0, 100])};
const mapPose = {x: 0, y: 0, th: 0}, mapPoseAge = 0;
const cloud = [], POLAR = 720, navPick = null, navSt = null;
''' + render_map + r'''
try {
  renderMap();
  const ctx = $('mapcanvas').getContext('2d');
  const white = (x, y) => Array.from(ctx.getImageData(x, y, 1, 1).data).join(',')
    === '230,233,239,255';
  // Cell centers: (0.5, 0.5) and (2.5, 1.5), converted to canvas pixels.
  const actual = [white(55, 55), white(45, 35), white(45, 55), white(55, 35)];
  document.body.dataset.result = actual.join(',');
} catch (error) {
  document.body.dataset.result = 'error:' + error.message;
}
</script></body></html>'''
    test_page = tmp_path / 'map_test.html'
    test_page.write_text(html, encoding='utf-8')
    result = subprocess.run(
        [chrome, '--headless', '--no-sandbox', '--disable-gpu',
         '--disable-background-networking', '--disable-extensions',
         '--no-first-run', '--force-device-scale-factor=1',
         f'--user-data-dir={tmp_path / "chrome"}', '--dump-dom', test_page.as_uri()],
        capture_output=True, text=True, timeout=30, check=True,
    )
    assert 'data-result="true,true,false,false"' in result.stdout, result.stdout


def test_calibrated_rotation_aligns_pose_grid_and_click(tmp_path):
    chrome = shutil.which('google-chrome')
    if chrome is None:
        pytest.skip('需要 Google Chrome 验证 Canvas 像素')

    page = Path(__file__).parents[1] / 'opi_robot/web_node/console_page.html'
    source = page.read_text(encoding='utf-8')
    start = source.index('let mapGrid = null')
    prelude = source[start:source.index('/* [C50C 阶段 D', start)]
    prelude = '\n'.join(
        'const MAP_VIEW_ROTATION = Math.PI - 2.1821283515400722;'
        if line.startswith('const MAP_VIEW_ROTATION = ') else line
        for line in prelude.splitlines()
    ) + '\n'
    start = source.index('function renderMap() {')
    render_map = source[start:source.index("\n$('map_zoom').oninput", start)]
    start = source.index('function screenToMap(sx, sy) {')
    screen_to_map = source[start:source.index("\n$('mapcanvas').addEventListener", start)]
    html = r'''<!doctype html><html><body>
<canvas id="mapcanvas" style="width:120px;height:120px"></canvas>
<input id="map_zoom" value="12">
<span id="map_state"></span><span id="mp_map_size"></span>
<span id="mp_map_age"></span><span id="pose_x"></span>
<span id="pose_y"></span><span id="pose_th"></span><span id="pose_src"></span>
<script>
const $ = id => document.getElementById(id);
''' + prelude + r'''
const mapOff = document.createElement('canvas');
mapGrid = {w: 3, h: 2, res: 1, ox: 0, oy: 0, age: 0,
           data: new Uint8Array([100, 0, 0, 0, 0, 100])};
mapPose = {x: 0, y: 0, th: 2.1821283515400722}; mapPoseAge = 0;
const cloud = [], POLAR = 720, navPick = null, navSt = null;
''' + render_map + screen_to_map + r'''
try {
  renderMap();
  const ctx = $('mapcanvas').getContext('2d');
  const rgba = (x, y) => Array.from(ctx.getImageData(x, y, 1, 1).data);
  const blue = (x, y) => { const p = rgba(x, y); return p[0] < 100 && p[2] > 200; };
  const white = (x, y) => rgba(x, y).join(',') === '230,233,239,255';
  const p = screenToMap(47, 44);
  const actual = [blue(58, 60) && !blue(60, 56), white(58, 53), white(62, 31),
                  Math.abs(p.x - 0.5640683817629955) < 1e-9,
                  Math.abs(p.y - 1.982883471285006) < 1e-9];
  document.body.dataset.result = actual.join(',');
} catch (error) {
  document.body.dataset.result = 'error:' + error.message;
}
</script></body></html>'''
    test_page = tmp_path / 'calibrated_map_test.html'
    test_page.write_text(html, encoding='utf-8')
    result = subprocess.run(
        [chrome, '--headless', '--no-sandbox', '--disable-gpu',
         '--disable-background-networking', '--disable-extensions',
         '--no-first-run', '--force-device-scale-factor=1',
         f'--user-data-dir={tmp_path / "chrome"}', '--dump-dom', test_page.as_uri()],
        capture_output=True, text=True, timeout=30, check=True,
    )
    assert 'data-result="true,true,true,true,true"' in result.stdout, result.stdout


def test_field_calibration_aligns_pose_grid_cloud_and_click(tmp_path):
    chrome = shutil.which('google-chrome')
    if chrome is None:
        pytest.skip('需要 Google Chrome 验证 Canvas 像素')

    page = Path(__file__).parents[1] / 'opi_robot/web_node/console_page.html'
    source = page.read_text(encoding='utf-8')
    start = source.index('let mapGrid = null')
    prelude = source[start:source.index('/* [C50C 阶段 D', start)]
    start = source.index('function renderMap() {')
    render_map = source[start:source.index("\n$('map_zoom').oninput", start)]
    start = source.index('function screenToMap(sx, sy) {')
    screen_to_map = source[start:source.index("\n$('mapcanvas').addEventListener", start)]
    html = r'''<!doctype html><html><body>
<canvas id="mapcanvas" style="width:120px;height:120px"></canvas>
<input id="map_zoom" value="12">
<span id="map_state"></span><span id="mp_map_size"></span>
<span id="mp_map_age"></span><span id="pose_x"></span>
<span id="pose_y"></span><span id="pose_th"></span><span id="pose_src"></span>
<script>
const $ = id => document.getElementById(id);
''' + prelude + r'''
const mapOff = document.createElement('canvas');
mapGrid = {w: 3, h: 2, res: 1, ox: 0, oy: 0, age: 0,
           data: new Uint8Array([100, 0, 0, 0, 0, 100])};
/* 仅用于离线回归的 14:35 样本（13:15 用户确认车头正西；车辆静止断电，
   map→base_link 随 SLAM 漂移更新为 −3.8084°），不直接作为后续现场实时值；
   期望值由旋转公式独立重算：R0 = π − th ≈ 3.208062015370946，
   占据格屏幕 (55.34,65.32)/(36.05,76.63)，正前 2m 点恒落 (40,60)。 */
mapPose = {x: 0, y: 0, th: -0.066469361781153}; mapPoseAge = 0;
const cloud = Array(720).fill(0), POLAR = 720, navPick = null, navSt = null;
cloud[0] = 1.92;
''' + render_map + screen_to_map + r'''
try {
  renderMap();
  const ctx = $('mapcanvas').getContext('2d');
  const rgba = (x, y) => Array.from(ctx.getImageData(x, y, 1, 1).data);
  const blue = (x, y) => { const p = rgba(x, y); return p[0] < 100 && p[2] > 200; };
  const white = (x, y) => rgba(x, y).join(',') === '230,233,239,255';
  const p = screenToMap(40, 60);
  const actual = [blue(58, 60) && !blue(60, 56), white(55, 65), white(36, 76),
                  rgba(40, 60)[1] > 180 && rgba(40, 60)[2] > 200,
                  Math.abs(p.x - 1.9955834503948298) < 1e-9,
                  Math.abs(p.y + 0.13284085407082283) < 1e-9];
  document.body.dataset.result = actual.join(',');
} catch (error) {
  document.body.dataset.result = 'error:' + error.message;
}
</script></body></html>'''
    test_page = tmp_path / 'calibrated_map_test.html'
    test_page.write_text(html, encoding='utf-8')
    result = subprocess.run(
        [chrome, '--headless', '--no-sandbox', '--disable-gpu',
         '--disable-background-networking', '--disable-extensions',
         '--no-first-run', '--force-device-scale-factor=1',
         f'--user-data-dir={tmp_path / "chrome"}', '--dump-dom', test_page.as_uri()],
        capture_output=True, text=True, timeout=30, check=True,
    )
    assert 'data-result="true,true,true,true,true,true"' in result.stdout, result.stdout

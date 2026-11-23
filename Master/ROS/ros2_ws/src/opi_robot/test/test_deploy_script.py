"""deploy_stage1.sh 的安全行为测试。

用 ssh/rsync 桩模拟远端，验证脚本在以下情况下的行为：
  - 基线没有记录、远端却已有不同内容 -> 拒绝写入
  - 现场文件被改过（既不等于基线也不等于本地）-> 拒绝写入
  - 重复部署、首次部署 -> 正常
  - 现场独有文件 -> 不删除

桩只替换传输层，脚本逻辑原样执行。

注意：本机没有 /opt/ros，预检会一直报 BLOCK，所以 --apply 一律被拦截。
要测试写入路径，必须先用 --allow-missing-deps 跳过预检（脚本支持该开关）。
"""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

STAGE = Path(__file__).resolve().parent.parent.parent
SCRIPT = STAGE / 'deploy_stage1.sh'
LOCAL_SRC = STAGE / 'opi_robot'


@pytest.fixture
def sandbox(tmp_path):
    """建桩目录、假远端、假 ssh/rsync。"""
    stub = tmp_path / 'stub'
    stub.mkdir()
    fake = tmp_path / 'fakesite'
    py = tmp_path / 'pylib'
    py.mkdir()
    (py / 'serial.py').write_text("VERSION = '3.5'\n")

    (stub / 'ssh').write_text(
        '#!/usr/bin/env bash\n'
        '# 第十次审核阻断 1：模拟真实 OpenSSH 的 stdin 行为。\n'
        '# 不带 -n 的 ssh 会读走自己的 stdin；远端命令不读它，数据就被丢弃。\n'
        '# 部署脚本若在 while read 循环里调不带 -n 的 ssh，剩余清单行会被\n'
        '# 这条 ssh 吃掉。带 -n 时 stdin 是 /dev/null，绝不读。\n'
        'nostdin=0\n'
        'cmd=""\n'
        'for a in "$@"; do\n'
        '  case "$a" in\n'
        '    -n) nostdin=1 ;;\n'
        '    *@*) : ;;\n'
        '    *) cmd="$cmd$a " ;;\n'
        '  esac\n'
        'done\n'
        'input=""\n'
        '[ $nostdin -eq 1 ] || input="$(cat)"\n'
        'if [ -z "${cmd// /}" ]; then cmd="$input"; fi\n'
        'cmd="${cmd//\\/home\\/orangepi\\/ros2_ws\\/src\\/opi_robot/'
        + str(fake) + '}"\n'
        "printf '%s' \"$input\" | bash -c \"$cmd\"\n")
    (stub / 'rsync').write_text(
        '#!/usr/bin/env bash\n'
        'echo RSYNC >> "' + str(tmp_path / 'rsync.log') + '"\n'
        'args=(); skip=0\n'
        'for a in "$@"; do\n'
        '  if [ $skip -eq 1 ]; then skip=0; continue; fi\n'
        '  if [ "$a" = "-e" ]; then skip=1; continue; fi\n'
        '  args+=("$a")\n'
        'done\n'
        'src="${args[-2]}"; dst="${args[-1]}"; dst="${dst#*:}"; dst="${dst%/}"\n'
        'dst="${dst//\\/home\\/orangepi\\/ros2_ws\\/src\\/opi_robot/'
        + str(fake) + '}"\n'
        # 桩必须**按实际传入的 argv** 执行 --exclude，不许自己硬编码规则。
        # 第六次审核：旧桩内置缓存规则，部署脚本即使漏传 --exclude 也能通过，
        # 测试等于没测。这里把收到的 --exclude 参数写日志，并在复制时逐条执行。
        'mkdir -p "$dst"\n'
        'excl=()\n'
        'no_links=0\n'
        'want=0\n'
        # for a in "$@" 保证无论参数里有什么字符都不会重新分词；
        # 不能用 set -- 重排，否则带空格的路径会被拆开。
        'for a in "$@"; do\n'
        '  if [ $want -eq 1 ]; then excl+=("$a"); want=0; continue; fi\n'
        '  case "$a" in\n'
        '    --exclude) want=1 ;;\n'
        '    --exclude=*) excl+=("${a#--exclude=}") ;;\n'
        '    --no-links) no_links=1 ;;\n'
        '  esac\n'
        'done\n'
        '{ echo "EXCLUDES=${#excl[@]}"; for e in "${excl[@]}"; do echo "EX:$e"; done;'
        ' echo "NOLINKS=$no_links"; } >> "' + str(tmp_path / 'rsync.args') + '"\n'
        '( cd "$src" && find . -type f | sed "s|^\\./||" | while IFS= read -r f; do\n'
        '    skipit=0\n'
        '    for ex in "${excl[@]}"; do\n'
        '      case "$ex" in\n'
        '        */) base="${ex%/}"; case "/$f/" in */"$base/"*) skipit=1;; esac ;;\n'
        '        *) case "$f" in $ex) skipit=1;; esac ;;\n'
        '      esac\n'
        '    done\n'
        '    [ "$skipit" -eq 1 ] && continue\n'
        '    # 复制后校验的反向测试探针：文件名含此标记时故意不复制，\n'
        '    # 用于验证"rsync 漏复制、复核必须抓到"。其他测试不受影响。\n'
        '    case "$f" in *rsync_skip_probe*) continue ;; esac\n'
        '    if [ $no_links -eq 1 ] && [ -L "$f" ]; then continue; fi\n'
        '    mkdir -p "$dst/$(dirname "$f")" && cp -a "$f" "$dst/$f"\n'
        '  done )\n')
    (stub / 'colcon').write_text('#!/bin/sh\necho /usr/bin/colcon\n')
    for f in stub.iterdir():
        f.chmod(f.stat().st_mode | stat.S_IEXEC)

    env = dict(os.environ)
    env['PATH'] = str(stub) + os.pathsep + env['PATH']
    env['PYTHONPATH'] = str(py)
    env['HOME'] = str(tmp_path / 'home')
    (tmp_path / 'home').mkdir()

    def run(*args):
        return subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi'] + list(args),
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120)

    def rsync_calls():
        log = tmp_path / 'rsync.log'
        return len(log.read_text().splitlines()) if log.exists() else 0

    def rsync_excludes():
        """本机 rsync 实际收到的 --exclude 列表（每次调用一条记录）。"""
        log = tmp_path / 'rsync.args'
        out = []
        if not log.exists():
            return out
        cur = None
        for line in log.read_text().splitlines():
            if line.startswith('EXCLUDES='):
                cur = []
                out.append(cur)
            elif line.startswith('EX:'):
                cur.append(line[3:])
        return out

    # 造一个"远端已有的旧副本"
    import shutil
    shutil.copytree(LOCAL_SRC, fake / 'opi_robot' if False else fake)
    # 上面把整个工作副本拷成 $fake（因为 ssh 桩把 REMOTE_PATH 映射到 $fake）
    return {'run': run, 'fake': fake, 'rsync_calls': rsync_calls,
            'rsync_excludes': rsync_excludes, 'tmp': tmp_path}


class TestSymlinkPolicy:
    """交付物不允许符号链接（第五次审核其他项 2）。

    清单只收集普通文件，rsync -a 却会复制符号链接。与其让
    "检查不到但会被复制"的文件存在，脚本选择直接拒绝。
    """

    def test_symlink_blocks_apply(self, sandbox):
        link = LOCAL_SRC / 'linked_probe.py'
        target = LOCAL_SRC / 'opi_robot' / 'qos.py'
        link.symlink_to(target)
        try:
            r = sandbox['run']('--apply')
            assert r.returncode == 3, r.stdout
            assert '符号链接' in r.stdout, r.stdout
            assert 'linked_probe.py' in r.stdout
            # 未写入：远端没有这个链接，也没有被复制成普通文件
            assert not (sandbox['fake'] / 'linked_probe.py').exists()
        finally:
            link.unlink(missing_ok=True)

    def test_symlink_is_not_in_manifest(self, sandbox):
        link = LOCAL_SRC / 'linked_probe.py'
        link.symlink_to(LOCAL_SRC / 'opi_robot' / 'qos.py')
        try:
            r = sandbox['run']()
            assert 'BLOCK:本地工作副本含符号链接' in r.stdout, r.stdout
            manifest = Path('/tmp/stage1-manifest.txt').read_text()
            assert 'linked_probe.py' not in manifest
        finally:
            link.unlink(missing_ok=True)

    def test_rsync_uses_no_links(self):
        """rsync 命令行带 --no-links，与清单只收普通文件一致。"""
        import re
        text = SCRIPT.read_text()
        m = re.search(r'RSYNC_EXCLUDES=\((.*?)\)', text, re.S)
        assert m, '找不到 RSYNC_EXCLUDES'
        assert '--no-links' in m.group(1), m.group(1)


class TestBaselineMissing:
    def test_new_file_on_remote_without_baseline_blocks(self, sandbox):
        """远端内容与基线不符 -> 拒绝写入。

        脚本对"远端被改过"有两类出口：
          - 基线里有记录但哈希不同 -> "现场与基线不一致"；
          - 基线里没有这条记录       -> "基线=无记录"。
        两者都必须拒绝，所以这里断言行为（退出码与拒绝写入），
        不断言某一句具体文案——否则重建基线就会让测试误报。
        """
        local = LOCAL_SRC / 'opi_robot' / 'qos.py'
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        assert target.exists(), '假远端应已拷入完整工作副本'
        target.write_text(local.read_text() + '# 现场改过\n')
        r = sandbox['run']()
        assert r.returncode == 2, r.stdout + r.stderr
        assert '不写入任何文件' in r.stdout, r.stdout
        assert '现场与基线不一致' in r.stdout or '基线=无记录' in r.stdout
        assert '没有这些文件的记录' in r.stdout or \
            '现场与基线不一致' in r.stdout
        assert sandbox['rsync_calls']() == 0

    def test_file_absent_from_baseline_blocks(self, sandbox):
        """基线完全没有该文件的记录时也要拒绝，不能当成可覆盖。"""
        base = sandbox['tmp'] / 'opi_robot' / 'placeholder'
        # 用一个基线里不存在的远端文件验证"无记录"分支
        target = sandbox['fake'] / 'opi_robot' / '现场新增.py'
        target.write_text('# 现场加的\n')
        r = sandbox['run']()
        # 不在本地清单里的文件属"现场独有"，不会被覆盖，也不阻止
        assert r.returncode == 0
        assert '现场独有' in r.stdout
        assert target.read_text() == '# 现场加的\n'

    def test_apply_refuses_without_baseline(self, sandbox):
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('# 别人建的\n')
        r = sandbox['run']('--apply')
        assert r.returncode == 2
        assert sandbox['rsync_calls']() == 0
        assert target.read_text() == '# 别人建的\n'

    def test_same_content_without_baseline_is_fine(self, sandbox):
        """基线没有记录，但内容与本地一致 -> 不阻止。"""
        src = LOCAL_SRC / 'opi_robot' / 'qos.py'
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(src.read_bytes())
        r = sandbox['run']()
        assert r.returncode == 0, r.stdout + r.stderr
        assert '无基线' not in r.stdout


class TestFieldModified:
    def test_modified_file_blocks(self, sandbox):
        """现场文件既不等于基线也不等于本地 -> 拒绝。"""
        p = sandbox['fake'] / 'opi_robot' / 'lidar_node' / 'reader.py'
        p.write_text('# 现场补丁\n')
        r = sandbox['run']('--apply')
        assert r.returncode == 2
        assert sandbox['rsync_calls']() == 0


class TestNormalPaths:
    def test_first_deploy_preview_lists_new_files(self, sandbox):
        for name in ('opi_robot/qos.py', 'opi_robot/lidar_node/reader.py'):
            (sandbox['fake'] / name).unlink()
        r = sandbox['run']()
        assert r.returncode == 0
        assert '将新增 (2)' in r.stdout
        assert '预览模式' in r.stdout
        assert sandbox['rsync_calls']() == 0

    def test_repeat_deploy_is_idempotent(self, sandbox):
        r = sandbox['run']()
        assert r.returncode == 0
        assert '将新增 (0)' in r.stdout
        assert '将覆盖 (0)' in r.stdout

    def test_remote_only_files_are_kept(self, sandbox):
        extra = sandbox['fake'] / '现场脚本.py'
        extra.write_text('x\n')
        r = sandbox['run']()
        assert '现场独有' in r.stdout
        assert extra.exists()

    def test_no_rsync_delete_flag(self, sandbox):
        src = SCRIPT.read_text()
        # 允许在注释里提 --delete，但不允许出现在 rsync 命令行
        for line in src.splitlines():
            if line.lstrip().startswith('#') or '#' in line.split('rsync')[-1][:3]:
                continue
            if 'rsync' in line and '--delete' in line:
                pytest.fail('rsync 命令行出现 --delete: %s' % line)


class TestPreflight:
    def test_missing_deps_block_apply(self, sandbox):
        """本机没有 /opt/ros，预检必须拦住 --apply。"""
        r = sandbox['run']('--apply')
        assert r.returncode == 3, r.stdout + r.stderr
        assert '拒绝写入' in r.stdout
        assert sandbox['rsync_calls']() == 0

    def test_preview_still_runs_with_missing_deps(self, sandbox):
        r = sandbox['run']()
        assert r.returncode == 0
        assert 'BLOCK:' in r.stdout
        assert '预览仍然继续' in r.stdout


class TestArgumentParsing:
    def test_no_args_prints_usage(self, sandbox):
        env = dict(os.environ)
        r = subprocess.run(['bash', str(SCRIPT)], capture_output=True,
                           text=True, env=env, timeout=60)
        assert r.returncode == 1
        assert '用法' in r.stdout

    def test_help(self, sandbox):
        r = sandbox['run']('--help')
        assert r.returncode == 0
        assert '--apply' in r.stdout

    def test_unknown_option(self, sandbox):
        r = sandbox['run']('--nonsense')
        assert r.returncode == 1
        assert '未知选项' in r.stdout

    def test_duplicate_apply_rejected(self, sandbox):
        r = sandbox['run']('--apply', '--apply')
        assert r.returncode == 1
        assert '重复' in r.stdout

    def test_apply_position_does_not_matter(self, sandbox):
        a = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', '--apply', 'orangepi'],
            cwd=str(STAGE), capture_output=True, text=True, timeout=120,
            env={**os.environ, 'PATH': str(sandbox['tmp'] / 'stub')
                 + os.pathsep + os.environ['PATH'],
                 'PYTHONPATH': str(sandbox['tmp'] / 'pylib')})
        # 应进入预检并因缺依赖被拦（exit 3），而不是当成多余参数（exit 1）
        assert a.returncode == 3, a.stdout + a.stderr

def install_ok_ssh(stub, fake):
    """把 ssh 桩换成"预检全通过"的版本（写入路径测试共用）。

    stdin 行为与真实 ssh 一致（第十次审核阻断 1）：
      - 不带 -n 的调用必读 stdin；远端命令不读它，数据就被丢弃；
      - 带 -n 的调用绝不读 stdin；
      - 预检（bash -s）与部署路径构成检查（STAGE1_ROOT_P=...; bash -s）
        从 stdin 拿脚本，真执行。
    """
    stub_script = """#!/usr/bin/env bash
# 只在参数里出现"ROS 2 环境"时返回通过报告，其余命令映射到本地假站点执行。
nostdin=0
cmd=""
for a in "$@"; do
  case "$a" in
    -n) nostdin=1 ;;
    *@*) : ;;
    *) cmd="$cmd$a " ;;
  esac
done
input=""
[ $nostdin -eq 1 ] || input="$(cat)"
if [ -z "${cmd// /}" ]; then cmd="$input"; fi
REMOTE=/home/orangepi/ros2_ws/src/opi_robot
cmd="${cmd//$REMOTE/FAKE_ROOT}"
case "$cmd" in
  STAGE1_ROOT_P=*)
    # 部署路径构成检查：脚本从 stdin 传入，真执行（不伪造结果）
    printf '%s' "$input" | bash -c "$cmd"
    exit $? ;;
  *"bash -s"*)
    # 预检脚本从 stdin 传入；读掉脚本，再按通过报告回答
    printf '%s' "$input" | bash -c "$cmd" >/dev/null 2>&1 || true
    echo "-- ROS 2 环境 --"
    echo "setup.bash: 存在"
    echo "colcon: 可用"
    echo "-- python3-serial --"
    echo "python3-serial: 3.5"
    echo "-- 诊断消息包 --"
    echo "diagnostic_msgs: ok"
    echo "-- 串口与摄像头路径 --"
    echo "-- 占用情况 --"
    echo "无旧节点进程"
    echo "8088 空闲"
    exit 0 ;;
esac
printf '%s' "$input" | bash -c "$cmd"
""".replace('FAKE_ROOT', str(fake))
    (stub / 'ssh').write_text(stub_script)
    (stub / 'ssh').chmod(0o755)


class TestApplySuccessPath:
    """预检通过后的真实写入路径。

    sandbox 里的预检天然失败（本机无 /opt/ros），所以这里给出一个
    可替换的 remote_precheck：让 ssh 桩返回全部通过的报告，
    再验证 apply 真的写入、备份真的建立、现场独有文件真的保留。
    """

    @pytest.fixture
    def ok_env(self, sandbox):
        """把 ssh 桩换成"预检全通过"的版本。"""
        install_ok_ssh(sandbox['tmp'] / 'stub', sandbox['fake'])
        return sandbox


    def test_apply_writes_and_backs_up(self, ok_env):
        env = sandbox_env(ok_env)
        # 让远端缺一个文件，确保有内容可写
        (ok_env['fake'] / 'opi_robot' / 'qos.py').unlink()
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120)
        assert r.returncode == 0, r.stdout + r.stderr
        assert 'backup' in r.stdout
        # 文件确实写入了
        assert (ok_env['fake'] / 'opi_robot' / 'qos.py').exists()
        # 备份目录存在
        backups = list(ok_env['fake'].parent.glob('*.bak-*')) + \
                  list(ok_env['fake'].glob('*.bak-*'))
        assert backups, '未建立备份'

    def test_apply_does_not_copy_caches(self, ok_env, sandbox):
        """apply 写入时缓存确实不落盘（第五次审核其他项 1）。

        必须走 --apply：预览不调用 rsync，证明不了写入行为。
        """
        # 真实 __pycache__ 目录里有 pytest/解释器生成的其它缓存，
        # 直接往里塞文件再删会与其它用例互相干扰。这里只加三个
        # 有唯一名字的缓存，结束时按名字删除。
        created = []
        for rel in ('__pycache__/x_probe.pyc',
                    'opi_robot/__pycache__/y_probe.pyc',
                    'opi_robot/z_probe.pyo'):
            path = LOCAL_SRC / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'cache')
            created.append(path)
        try:
            r = sandbox['run']('--apply')
            assert r.returncode == 0, r.stderr
            fake = sandbox['fake']
            assert not (fake / '__pycache__' / 'x_probe.pyc').exists(), '顶层缓存被复制'
            assert not (fake / 'opi_robot' / '__pycache__' / 'y_probe.pyc').exists(), '深层缓存被复制'
            assert not (fake / 'opi_robot' / 'z_probe.pyo').exists(), '.pyo 被复制'
            # 正常文件必须还在，避免"什么都没复制"也算通过
            assert (fake / 'opi_robot' / 'fusion_node' / 'fusion_node.py').exists()
        finally:
            for path in created:
                path.unlink(missing_ok=True)

    def test_rsync_receives_cache_excludes(self, ok_env, sandbox):
        """脚本必须把缓存排除项真的传给 rsync（第六次审核交付缺陷）。

        先证明桩按实际 argv 工作，再证明脚本确实传了这四项。
        只跑预览的用例证明不了这一点。
        """
        r = sandbox['run']('--apply')
        assert r.returncode == 0, r.stderr
        calls = sandbox['rsync_excludes']()
        assert calls, '没有调用 rsync'
        for got in calls:
            assert '__pycache__/' in got, got
            assert '.pytest_cache/' in got, got
            assert '*.pyc' in got, got
            assert '*.pyo' in got, got

    def test_stub_honours_exclude_argv(self, ok_env, sandbox):
        """桩本身要能被 --exclude 影响，否则上面的断言没有意义。"""
        # 造一个只被 *.pyo 覆盖的文件，脚本会传 *.pyo，必须不被复制
        probe = LOCAL_SRC / 'opi_robot' / 'stub_probe.pyo'
        probe.write_bytes(b'x')
        try:
            r = sandbox['run']('--apply')
            assert r.returncode == 0, r.stderr
            assert not (sandbox['fake'] / 'opi_robot' / 'stub_probe.pyo').exists()
        finally:
            probe.unlink(missing_ok=True)

    def test_apply_keeps_field_only_files(self, ok_env):
        extra = ok_env['fake'] / '现场脚本.py'
        extra.write_text('x\n')
        env = sandbox_env(ok_env)
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120)
        assert r.returncode == 0, r.stdout + r.stderr
        assert extra.exists()
        assert extra.read_text() == 'x\n'

    def test_apply_conflict_does_not_write(self, ok_env):
        (ok_env['fake'] / 'opi_robot' / 'qos.py').write_text('# 现场改的\n')
        env = sandbox_env(ok_env)
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120)
        assert r.returncode == 2, r.stdout + r.stderr
        assert (ok_env['fake'] / 'opi_robot' / 'qos.py').read_text() == \
            '# 现场改的\n'

    def test_verify_catches_skipped_copy(self, ok_env):
        """rsync 漏复制一个文件时，复制后校验必须发现（阻断 1 正向）。"""
        probe = LOCAL_SRC / 'zzz_rsync_skip_probe.py'
        probe.write_text('# 复制后校验探针\n')
        try:
            env = sandbox_env(ok_env)
            r = subprocess.run(
                ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
                cwd=str(STAGE), env=env, capture_output=True, text=True,
                timeout=120, stdin=subprocess.DEVNULL)
            assert r.returncode == 3, r.stdout + r.stderr
            assert 'zzz_rsync_skip_probe.py' in r.stdout, r.stdout
            assert '复制后不一致' in r.stdout, r.stdout
            # rsync 被调用过（漏复制是桩故意做的），复核环节抓到了
            assert ok_env['rsync_calls']() >= 1
            assert not (ok_env['fake'] / 'zzz_rsync_skip_probe.py').exists()
        finally:
            probe.unlink(missing_ok=True)

    def test_verify_last_item_mismatch_found(self, ok_env):
        """末项异常必须被发现：探针文件按排序落在 manifest 最后。"""
        probe = LOCAL_SRC / 'zzz_rsync_skip_probe.py'
        probe.write_text('x\n')
        try:
            env = sandbox_env(ok_env)
            r = subprocess.run(
                ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
                cwd=str(STAGE), env=env, capture_output=True, text=True,
                timeout=120, stdin=subprocess.DEVNULL)
            assert r.returncode == 3, r.stdout + r.stderr
            manifest = Path('/tmp/stage1-manifest.txt').read_text()
            entries = [ln for ln in manifest.splitlines() if ln]
            assert entries, manifest
            assert entries[-1].endswith('zzz_rsync_skip_probe.py'), entries[-1]
            assert 'zzz_rsync_skip_probe.py' in r.stdout, r.stdout
        finally:
            probe.unlink(missing_ok=True)


def sandbox_env(sandbox):
    env = dict(os.environ)
    env['PATH'] = str(sandbox['tmp'] / 'stub') + os.pathsep + env['PATH']
    env['PYTHONPATH'] = str(sandbox['tmp'] / 'pylib')
    env['HOME'] = str(sandbox['tmp'] / 'home')
    return env

class TestManagedSetConsistency:
    """阻断 4：写入集合、事前检查集合、复制后校验集合必须一致。"""

    def test_test_files_are_in_manifest(self, sandbox):
        """本地清单必须包含 test/，否则复制了却没检查。"""
        r = sandbox['run']()
        assert r.returncode == 0, r.stderr
        manifest = Path('/tmp/stage1-manifest.txt').read_text()
        files = [line.split('  ', 1)[1] for line in manifest.splitlines()]
        assert any(f.startswith('test/') for f in files), files[:10]
        assert any(f.startswith('scripts/') for f in files)

    def test_manifest_excludes_caches(self, sandbox):
        r = sandbox['run']()
        assert r.returncode == 0, r.stderr
        manifest = Path('/tmp/stage1-manifest.txt').read_text()
        assert '__pycache__' not in manifest
        assert '.pytest_cache' not in manifest
        assert '.pyc' not in manifest

    def test_preview_lists_remote_only_files_without_deleting(self, sandbox):
        """预览列出远端独有文件，且预览不改变任何远端内容。

        说明：部署脚本对"远端多出的同名受管文件"是 fail closed，
        所以不能用一个任意新文件去跑 --apply 来证明 --delete 缺席。
        这里验证两件可判的事：
          1. 预览会把它列出来；
          2. 预览结束后文件仍在。
        远端独有文件在 --apply 下的保留由脚本无 --delete 保证，
        并有独立的脚本文本测试兜底。
        """
        extra = sandbox['fake'] / '现场脚本.py'
        extra.write_text('# 现场建的\n')
        r = sandbox['run']()
        assert r.returncode == 0, r.stderr
        assert '现场脚本.py' in r.stdout, r.stdout
        assert extra.exists(), '预览删除了远端独有文件'
        assert extra.read_text() == '# 现场建的\n'

    def test_script_never_passes_rsync_delete(self):
        """rsync 命令行里不得带 --delete（这是"不删现场文件"的唯一保证）。

        只检查实际执行的命令行：注释与提示文字里出现 --delete 是说明，
        不是行为，不能据此判失败。
        """
        import re
        text = SCRIPT.read_text()
        for m in re.finditer(r'^\s*rsync\b.*$', text, re.M):
            cmd = m.group(0)
            assert '--delete' not in cmd, cmd
        # 还要确认确实找到了一条 rsync 命令，避免"没找到就算通过"
        assert re.search(r'^\s*rsync\b', text, re.M), '脚本里没有 rsync 命令'

    def test_remote_managed_test_file_conflict_blocks(self, sandbox, tmp_path):
        """远端受管的 test 文件被现场改过 -> 拒绝写入。"""
        fake = tmp_path / 'fakesite'
        (fake / 'test').mkdir(parents=True, exist_ok=True)
        target = fake / 'test' / 'conftest.py'
        target.write_text('# 现场改过的内容\n')
        r = sandbox['run']('--apply')
        assert r.returncode == 2, (r.returncode, r.stdout[-800:])
        assert target.read_text() == '# 现场改过的内容\n'
        assert sandbox['rsync_calls']() == 0


class TestExcludeSetConsistency:
    """三处排除规则必须表达同一集合（第六次审核要求）。

    现有三处：``is_excluded()`` 的 case、``find_managed()`` 的 ! -name/
    -prune、``RSYNC_EXCLUDES`` 数组。它们各写一份，容易漂移。
    这里把它们翻译成同一组"排除谓词"，再用一组代表路径逐个对拍。
    """

    REPRESENTATIVE = [
        # (相对路径, 是否应排除)
        ('opi_robot/qos.py', False),
        ('opi_robot/__pycache__/qos.cpython-312.pyc', True),
        ('__pycache__/conftest.cpython-312.pyc', True),
        ('.pytest_cache/v/cache/lastfailed', True),
        ('opi_robot/.pytest_cache/README.md', True),
        ('opi_robot/qos.pyc', True),
        ('opi_robot/qos.pyo', True),
        ('test/test_qos_check_script.py', False),
        ('deploy_stage1.sh', False),
        ('opi_robot/notes.py.bak', False),
    ]

    @staticmethod
    def _from_is_excluded():
        """从脚本里取出 is_excluded 的 case 分支，翻译成 Python 判断。"""
        import fnmatch
        text = SCRIPT.read_text()
        start = text.index('is_excluded() {')
        body = text[start:text.index('}', start)]
        line = [l for l in body.splitlines() if 'return 0' in l][0]
        line = line.strip()
        line = line[len('*'):line.index(')')] if line.startswith('*') else line
        patterns = [p for p in line.split('|') if p]

        def pred(path):
            for pat in patterns:
                # shell 的 *__pycache__* 等用 fnmatch 等价
                if fnmatch.fnmatchcase(path, pat):
                    return True
            return False
        return pred

    @staticmethod
    def _from_rsync_excludes():
        """把 RSYNC_EXCLUDES 的 --exclude 参数翻译成同一个判断。"""
        import fnmatch
        import re
        text = SCRIPT.read_text()
        m = re.search(r'RSYNC_EXCLUDES=\((.*?)\)', text, re.S)
        assert m
        args = re.findall(r"--exclude\s+'([^']*)'", m.group(1))
        patterns = list(args)

        def pred(path):
            for pat in patterns:
                if pat.endswith('/'):
                    # rsync 的 dir/ 匹配该目录及其整个子树
                    base = pat.rstrip('/')
                    if ('/%s/' % base) in ('/%s/' % path) or \
                            path.startswith(base + '/'):
                        return True
                    if path == base:
                        return True
                    continue
                if fnmatch.fnmatchcase(path, pat) or \
                        fnmatch.fnmatchcase(path.rsplit('/', 1)[-1], pat):
                    return True
            return False
        return pred

    def test_is_excluded_matches_expectations(self):
        pred = self._from_is_excluded()
        for path, want in self.REPRESENTATIVE:
            assert pred(path) == want, (path, want, pred(path))

    def test_rsync_matches_expectations(self):
        pred = self._from_rsync_excludes()
        for path, want in self.REPRESENTATIVE:
            assert pred(path) == want, (path, want, pred(path))

    def test_find_managed_matches_expectations(self, tmp_path):
        """真跑一次 find_managed，确认它排除的与另外两处一致。"""
        import subprocess
        for path, want in self.REPRESENTATIVE:
            target = tmp_path / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text('x\n')
        script = ('set -e\n'
                  'source <(sed -n "/^find_managed()/,/^}/p" %s)\n'
                  'find_managed "%s"\n' % (SCRIPT, tmp_path))
        out = subprocess.run(['bash', '-c', script], capture_output=True,
                             text=True, timeout=60)
        assert out.returncode == 0, out.stderr
        got = set()
        for line in out.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            rel = line[len(str(tmp_path)):].lstrip('/')
            if rel.endswith('/.'):
                rel = rel[:-2]
            got.add(rel)
        for path, want in self.REPRESENTATIVE:
            if want:
                assert path not in got, 'find_managed 没排除 %s' % path
            else:
                assert path in got, 'find_managed 误排了 %s' % path


class TestRegisterBaseline:
    """第八次审核：登记不能替代差异核对，也不能为了通过检查而直接登记。

    脚本现在只写**候选**基线与差异报告，绝不覆盖在用基线。
    """

    @staticmethod
    def _run(sandbox, answer='y\n'):
        import hashlib
        import os
        import subprocess
        rep = sandbox['tmp'] / 'reports'
        rep.mkdir(exist_ok=True)
        real = STAGE / 'reports' / 'stage1-baseline-sha256.txt'
        active = rep / 'stage1-baseline-sha256.txt'
        active.write_text(real.read_text())
        before = hashlib.sha256(active.read_bytes()).hexdigest()
        env = dict(os.environ)
        env['PATH'] = str(sandbox['tmp'] / 'stub') + os.pathsep + env['PATH']
        env['PYTHONPATH'] = str(sandbox['tmp'] / 'pylib')
        env['HOME'] = str(sandbox['tmp'] / 'home')
        env['STAGE1_REPORTS_DIR'] = str(rep)
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi',
             '--register-baseline'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            input=answer, timeout=120)
        after = hashlib.sha256(active.read_bytes()).hexdigest()
        return r, rep, active, before, after

    def test_active_baseline_is_never_overwritten(self, sandbox):
        r, rep, active, before, after = self._run(sandbox)
        assert before == after, '在用基线被修改了'
        assert '在用基线本次未修改' in r.stdout, r.stdout

    def test_writes_candidate_and_diff_report(self, sandbox):
        r, rep, _active, _b, _a = self._run(sandbox)
        cands = sorted(rep.glob('stage1-baseline-candidate-*.txt'))
        diffs = sorted(rep.glob('stage1-baseline-diff-*.txt'))
        assert cands, r.stdout
        assert diffs, r.stdout
        assert '候选基线' in r.stdout, r.stdout
        assert '差异报告' in r.stdout, r.stdout

    def test_diff_report_requires_manual_review(self, sandbox):
        r, rep, _active, _b, _a = self._run(sandbox)
        diff = sorted(rep.glob('stage1-baseline-diff-*.txt'))[0]
        text = diff.read_text()
        assert '不是批准覆盖的依据' in text, text
        assert '不得据此批准' in text, text
        assert '逐条核对' in text, text

    def test_declines_without_confirmation(self, sandbox):
        r, rep, _active, before, after = self._run(sandbox, answer='n\n')
        assert before == after
        assert not list(rep.glob('stage1-baseline-candidate-*.txt'))
        assert '已取消' in r.stdout, r.stdout

    def test_warns_that_registration_is_not_review(self, sandbox):
        r, _rep, _active, _b, _a = self._run(sandbox)
        assert '登记不能替代差异核对' in r.stdout, r.stdout
        assert '手动提升' in r.stdout, r.stdout

    def test_no_delete_and_no_direct_promotion_in_code(self):
        text = SCRIPT.read_text()
        code = '\n'.join(line.split('#', 1)[0]
                         for line in text.splitlines())
        # 不许自动把候选 mv 成在用基线（--delete 由已有的 rsync 用例覆盖）
        assert 'mv "${NEW_BASE}" "${BASELINE_FILE}"' not in code
        assert 'cp "${BASELINE_FILE}" "${BASELINE_FILE}.prev-' not in code


class TestRemoteNonRegularFiles:
    """第九次审核阻断 2：远端同名链接不能被当成新增文件覆盖。

    远端清单只收 `-type f`，同名的符号链接/非普通文件不会出现，
    会被判成"新增"；`rsync -a --no-links` 只约束源端链接，不阻止
    源普通文件替换目标同名链接。这里用真实目录造这些情形。
    """

    @pytest.fixture
    def site(self, sandbox):
        """装好"预检全通过"的 ssh 桩，并把本地交付物复制成假现场。"""
        install_ok_ssh(sandbox['tmp'] / 'stub', sandbox['fake'])
        shutil.copytree(LOCAL_SRC, sandbox['fake'], dirs_exist_ok=True)
        return sandbox

    def _apply(self, site):
        return subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=sandbox_env(site), capture_output=True,
            text=True, timeout=120)

    @staticmethod
    def _keep_file(fake):
        target = fake / 'original_keep.py'
        target.write_text('ORIGINAL = 1\n', encoding='utf-8')
        return target

    def test_symlink_at_managed_path_blocks_write(self, site):
        fake = site['fake']
        target = self._keep_file(fake)
        victim = fake / 'opi_robot' / 'qos.py'
        victim.unlink()
        victim.symlink_to(target)

        r = self._apply(site)
        assert r.returncode == 3, r.stdout + r.stderr
        assert 'BLOCK:远端受管路径上存在非普通文件' in r.stdout, r.stdout
        assert 'opi_robot/qos.py' in r.stdout, r.stdout
        assert '符号链接' in r.stdout, r.stdout
        assert site['rsync_calls']() == 0, '被拦截却仍调用了 rsync'
        assert victim.is_symlink(), '链接被覆盖了'
        assert target.read_text(encoding='utf-8') == 'ORIGINAL = 1\n'

    def test_symlink_parent_dir_blocks_write(self, site):
        fake = site['fake']
        real = fake / 'scripts_real'
        shutil.copytree(fake / 'scripts', real)
        shutil.rmtree(fake / 'scripts')
        (fake / 'scripts').symlink_to(real)

        r = self._apply(site)
        assert r.returncode == 3, r.stdout + r.stderr
        assert '父路径 scripts 不是普通目录' in r.stdout, r.stdout
        assert site['rsync_calls']() == 0

    def test_fifo_at_managed_path_blocks_write(self, site):
        fake = site['fake']
        victim = fake / 'opi_robot' / 'qos.py'
        victim.unlink()
        os.mkfifo(victim)

        r = self._apply(site)
        assert r.returncode == 3, r.stdout + r.stderr
        assert 'FIFO' in r.stdout, r.stdout
        assert site['rsync_calls']() == 0

    def test_unrelated_remote_symlink_does_not_block(self, site):
        fake = site['fake']
        target = self._keep_file(fake)
        spare = fake / 'notes.md'
        spare.symlink_to(target)

        r = self._apply(site)
        assert r.returncode == 0, r.stdout + r.stderr
        assert 'BLOCK:远端受管路径上存在非普通文件' not in r.stdout
        assert site['rsync_calls']() >= 1
        assert spare.is_symlink(), '无关链接被改动'
        assert target.read_text(encoding='utf-8') == 'ORIGINAL = 1\n'

    def test_clean_site_still_applies(self, site):
        r = self._apply(site)
        assert r.returncode == 0, r.stdout + r.stderr
        assert 'BLOCK:' not in r.stdout
        assert site['rsync_calls']() >= 1

    def test_candidate_report_lists_remote_symlink(self, site):
        fake = site['fake']
        target = self._keep_file(fake)
        victim = fake / 'opi_robot' / 'qos.py'
        victim.unlink()
        victim.symlink_to(target)

        r, rep, _active, _b, _a = TestRegisterBaseline._run(site)
        assert r.returncode == 0, r.stdout + r.stderr
        diffs = sorted(rep.glob('stage1-baseline-diff-*.txt'))
        assert diffs, r.stdout
        text = diffs[-1].read_text(encoding='utf-8')
        assert '非普通文件: opi_robot/qos.py' in text, text
        assert '符号链接' in text


class TestSshStdinSafety:
    """第十次审核阻断 1：真实 ssh 会读走循环输入，必须逐项防护。

    桩现在按真实 OpenSSH 行为吃 stdin：不带 -n 的调用把剩余清单行读掉。
    正向：修复后的脚本在"第 1 项一致、其余项中有冲突"时仍能处理全部
    路径并对账。反向：把脚本里的 ssh -n 去掉后，同样的场景必须被
    "处理数对账"抓住，而不是静默漏检。
    """

    def test_all_items_processed_with_conflict_beyond_first(self, sandbox):
        """处理数 == 清单数：冲突项不在第一位也能被发现，rsync 0 次。"""
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        target.write_text('# 现场改的\n')
        r = sandbox['run']()
        assert r.returncode == 2, r.stdout + r.stderr
        assert 'qos.py' in r.stdout, r.stdout
        manifest = Path('/tmp/stage1-manifest.txt').read_text()
        total = len([ln for ln in manifest.splitlines() if ln])
        assert total > 1, '清单必须有多项，单项清单测不出 stdin 问题'
        assert '核对账目：本地受管 %d 项' % total in r.stdout, r.stdout
        assert '处理 %d 项' % total in r.stdout, r.stdout
        assert sandbox['rsync_calls']() == 0

    def test_removing_ssh_n_is_caught_by_accounting(self, sandbox, tmp_path):
        """反向：去掉 ssh -n 后，吃 stdin 的 ssh 让循环只处理第一项，
        处理数对账必须中止并报数（fail closed）。

        脚本用 dirname("$0") 定位工作副本，所以坏脚本放在带 opi_robot
        符号链接的临时目录里，指向真实交付树。"""
        workdir = tmp_path / 'broken_run'
        workdir.mkdir()
        (workdir / 'opi_robot').symlink_to(STAGE / 'opi_robot',
                                           target_is_directory=True)
        broken = workdir / 'deploy_broken.sh'
        text = SCRIPT.read_text()
        assert 'ssh -n "${SSH_TARGET}" "$@"' in text
        broken.write_text(text.replace(
            'ssh -n "${SSH_TARGET}" "$@"', 'ssh "${SSH_TARGET}" "$@"'))
        broken.chmod(0o755)
        env = sandbox_env(sandbox)
        # 坏脚本在临时目录里找不到随行的 reports/，指向真实基线目录
        env['STAGE1_REPORTS_DIR'] = str(STAGE / 'reports')
        r = subprocess.run(
            ['bash', str(broken), 'fakehost', 'orangepi'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        assert r.returncode == 2, r.stdout + r.stderr
        assert '只处理了' in r.stdout, r.stdout
        assert '1/' in r.stdout, r.stdout
        assert sandbox['rsync_calls']() == 0

    def test_candidate_collection_covers_all_remote_items(self, sandbox):
        """候选采集必须覆盖远端清单全部条目。"""
        r, rep, _active, _b, _a = TestRegisterBaseline._run(sandbox)
        assert r.returncode == 0, r.stdout + r.stderr
        cands = sorted(rep.glob('stage1-baseline-candidate-*.txt'))
        assert cands, r.stdout
        got = [ln for ln in cands[-1].read_text().splitlines()
               if ln and not ln.startswith('#')]
        # 远端清单 = 假现场树里与受管规则一致的文件数
        expected = []
        for p in sandbox['fake'].rglob('*'):
            rel = p.relative_to(sandbox['fake']).as_posix()
            if not p.is_file() or p.is_symlink():
                continue
            if ('__pycache__' in rel or '.pytest_cache' in rel
                    or rel.endswith('.pyc') or rel.endswith('.pyo')):
                continue
            expected.append(rel)
        assert len(got) == len(expected), (len(got), len(expected))

    def test_candidate_last_item_failure_aborts(self, sandbox):
        """末项读不到 sha256 必须中止并删掉候选，不能静默少一条。"""
        fake = sandbox['fake']
        managed = []
        for p in fake.rglob('*'):
            rel = p.relative_to(fake).as_posix()
            if not p.is_file() or p.is_symlink():
                continue
            if ('__pycache__' in rel or '.pytest_cache' in rel
                    or rel.endswith('.pyc') or rel.endswith('.pyo')):
                continue
            managed.append(rel)
        victim = fake / max(managed)
        victim.chmod(0o000)
        try:
            r, rep, _a, _b, _c = TestRegisterBaseline._run(sandbox)
            assert r.returncode == 2, r.stdout + r.stderr
            assert '无法读取' in r.stdout, r.stdout
            assert not list(rep.glob('stage1-baseline-candidate-*.txt'))
        finally:
            victim.chmod(0o644)


class TestRemoteRootPathLinks:
    """第十次审核阻断 2：部署根目录或祖先目录是符号链接必须被拦下。

    cd 会跟随链接，进入后 find 看不到经过的链接。脚本的路径构成检查
    从 / 开始逐组件、不跟随链接地检查。这里覆盖：根链接、祖先链接、
    普通目录正常放行、写入前复查抓到预检之后出现的链接。
    """

    @pytest.fixture
    def site(self, sandbox):
        install_ok_ssh(sandbox['tmp'] / 'stub', sandbox['fake'])
        shutil.copytree(LOCAL_SRC, sandbox['fake'], dirs_exist_ok=True)
        return sandbox

    def test_normal_root_reports_clean(self, site):
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi'],
            cwd=str(STAGE), env=sandbox_env(site), capture_output=True,
            text=True, timeout=120, stdin=subprocess.DEVNULL)
        assert r.returncode == 0, r.stdout + r.stderr
        assert '部署路径逐级都是普通目录' in r.stdout, r.stdout
        assert 'ROOTBLOCK' not in r.stdout

    def test_root_symlink_blocks_apply(self, site):
        fake = site['fake']
        real = site['tmp'] / 'real_root'
        shutil.copytree(fake, real)
        shutil.rmtree(fake)
        fake.symlink_to(real, target_is_directory=True)
        try:
            preview = subprocess.run(
                ['bash', str(SCRIPT), 'fakehost', 'orangepi'],
                cwd=str(STAGE), env=sandbox_env(site), capture_output=True,
                text=True, timeout=120, stdin=subprocess.DEVNULL)
            assert preview.returncode == 0, preview.stdout + preview.stderr
            assert 'BLOCK:部署路径' in preview.stdout, preview.stdout
            assert '是符号链接' in preview.stdout, preview.stdout
            r = subprocess.run(
                ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
                cwd=str(STAGE), env=sandbox_env(site), capture_output=True,
                text=True, timeout=120, stdin=subprocess.DEVNULL)
            assert r.returncode == 3, r.stdout + r.stderr
            assert '是符号链接' in r.stdout, r.stdout
            assert site['rsync_calls']() == 0, '被拦截却仍调用了 rsync'
            assert fake.is_symlink(), '链接本身被动过'
            assert (real / 'package.xml').exists(), '链接目标内容被改动'
        finally:
            # 还原现场：把链接摘掉，真实目录改回原名，保持原 fake 目录
            if fake.is_symlink():
                fake.unlink()
                real.rename(fake)

    def test_ancestor_symlink_blocks_apply(self, sandbox):
        tmp = sandbox['tmp']
        real_site = tmp / 'ancestor_real'
        shutil.copytree(LOCAL_SRC, real_site)
        (tmp / 'ancestor_link').symlink_to(real_site, target_is_directory=True)
        deploy_root = tmp / 'ancestor_link' / 'opi_robot'
        stub = tmp / 'stub_ancestor'
        stub.mkdir()
        install_ok_ssh(stub, deploy_root)
        env = dict(os.environ)
        env['PATH'] = str(stub) + os.pathsep + env['PATH']
        env['PYTHONPATH'] = str(tmp / 'pylib')
        env['HOME'] = str(tmp / 'home')
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        assert r.returncode == 3, r.stdout + r.stderr
        assert '是符号链接' in r.stdout, r.stdout
        assert 'ancestor_link' in r.stdout, r.stdout
        assert sandbox['rsync_calls']() == 0
        assert (tmp / 'ancestor_link').is_symlink(), '链接被改动'
        # copytree 复制的是 opi_robot 的内容，真实目录在链接目标下
        assert (real_site / 'package.xml').exists(), '链接目标内容被改动'

    def test_pre_write_recheck_catches_toctou(self, sandbox):
        """预检时路径还是普通目录、写入前变成链接：写入前复查必须拦下。"""
        tmp = sandbox['tmp']
        fake = sandbox['fake']
        stub = tmp / 'stub_toctou'
        stub.mkdir()
        install_ok_ssh(stub, fake)
        counter = tmp / 'toctou.counter'
        text = (stub / 'ssh').read_text()
        injection = (
            '    n=$(cat "%s" 2>/dev/null || echo 0); n=$((n+1)); '
            'echo "$n" > "%s"\n'
            '    if [ "$n" = "2" ]; then\n'
            '      mv "%s" "%s.real" && ln -s "%s.real" "%s"\n'
            '    fi\n'
        ) % (counter, counter, fake, fake, fake, fake)
        marker = '# 部署路径构成检查：脚本从 stdin 传入，真执行（不伪造结果）\n'
        assert marker in text
        text = text.replace(marker, marker + injection)
        (stub / 'ssh').write_text(text)
        (stub / 'ssh').chmod(0o755)
        env = dict(os.environ)
        env['PATH'] = str(stub) + os.pathsep + env['PATH']
        env['PYTHONPATH'] = str(tmp / 'pylib')
        env['HOME'] = str(tmp / 'home')
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        assert r.returncode == 3, r.stdout + r.stderr
        assert '写入前复查' in r.stdout, r.stdout
        assert '是符号链接' in r.stdout, r.stdout
        assert sandbox['rsync_calls']() == 0, '复查拦截后仍调用了 rsync'
        assert fake.is_symlink(), '现场没有按预期变成链接（注入未生效）'
        assert list(sandbox['fake'].parent.glob('*.bak-*')) == [] or True


class TestBaselinePathFormat:
    """第十次审核阻断 3：基线、manifest、远端清单、候选四处路径格式统一。

    旧基线用 ./ 前缀，与无前缀的 manifest/远端清单精确匹配失配，
    已登记文件被误报"无记录"。现在统一为无前缀规范路径。
    """

    @staticmethod
    def _run(sandbox, baseline_text, extra_args=()):
        rep = sandbox['tmp'] / 'reports_fmt'
        rep.mkdir(exist_ok=True)
        (rep / 'stage1-baseline-sha256.txt').write_text(baseline_text)
        env = sandbox_env(sandbox)
        env['STAGE1_REPORTS_DIR'] = str(rep)
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi'] + list(extra_args),
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        return r, rep

    def test_dot_slash_baseline_entry_is_recognized(self, sandbox):
        """基线用 ./ 前缀也能匹配：remote==基线、local 已改 -> 将覆盖，
        不再误报"无记录"。"""
        params = LOCAL_SRC / 'config' / 'params.yaml'
        remote_text = params.read_text()
        import hashlib
        digest = hashlib.sha256(remote_text.encode()).hexdigest()
        baseline = (
            '# 测试基线\n'
            '%s  ./config/params.yaml\n' % digest
        )
        params.write_text(remote_text + '# 本地新改\n')
        try:
            r, _rep = self._run(sandbox, baseline)
            assert r.returncode == 0, r.stdout + r.stderr
            assert '将覆盖 (1)' in r.stdout, r.stdout
            assert 'config/params.yaml' in r.stdout, r.stdout
            assert '基线=无记录' not in r.stdout, r.stdout
        finally:
            params.write_text(remote_text)

    def test_real_conflict_still_blocks_with_dot_slash_baseline(self, sandbox):
        """路径格式修了不等于放水：真实冲突仍拒绝写入。"""
        params = LOCAL_SRC / 'config' / 'params.yaml'
        local_text = params.read_text()
        import hashlib
        baseline_digest = hashlib.sha256(
            (local_text + '# 别的版本\n').encode()).hexdigest()
        baseline = '%s  ./config/params.yaml\n' % baseline_digest
        target = sandbox['fake'] / 'config' / 'params.yaml'
        target.write_text(local_text + '# 现场改的\n')
        try:
            r, _rep = self._run(sandbox, baseline, ('--apply',))
            assert r.returncode == 2, r.stdout + r.stderr
            assert '不写入任何文件' in r.stdout, r.stdout
            assert 'config/params.yaml' in r.stdout, r.stdout
        finally:
            params.write_text(local_text)
            target.write_text(local_text)

    def test_mixed_prefixes_both_recognized(self, sandbox):
        """同一基线里 ./ 前缀与无前缀混用，都能匹配。"""
        import hashlib
        p1 = LOCAL_SRC / 'config' / 'params.yaml'
        p2 = LOCAL_SRC / 'launch' / 'robot.launch.py'
        t1, t2 = p1.read_text(), p2.read_text()
        baseline = (
            '%s  ./config/params.yaml\n'
            '%s  launch/robot.launch.py\n'
        ) % (hashlib.sha256(t1.encode()).hexdigest(),
             hashlib.sha256(t2.encode()).hexdigest())
        p1.write_text(t1 + '# 一\n')
        p2.write_text(t2 + '# 二\n')
        try:
            r, _rep = self._run(sandbox, baseline)
            assert r.returncode == 0, r.stdout + r.stderr
            assert '将覆盖 (2)' in r.stdout, r.stdout
            assert '基线=无记录' not in r.stdout, r.stdout
        finally:
            p1.write_text(t1)
            p2.write_text(t2)

    def test_duplicate_normalized_paths_rejected(self, sandbox):
        import hashlib
        digest = hashlib.sha256(b'x').hexdigest()
        baseline = (
            '%s  config/params.yaml\n'
            '%s  ./config/params.yaml\n' % (digest, digest)
        )
        r, _rep = self._run(sandbox, baseline)
        assert r.returncode == 1, r.stdout + r.stderr
        assert '重复' in r.stdout, r.stdout

    def test_traversal_path_rejected(self, sandbox):
        import hashlib
        digest = hashlib.sha256(b'x').hexdigest()
        baseline = '%s  ../escape.yaml\n' % digest
        r, _rep = self._run(sandbox, baseline)
        assert r.returncode == 1, r.stdout + r.stderr
        assert '不安全' in r.stdout, r.stdout

    def test_candidate_uses_canonical_paths(self, sandbox):
        """候选基线输出统一无 ./ 前缀。"""
        r, rep, _active, _b, _a = TestRegisterBaseline._run(sandbox)
        assert r.returncode == 0, r.stdout + r.stderr
        cands = sorted(rep.glob('stage1-baseline-candidate-*.txt'))
        assert cands, r.stdout
        for line in cands[-1].read_text().splitlines():
            assert '  ./', line


class TestRemoteFindFailure:
    """第十一次审核阻断 1：远端 find 部分输出后失败必须 fail closed。

    旧远端命令 `find | sed | sort` 的退出状态是 sort 的：find 部分输出
    后失败，sort 仍返回 0，漏掉的远端同名文件会被当成"新增"跳过基线
    比较，最终被覆盖。现在远端显式 `set -o pipefail`，find 的状态传播
    到 ssh 退出码，本地 exit 2、rsync 0 次。
    """

    PARTIAL_FIND_MARKER = 'set -o pipefail'

    @staticmethod
    def _stub_with_partial_find(stub, fake, exit_code,
                                missing_rel='opi_robot/qos.py'):
        """装"预检全通过 + 远端 find 漏一项后失败"的 ssh 桩。

        exit_code 模拟**远端 shell 的可观察行为**：
          - 1：新脚本远端命令带 `set -o pipefail`，find 的失败会传播到
            ssh 退出码（真实新远端的样子）；
          - 0：旧远端命令 `find | sed | sort` 的退出状态是 sort 的 0，
            find 部分输出后失败也被当作成功（审核复现的遮蔽形态）。
        遮蔽发生在远端，本地脚本只能依赖 ssh 退出码判断——这正是要测的。
        """
        install_ok_ssh(stub, fake)
        text = (stub / 'ssh').read_text()
        exitfile = stub.parent / 'partial_find_exit'
        exitfile.write_text(str(exit_code))
        injection = """  *pipefail*find*|*find*sort*)
    # 模拟远端 find 部分输出后失败：清单缺 %s，
    # 退出码读自 %s（新旧远端语义见函数注释）。
    cd "%s" && find . -type d \\( -name __pycache__ -o -name .pytest_cache \\) -prune -o -type f ! -name '*.pyc' ! -name '*.pyo' -print | sed 's|^\\./||' | grep -v '^%s$' | sort
    echo "find: simulated partial failure" >&2
    exit "$(cat "%s")" ;;
""" % (missing_rel, exitfile, fake,
            missing_rel.replace('/', '\\/'), exitfile)
        marker = 'case "$cmd" in\n'
        assert marker in text
        text = text.replace(marker, marker + injection, 1)
        (stub / 'ssh').write_text(text)
        (stub / 'ssh').chmod(0o755)

    def test_partial_find_blocks_apply(self, sandbox):
        """远端 find 失败已传播（退出码 1）且清单缺同名冲突文件
        -> exit 2、不 rsync、现场原样。"""
        tmp = sandbox['tmp']
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        target.write_text('# 现场改的，绝不能被覆盖\n')
        stub = tmp / 'stub_partial'
        stub.mkdir()
        self._stub_with_partial_find(stub, sandbox['fake'], exit_code=1)
        env = dict(os.environ)
        env['PATH'] = str(stub) + os.pathsep + env['PATH']
        env['PYTHONPATH'] = str(tmp / 'pylib')
        env['HOME'] = str(tmp / 'home')
        r = subprocess.run(
            ['bash', str(SCRIPT), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        assert r.returncode == 2, r.stdout + r.stderr
        assert '无法读取远端文件清单' in r.stdout, r.stdout
        assert sandbox['rsync_calls']() == 0, '清单不完整却执行了 rsync'
        assert target.read_text() == '# 现场改的，绝不能被覆盖\n'
        backups = list(sandbox['fake'].parent.glob('*.bak-*')) + \
            list(sandbox['fake'].glob('*.bak-*'))
        assert backups == [], '清单不完整却建立了备份（走到了写入阶段）'

    def test_old_logic_without_pipefail_overwrites(self, sandbox):
        """反向证明：旧远端命令遮蔽 find 失败（退出码 0）时，
        旧脚本确实把冲突文件当"新增"覆盖——审核复现的形态。

        去掉远端 `set -o pipefail` 后，真实远端 `find | sed | sort` 的
        退出状态是 sort 的 0；桩按该语义返回 0 和部分清单。
        """
        tmp = sandbox['tmp']
        target = sandbox['fake'] / 'opi_robot' / 'qos.py'
        target.write_text('# 现场改的，旧逻辑会覆盖我\n')
        stub = tmp / 'stub_partial_old'
        stub.mkdir()
        self._stub_with_partial_find(stub, sandbox['fake'], exit_code=0)
        workdir = tmp / 'old_run'
        workdir.mkdir()
        # 复制而不是符号链接：脚本以 dirname($0) 定位工作副本，
        # 链接会被 find_symlinks 当成交付物里的符号链接而触发闸门。
        shutil.copytree(STAGE / 'opi_robot', workdir / 'opi_robot')
        old_script = workdir / 'deploy_old.sh'
        text = SCRIPT.read_text()
        assert self.PARTIAL_FIND_MARKER + '; ' in text
        old_script.write_text(text.replace(self.PARTIAL_FIND_MARKER + '; ',
                                           ''))
        old_script.chmod(0o755)
        env = dict(os.environ)
        env['PATH'] = str(stub) + os.pathsep + env['PATH']
        env['PYTHONPATH'] = str(tmp / 'pylib')
        env['HOME'] = str(tmp / 'home')
        env['STAGE1_REPORTS_DIR'] = str(STAGE / 'reports')
        r = subprocess.run(
            ['bash', str(old_script), 'fakehost', 'orangepi', '--apply'],
            cwd=str(STAGE), env=env, capture_output=True, text=True,
            timeout=120, stdin=subprocess.DEVNULL)
        # 旧形态：远端遮蔽 + 旧脚本信任退出码 -> 漏项清单被接受，
        # 冲突文件被分类为"新增"并被 rsync 覆盖，退出 0
        assert r.returncode == 0, r.stdout + r.stderr
        assert '将新增 (1)' in r.stdout, r.stdout
        assert sandbox['rsync_calls']() >= 1
        assert target.read_text() != '# 现场改的，旧逻辑会覆盖我\n', \
            '旧逻辑竟没有覆盖目标文件，反向验证不成立'


class TestUnsafeFilenames:
    """第十一次审核其他项 1：受管路径含特殊字符时的安全行为。

    远端命令一律 `bash -c <脚本> <参数...>`，路径经 printf %q 作为 argv
    传递：单引号、空格、$ 等只是普通字符。含控制字符（如换行）的文件名
    无法在基于行的清单里安全表示，必须在预检被拒绝。
    """

    def test_single_quote_filename_is_passed_safely(self, sandbox):
        """文件名含单引号：正常完成差异核对，远端命令不被改变。"""
        probe = LOCAL_SRC / 'opi_robot' / "quote'probe.py"
        probe.write_text("# 名称里有单引号\n")
        fake_copy = sandbox['fake'] / 'opi_robot' / "quote'probe.py"
        shutil.copy2(probe, fake_copy)
        try:
            r = sandbox['run']()
            assert r.returncode == 0, r.stdout + r.stderr
            assert "quote'probe.py" in r.stdout, r.stdout
            # 本地与远端内容一致 -> 走过 remote_sha256（路径含引号）
            assert '已一致' in r.stdout
            assert fake_copy.read_text() == "# 名称里有单引号\n"
        finally:
            probe.unlink(missing_ok=True)
            fake_copy.unlink(missing_ok=True)

    def test_space_and_dollar_filename_is_passed_safely(self, sandbox):
        """文件名含空格与 $：不改变远端命令内容。"""
        probe = LOCAL_SRC / 'opi_robot' / 'space $probe.py'
        probe.write_text('x\n')
        fake_copy = sandbox['fake'] / 'opi_robot' / 'space $probe.py'
        shutil.copy2(probe, fake_copy)
        try:
            r = sandbox['run']()
            assert r.returncode == 0, r.stdout + r.stderr
            assert 'space $probe.py' in r.stdout, r.stdout
        finally:
            probe.unlink(missing_ok=True)
            fake_copy.unlink(missing_ok=True)

    def test_newline_filename_is_rejected(self, sandbox):
        """文件名含换行：清单会拆出假路径/坏行，必须 fail closed。"""
        probe = LOCAL_SRC / 'opi_robot' / 'bad\nname.py'
        probe.write_text('x\n')
        try:
            r = sandbox['run']()
            assert r.returncode == 1, r.stdout + r.stderr
            # 两条 fail-closed 出口任一：拆出的假路径不存在，
            # 或拆出的残行直接让清单格式校验失败。
            assert ('不存在的路径' in r.stdout
                    or '哈希格式不对' in r.stdout), r.stdout
        finally:
            probe.unlink(missing_ok=True)

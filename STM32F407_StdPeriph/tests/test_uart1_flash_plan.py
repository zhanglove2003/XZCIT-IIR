import importlib.util
import pathlib
import unittest
import tempfile
from unittest import mock
from contextlib import redirect_stdout, redirect_stderr
import io

p = pathlib.Path(__file__).resolve().parents[1] / 'tools' / 'c50c_uart1_flash.py'
s = importlib.util.spec_from_file_location('c50c_uart1_flash', p)
m = importlib.util.module_from_spec(s)
s.loader.exec_module(m)


def image(length):
    b = bytearray([0x5a] * length)
    b[:4] = (0x20020000).to_bytes(4, 'little')
    b[4:8] = (0x08000101).to_bytes(4, 'little')
    return bytes(b)


class PlanTest(unittest.TestCase):
    def test_preserves_bytes_outside_image_and_sector7(self):
        backup = bytes((i % 251 for i in range(512 * 1024)))
        img = image(88596)
        plan = m.prepare_image(img, backup)
        self.assertEqual(plan['sectors'], [0, 1, 2, 3, 4])
        self.assertEqual(len(plan['payload']), 0x20000)
        self.assertEqual(plan['payload'][:len(img)], img)
        self.assertEqual(plan['payload'][len(img):], backup[len(img):0x20000])

    def test_rejects_parameter_sector_overlap_and_invalid_vectors(self):
        backup = bytes([255]) * (512 * 1024)
        with self.assertRaises(ValueError):
            m.prepare_image(image(0x60001), backup)
        with self.assertRaises(ValueError):
            m.prepare_image(b'\0' * 1024, backup)
        with self.assertRaises(ValueError):
            m.prepare_image(image(1024), backup[:-1])

class WorkflowTest(unittest.TestCase):
    def test_download_uses_115200_baud_by_default(self):
        result = mock.Mock(returncode=0, stdout='Device ID    : 0x0413')
        with mock.patch.object(m.subprocess, 'run', return_value=result) as run, redirect_stdout(io.StringIO()):
            m.invoke('stm32flash', '/dev/test')
        args = run.call_args.args[0]
        self.assertEqual(args[args.index('-b') + 1], '115200')

    def test_download_supports_57600_fallback(self):
        result = mock.Mock(returncode=0, stdout='Device ID    : 0x0413')
        with mock.patch.object(m.subprocess, 'run', return_value=result) as run, redirect_stdout(io.StringIO()):
            m.invoke('stm32flash', '/dev/test', baud=57600)
        args = run.call_args.args[0]
        self.assertEqual(args[args.index('-b') + 1], '57600')

    def test_230400_available_for_explicit_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--baud', '230400']
            output = io.StringIO()
            with mock.patch('sys.argv', argv), redirect_stdout(output):
                self.assertEqual(m.main(), 0)
            self.assertIn('baud: 230400', output.getvalue())
            self.assertIn('Preview only', output.getvalue())

    def test_selected_baud_is_used_for_each_step(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            calls = []
            payload = None
            def fake(exe, device, *args, **kwargs):
                nonlocal payload
                calls.append((args, kwargs))
                if '-w' in args:
                    payload = pathlib.Path(args[args.index('-w') + 1]).read_bytes()
                elif '-r' in args:
                    path = pathlib.Path(args[args.index('-r') + 1])
                    path.write_bytes(bytes([255]) * m.FLASH_SIZE if payload is None else payload)
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--baud', '57600', '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 0)
            self.assertEqual(len(calls), 4)
            self.assertTrue(all(kw.get('baud') == 57600 for _, kw in calls))
            self.assertIn('-g', calls[-1][0])

    def test_quick_mode_matches_existing_region_and_preserves_tail(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            baseline = root / 'previous-readback.bin'
            previous = bytes([0xA3]) * 0x4000
            baseline.write_bytes(previous)
            calls, written = [], []
            def fake(exe, device, *args, **kwargs):
                calls.append(args)
                if '-w' in args:
                    written.append(pathlib.Path(args[args.index('-w') + 1]).read_bytes())
                elif '-r' in args:
                    path = pathlib.Path(args[args.index('-r') + 1])
                    path.write_bytes(previous if not written else written[-1])
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--quick-baseline', str(baseline), '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 0)
            self.assertEqual(written[0], img.read_bytes() + previous[1024:])
            self.assertEqual(len(calls), 4)
            self.assertIn('0x08000000:16384', calls[0])
            self.assertFalse(list(root.glob('flash-before-*')))
            self.assertIn('-g', calls[-1])

    def test_quick_mode_readback_mismatch_never_sends_go(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            baseline = root / 'previous-readback.bin'
            previous = bytes([0xA3]) * 0x4000
            baseline.write_bytes(previous)
            calls = []
            def fake(exe, device, *args, **kwargs):
                calls.append(args)
                if '-r' in args:
                    path = pathlib.Path(args[args.index('-r') + 1])
                    path.write_bytes(previous)
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--quick-baseline', str(baseline), '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 1)
            self.assertTrue(any('-w' in c for c in calls))
            self.assertFalse(any('-g' in c for c in calls))

    def test_quick_mode_rejects_changed_flash_before_erasing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            baseline = root / 'previous-readback.bin'
            baseline.write_bytes(bytes([0xA3]) * 0x4000)
            calls = []
            def fake(exe, device, *args, **kwargs):
                calls.append(args)
                if '-r' in args:
                    pathlib.Path(args[args.index('-r') + 1]).write_bytes(bytes([0xA4]) * 0x4000)
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--quick-baseline', str(baseline), '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 1)
            self.assertEqual(len(calls), 1)
            self.assertFalse(any('-w' in c or '-g' in c for c in calls))

    def test_quick_mode_rejects_unmatched_sector_size_before_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            baseline = root / 'previous-readback.bin'
            baseline.write_bytes(bytes([0xA3]) * 0x8000)
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--quick-baseline', str(baseline), '--execute']
            with mock.patch.object(m, 'invoke') as invoke, mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 1)
            invoke.assert_not_called()

    def test_no_write_if_backup_is_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            calls = []
            def fake(exe, device, *args, **kwargs):
                calls.append(args)
                if '-r' in args:
                    pathlib.Path(args[args.index('-r') + 1]).write_bytes(b'bad')
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 1)
            self.assertEqual(len(calls), 1)
            self.assertFalse(any('-w' in c for c in calls))

    def test_readback_mismatch_never_sends_go(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            img = root / 'image.bin'
            img.write_bytes(image(1024))
            calls = []
            def fake(exe, device, *args, **kwargs):
                calls.append(args)
                if '-r' in args:
                    path = pathlib.Path(args[args.index('-r') + 1])
                    path.write_bytes(bytes([255]) * (m.FLASH_SIZE if len(calls) == 1 else 0x4000))
            argv = ['flash', '--image', str(img), '--device', '/dev/test',
                    '--backup-dir', str(root), '--execute']
            with mock.patch.object(m, 'invoke', side_effect=fake), mock.patch('sys.argv', argv), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(), 1)
            self.assertTrue(any('-w' in c for c in calls))
            self.assertFalse(any('-g' in c for c in calls))


if __name__ == '__main__':
    unittest.main()

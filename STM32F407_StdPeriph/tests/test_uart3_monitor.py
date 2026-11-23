import importlib.util
import pathlib
import unittest

path = pathlib.Path(__file__).resolve().parents[1] / 'tools' / 'c50c_uart3_monitor.py'
spec = importlib.util.spec_from_file_location('c50c_uart3_monitor', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def frame():
    data = bytearray([0x7B, 0])
    for n in (123, -45, 67, 1, -2, 3, 4, -5, 6, 12345):
        data.extend(int(n).to_bytes(2, 'big', signed=True))
    checksum = 0
    for value in data:
        checksum ^= value
    return bytes(data + bytes([checksum, 0x7D]))


class FrameParserTests(unittest.TestCase):
    def test_stream_split_corruption_and_resynchronization(self):
        parser = module.StatusFrameParser()
        valid = frame()
        broken = bytearray(valid)
        broken[8] ^= 0x20
        self.assertEqual(parser.feed(b'noise' + bytes(broken) + valid[:9]), [])
        items = parser.feed(valid[9:] + valid)
        self.assertEqual(len(items), 2)
        self.assertEqual(items[0]['velocity_mm_s'], [123, -45])
        self.assertEqual(items[0]['yaw_mrad_s'], 67)
        self.assertEqual(items[0]['accel_raw'], [1, -2, 3])
        self.assertEqual(items[0]['gyro_raw'], [4, -5, 6])
        self.assertEqual(items[0]['voltage_mv'], 12345)

    def test_unfinished_noise_is_bounded(self):
        parser = module.StatusFrameParser()
        self.assertEqual(parser.feed(b'\x7b' * 10000), [])
        self.assertLessEqual(len(parser.buffer), 23)


if __name__ == '__main__':
    unittest.main()

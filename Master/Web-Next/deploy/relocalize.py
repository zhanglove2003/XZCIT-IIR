"""纯 NumPy 全局激光雷达扫描-地图匹配重定位（零 scipy 依赖，零底盘运动）。

用于在导航待命/静止状态下（例如人工抱起小车放到地图另一位置后），将当前
``/scan`` 与静态占据栅格 ``/map`` 做 360° 无先验全局匹配，求得机器人
``base_link`` 在 ``map`` 坐标系下的 ``(x, y, yaw)`` 并发布到 ``/initialpose``。
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np

# 与 console_page.html 保持一致：激光雷达中心相对 base_link 前移 0.08 m。
DEFAULT_LIDAR_X_M = 0.08
DEFAULT_LIDAR_Y_M = 0.0

MIN_RANGE_M = 0.12
MAX_RANGE_M = 8.0
MIN_VALID_BEAMS = 30
MIN_OCC_CELLS = 10
MIN_FREE_CELLS = 10
MIN_HIT_RATIO = 0.35

COARSE_MAX_BEAMS = 90
FINE_MAX_BEAMS = 180
DEFAULT_LUT_SIGMA_M = 0.15
DEFAULT_CLEARANCE_M = 0.18
DEFAULT_COARSE_STRIDE = 4
MID_WALL_PENALTY = 0.15


def _edt_1d(f: np.ndarray) -> np.ndarray:
    """Felzenszwalb & Huttenlocher 一维精确平方距离变换（单行）。"""
    n = f.shape[0]
    out = np.empty(n, dtype=np.float64)
    v = [0] * n
    z = [0.0] * (n + 1)
    k = 0
    v[0] = 0
    z[0] = -1e18
    z[1] = 1e18
    for q in range(1, n):
        fq = float(f[q])
        qq = float(q * q)
        while True:
            r = v[k]
            s = ((fq + qq) - (float(f[r]) + float(r * r))) / (2.0 * float(q - r))
            if s > z[k]:
                break
            k -= 1
            if k < 0:
                k = 0
                break
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = 1e18
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d = q - v[k]
        out[q] = float(d * d) + float(f[v[k]])
    return out


def euclidean_distance_transform(occ: np.ndarray) -> np.ndarray:
    """计算二维布尔占据栅格 ``occ`` 到最近占据格的精确欧氏距离（单位：栅格）。"""
    h, w = occ.shape
    if h == 0 or w == 0 or not np.any(occ):
        return np.full((h, w), float(h + w), dtype=np.float32)
    inf = float(h * h + w * w) * 4.0

    # Pass 1：沿列方向对二值 {0, +inf} 做两次线性扫描即可得到精确竖直距离平方。
    col_d = np.empty((h, w), dtype=np.float64)
    col_d[0] = np.where(occ[0], 0.0, float(h + w))
    for r in range(1, h):
        col_d[r] = np.where(occ[r], 0.0, col_d[r - 1] + 1.0)
    for r in range(h - 2, -1, -1):
        np.minimum(col_d[r], col_d[r + 1] + 1.0, out=col_d[r])
    g = col_d * col_d
    # 没有障碍物的列赋有限大 INF，避免 +/-inf 运算产生 NaN。
    no_occ_col = ~np.any(occ, axis=0)
    if np.any(no_occ_col):
        g[:, no_occ_col] = inf

    # Pass 2：沿行方向做精确抛物线下包络。
    dist2 = np.empty((h, w), dtype=np.float64)
    for r in range(h):
        dist2[r] = _edt_1d(g[r])
    return np.sqrt(np.maximum(dist2, 0.0)).astype(np.float32)


def _downsample_indices(n: int, max_count: int) -> np.ndarray:
    """从 ``n`` 条光束中等距抽取最多 ``max_count`` 条索引。"""
    if n <= max_count:
        return np.arange(n, dtype=np.int32)
    return np.linspace(0, n - 1, num=max_count, dtype=np.int32)


def _wrap_angle(rad: float) -> float:
    """将角度规范化到 (-pi, pi]。"""
    return float((rad + math.pi) % (2.0 * math.pi) - math.pi)


class ScanMapRelocalizer:
    """激光雷达扫描-静态地图全局重定位器。

    按 ``map_seq`` 缓存相似场查找表（LUT）与候选位姿网格，线程安全。
    """

    def __init__(
        self,
        lidar_x: float = DEFAULT_LIDAR_X_M,
        lidar_y: float = DEFAULT_LIDAR_Y_M,
        sigma_m: float = DEFAULT_LUT_SIGMA_M,
        clearance_m: float = DEFAULT_CLEARANCE_M,
        coarse_stride: int = DEFAULT_COARSE_STRIDE,
    ):
        self._lidar_x = float(lidar_x)
        self._lidar_y = float(lidar_y)
        self._sigma_m = float(sigma_m)
        self._clearance_m = float(clearance_m)
        self._coarse_stride = max(1, int(coarse_stride))
        self._cache_lock = threading.Lock()
        self._cached_key = None
        self._cached_lut = None

    def _build_lut(self, grid: dict) -> dict:
        w = int(grid['w'])
        h = int(grid['h'])
        res = float(grid['res'])
        ox = float(grid['ox'])
        oy = float(grid['oy'])
        raw = grid['data']
        raw_bytes = raw if isinstance(raw, (bytes, bytearray)) else bytes(raw)
        if w <= 0 or h <= 0 or res <= 0.0 or len(raw_bytes) != w * h:
            raise ValueError('地图栅格尺寸或数据非法，无法重定位')

        arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w))
        occ = (arr >= 50) & (arr <= 100)
        free = arr < 50
        n_occ = int(np.count_nonzero(occ))
        n_free = int(np.count_nonzero(free))
        if n_occ < MIN_OCC_CELLS:
            raise ValueError(f'地图障碍物栅格不足（仅 {n_occ} 格），无法定位')
        if n_free < MIN_FREE_CELLS:
            raise ValueError(f'地图已知空闲区域不足（仅 {n_free} 格），无法定位')

        # 裁剪到占据区域外扩 10 格的包络框，减少无关未知边界计算与无效候选。
        occ_ys, occ_xs = np.nonzero(occ)
        pad = 10
        x0 = max(0, int(occ_xs.min()) - pad)
        x1 = min(w, int(occ_xs.max()) + 1 + pad)
        y0 = max(0, int(occ_ys.min()) - pad)
        y1 = min(h, int(occ_ys.max()) + 1 + pad)

        dist_cells = euclidean_distance_transform(occ)
        dist_m = dist_cells * np.float32(res)
        sigma = max(self._sigma_m, 2.5 * res)
        lut = np.exp(-0.5 * (dist_m / np.float32(sigma)) ** 2).astype(np.float32)

        bbox_mask = np.zeros((h, w), dtype=bool)
        bbox_mask[y0:y1, x0:x1] = True
        free_clear = free & bbox_mask & (dist_m >= np.float32(self._clearance_m))
        if int(np.count_nonzero(free_clear)) < 5:
            free_clear = free & bbox_mask
        if int(np.count_nonzero(free_clear)) == 0:
            raise ValueError('地图无满足安全余量的候选空闲栅格')

        cand_ix = cand_iy = None
        for stride in (self._coarse_stride, 2, 1):
            stride_mask = np.zeros((h, w), dtype=bool)
            stride_mask[::stride, ::stride] = True
            ys, xs = np.nonzero(free_clear & stride_mask)
            if len(xs) >= 16 or stride == 1:
                cand_ix = xs.astype(np.int32)
                cand_iy = ys.astype(np.int32)
                break

        if cand_ix is None or len(cand_ix) == 0:
            raise ValueError('地图候选点为空，无法重定位')

        return {
            'w': w,
            'h': h,
            'res': res,
            'ox': ox,
            'oy': oy,
            'occ': occ,
            'dist_m': dist_m,
            'lut': lut,
            'cand_ix': cand_ix,
            'cand_iy': cand_iy,
            'hit_thresh_m': max(self._sigma_m, 2.5 * res),
        }

    def _get_lut(self, grid: dict) -> tuple[dict, bool, float]:
        t0 = time.monotonic()
        raw = grid.get('data', b'')
        raw_bytes = raw if isinstance(raw, (bytes, bytearray)) else bytes(raw)
        key = (
            int(grid.get('seq', -1)),
            int(grid.get('w', 0)),
            int(grid.get('h', 0)),
            round(float(grid.get('res', 0.0)), 6),
            round(float(grid.get('ox', 0.0)), 6),
            round(float(grid.get('oy', 0.0)), 6),
            hash(raw_bytes),
        )
        with self._cache_lock:
            if self._cached_key == key and self._cached_lut is not None:
                return self._cached_lut, True, (time.monotonic() - t0) * 1000.0
        built = self._build_lut(grid)
        dt_ms = (time.monotonic() - t0) * 1000.0
        with self._cache_lock:
            self._cached_key = key
            self._cached_lut = built
        return built, False, dt_ms

    def _extract_scan(self, scan_pts) -> tuple[np.ndarray, np.ndarray]:
        if not scan_pts:
            raise ValueError('当前无激光雷达扫描数据')
        arr = np.asarray(scan_pts, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            raise ValueError('激光扫描格式无效')
        degs = arr[:, 0]
        rs = arr[:, 1]
        valid = np.isfinite(degs) & np.isfinite(rs) & (rs >= MIN_RANGE_M) & (rs <= MAX_RANGE_M)
        n_valid = int(np.count_nonzero(valid))
        if n_valid < MIN_VALID_BEAMS:
            raise ValueError(f'有效激光束过少（{n_valid} < {MIN_VALID_BEAMS}），请移至开阔处重试')
        return np.deg2rad(degs[valid]), rs[valid]

    def _beam_offsets_cells(
        self, thetas: np.ndarray, rs: np.ndarray, yaw: float, res: float, scale: float = 1.0
    ) -> tuple[np.ndarray, np.ndarray]:
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        lx = (self._lidar_x * cy - self._lidar_y * sy) / res
        ly = (self._lidar_x * sy + self._lidar_y * cy) / res
        ang = thetas + yaw
        sr = (rs * scale) / res
        dx = np.floor(0.5 + lx + sr * np.cos(ang)).astype(np.int32)
        dy = np.floor(0.5 + ly + sr * np.sin(ang)).astype(np.int32)
        return dx, dy

    def _score_pose_batch(
        self,
        lut_data: dict,
        cand_x_m: np.ndarray,
        cand_y_m: np.ndarray,
        thetas: np.ndarray,
        rs: np.ndarray,
        yaws: np.ndarray,
        apply_wall_penalty: bool,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """对一批显式 (x_m, y_m, yaw) 候选在连续世界坐标中打分。

        返回 ``(scores, best_yaw_idx, hit_ratios)``? 这里直接返回 ``scores``，
        外层按需挑选最优后另算 ``hit_ratio``。实际上为速度考虑，直接对给定的
        ``(cand_x_m, cand_y_m) x yaws`` 展开并返回二维分数矩阵 ``(len(cand_x_m), len(yaws))``。
        """
        w = lut_data['w']
        h = lut_data['h']
        res = lut_data['res']
        ox = lut_data['ox']
        oy = lut_data['oy']
        lut = lut_data['lut']
        occ = lut_data['occ']

        cx_f = (cand_x_m - ox) / res
        cy_f = (cand_y_m - oy) / res
        scores = np.empty((len(cand_x_m), len(yaws)), dtype=np.float32)

        if apply_wall_penalty:
            long_mask = rs >= 0.40
            mid_th = thetas[long_mask]
            mid_r = rs[long_mask]
            has_mid = len(mid_r) > 0
        else:
            mid_th = mid_r = None
            has_mid = False

        inv_n = np.float32(1.0 / max(1, len(rs)))
        for ai, yaw in enumerate(yaws):
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            lx_cells = (self._lidar_x * cy - self._lidar_y * sy) / res
            ly_cells = (self._lidar_x * sy + self._lidar_y * cy) / res
            ang = thetas + yaw
            rx_cells = lx_cells + (rs / res) * np.cos(ang)
            ry_cells = ly_cells + (rs / res) * np.sin(ang)

            hx = np.floor(cx_f[:, None] + rx_cells[None, :]).astype(np.int32)
            hy = np.floor(cy_f[:, None] + ry_cells[None, :]).astype(np.int32)
            inb = (hx >= 0) & (hx < w) & (hy >= 0) & (hy < h)
            hx_c = np.clip(hx, 0, w - 1)
            hy_c = np.clip(hy, 0, h - 1)
            vals = np.where(inb, lut[hy_c, hx_c], np.float32(0.0))
            sc = vals.sum(axis=1) * inv_n

            if apply_wall_penalty and has_mid:
                mang = mid_th + yaw
                mx_cells = lx_cells + (0.5 * mid_r / res) * np.cos(mang)
                my_cells = ly_cells + (0.5 * mid_r / res) * np.sin(mang)
                mhx = np.floor(cx_f[:, None] + mx_cells[None, :]).astype(np.int32)
                mhy = np.floor(cy_f[:, None] + my_cells[None, :]).astype(np.int32)
                minb = (mhx >= 0) & (mhx < w) & (mhy >= 0) & (mhy < h)
                mhx_c = np.clip(mhx, 0, w - 1)
                mhy_c = np.clip(mhy, 0, h - 1)
                mid_occ = np.where(minb, occ[mhy_c, mhx_c], False)
                pen = mid_occ.mean(axis=1).astype(np.float32) * np.float32(MID_WALL_PENALTY)
                sc = sc - pen

            scores[:, ai] = sc
        return scores, np.empty(0), np.empty(0)

    def _coarse_search(
        self, lut_data: dict, c_th: np.ndarray, c_rs: np.ndarray
    ) -> list[tuple[float, float, float, float]]:
        """阶段 1：360° 全局网格粗搜索，保留最多 10 个时空分散的局部峰。"""
        w = lut_data['w']
        h = lut_data['h']
        res = lut_data['res']
        ox = lut_data['ox']
        oy = lut_data['oy']
        lut = lut_data['lut']
        cand_ix = lut_data['cand_ix']
        cand_iy = lut_data['cand_iy']
        n_cands = len(cand_ix)
        n_beams = len(c_rs)
        inv_n = np.float32(1.0 / n_beams)

        yaws = np.linspace(-math.pi, math.pi, num=72, endpoint=False)
        n_yaws = len(yaws)
        # 为每个 (cand, angle) 直接维护全局候选分数；内存占 n_cands * 72 * 4 B（~2 MB）
        score_grid = np.empty((n_cands, n_yaws), dtype=np.float32)

        chunk_size = 4096
        for ai, yaw in enumerate(yaws):
            dx, dy = self._beam_offsets_cells(c_th, c_rs, float(yaw), res, scale=1.0)
            for s0 in range(0, n_cands, chunk_size):
                s1 = min(n_cands, s0 + chunk_size)
                cix = cand_ix[s0:s1, None]
                ciy = cand_iy[s0:s1, None]
                hx = cix + dx[None, :]
                hy = ciy + dy[None, :]
                inb = (hx >= 0) & (hx < w) & (hy >= 0) & (hy < h)
                hx_c = np.clip(hx, 0, w - 1)
                hy_c = np.clip(hy, 0, h - 1)
                vals = np.where(inb, lut[hy_c, hx_c], np.float32(0.0))
                score_grid[s0:s1, ai] = vals.sum(axis=1) * inv_n

        # 从全矩阵抽前 300 个最高分做 3D (x, y, yaw) 非极大值抑制，避免单一对称峰丢失次优真峰
        flat = score_grid.ravel()
        top_n = min(300, len(flat))
        top_flat_idx = np.argpartition(flat, -top_n)[-top_n:]
        top_flat_idx = top_flat_idx[np.argsort(flat[top_flat_idx])[::-1]]

        peaks: list[tuple[float, float, float, float]] = []
        for fi in top_flat_idx:
            ci = int(fi // n_yaws)
            ai = int(fi % n_yaws)
            sc = float(flat[fi])
            px = ox + (float(cand_ix[ci]) + 0.5) * res
            py = oy + (float(cand_iy[ci]) + 0.5) * res
            pyaw = float(yaws[ai])
            duplicate = False
            for ex, ey, eyaw, _ in peaks:
                if (px - ex) ** 2 + (py - ey) ** 2 < 0.25 ** 2 and abs(_wrap_angle(pyaw - eyaw)) < math.radians(20.0):
                    duplicate = True
                    break
            if not duplicate:
                peaks.append((px, py, pyaw, sc))
                if len(peaks) >= 10:
                    break
        return peaks

    def _hit_ratio(self, lut_data: dict, x_m: float, y_m: float, yaw: float, thetas: np.ndarray, rs: np.ndarray) -> float:
        w = lut_data['w']
        h = lut_data['h']
        res = lut_data['res']
        ox = lut_data['ox']
        oy = lut_data['oy']
        dist_m = lut_data['dist_m']
        thresh = lut_data['hit_thresh_m']

        cy = math.cos(yaw)
        sy = math.sin(yaw)
        lx = self._lidar_x * cy - self._lidar_y * sy
        ly = self._lidar_x * sy + self._lidar_y * cy
        ang = thetas + yaw
        wx = x_m + lx + rs * np.cos(ang)
        wy = y_m + ly + rs * np.sin(ang)
        hx = np.floor((wx - ox) / res).astype(np.int32)
        hy = np.floor((wy - oy) / res).astype(np.int32)
        inb = (hx >= 0) & (hx < w) & (hy >= 0) & (hy < h)
        if not np.any(inb):
            return 0.0
        hits = np.zeros(len(rs), dtype=bool)
        hits[inb] = dist_m[hy[inb], hx[inb]] <= thresh
        return float(np.mean(hits))

    def relocalize(self, grid: dict, scan_pts) -> dict:
        """执行 360° 扫描-地图全局匹配，返回最优位姿及诊断指标。"""
        if not grid or not grid.get('data'):
            raise ValueError('尚未收到静态地图 (/map)，无法定位校准')

        all_th, all_rs = self._extract_scan(scan_pts)
        n_valid = len(all_rs)

        lut_data, lut_cached, lut_ms = self._get_lut(grid)
        t_match0 = time.monotonic()

        c_idx = _downsample_indices(n_valid, COARSE_MAX_BEAMS)
        f_idx = _downsample_indices(n_valid, FINE_MAX_BEAMS)
        c_th, c_rs = all_th[c_idx], all_rs[c_idx]
        f_th, f_rs = all_th[f_idx], all_rs[f_idx]

        # 阶段 1：全局 360° 粗搜
        peaks = self._coarse_search(lut_data, c_th, c_rs)
        if not peaks:
            raise ValueError('全局定位未找到有效匹配峰值')

        # 阶段 2：围绕 top-K 粗峰做 ±0.25 m（步长 0.05 m）、±6°（步长 1°）精细搜索
        d_xy = np.linspace(-0.25, 0.25, 11)
        gx_off, gy_off = np.meshgrid(d_xy, d_xy, indexing='xy')
        gx_off = gx_off.ravel()
        gy_off = gy_off.ravel()
        fine_dyaw = np.deg2rad(np.arange(-6.0, 6.5, 1.0))

        fine_candidates: list[tuple[float, float, float, float]] = []
        for px, py, pyaw, _ in peaks:
            cand_x = px + gx_off
            cand_y = py + gy_off
            cand_yaws = pyaw + fine_dyaw
            mat, _, _ = self._score_pose_batch(
                lut_data, cand_x, cand_y, f_th, f_rs, cand_yaws, apply_wall_penalty=True
            )
            flat_best = int(np.argmax(mat))
            bi = flat_best // len(cand_yaws)
            ai = flat_best % len(cand_yaws)
            fine_candidates.append(
                (float(cand_x[bi]), float(cand_y[bi]), float(cand_yaws[ai]), float(mat[bi, ai]))
            )
        fine_candidates.sort(key=lambda item: item[3], reverse=True)

        # 阶段 3：围绕前 2 个精搜峰值做 ±0.06 m（步长 0.015 m）、±2°（步长 0.5°）微调
        md_xy = np.linspace(-0.06, 0.06, 9)
        mgx, mgy = np.meshgrid(md_xy, md_xy, indexing='xy')
        mgx = mgx.ravel()
        mgy = mgy.ravel()
        micro_dyaw = np.deg2rad(np.arange(-2.0, 2.25, 0.5))

        best_x = fine_candidates[0][0]
        best_y = fine_candidates[0][1]
        best_yaw = fine_candidates[0][2]
        best_score = -1e9
        for px, py, pyaw, _ in fine_candidates[:2]:
            cand_x = px + mgx
            cand_y = py + mgy
            cand_yaws = pyaw + micro_dyaw
            mat, _, _ = self._score_pose_batch(
                lut_data, cand_x, cand_y, f_th, f_rs, cand_yaws, apply_wall_penalty=True
            )
            flat_best = int(np.argmax(mat))
            bi = flat_best // len(cand_yaws)
            ai = flat_best % len(cand_yaws)
            sc = float(mat[bi, ai])
            if sc > best_score:
                best_score = sc
                best_x = float(cand_x[bi])
                best_y = float(cand_y[bi])
                best_yaw = float(cand_yaws[ai])

        best_yaw = _wrap_angle(best_yaw)
        hit_ratio = self._hit_ratio(lut_data, best_x, best_y, best_yaw, all_th, all_rs)
        match_ms = (time.monotonic() - t_match0) * 1000.0

        if hit_ratio < MIN_HIT_RATIO:
            raise ValueError(
                f'扫描与地图匹配度过低（命中率 {hit_ratio * 100:.1f}% < '
                f'{MIN_HIT_RATIO * 100:.0f}%），请确认小车当前在已建图区域内'
            )

        return {
            'ok': True,
            'x': round(best_x, 4),
            'y': round(best_y, 4),
            'yaw_rad': round(best_yaw, 5),
            'yaw_deg': round(math.degrees(best_yaw), 2),
            'score': round(best_score, 4),
            'hit_ratio': round(hit_ratio, 4),
            'valid_beams': n_valid,
            'used_beams': len(f_rs),
            'candidates': len(lut_data['cand_ix']),
            'lut_ms': round(lut_ms, 1),
            'match_ms': round(match_ms, 1),
            'lut_cached': lut_cached,
        }

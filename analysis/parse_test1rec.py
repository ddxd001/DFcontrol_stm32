#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解析 TEST1-REC 经 USART1 输出的 VOFA+ FireWater 文本，复现固件中与
App_AnalyzeTest1RecBuffer() 相同的路口 debounce 与 l1/l2 距离公式。

固件行格式（行尾 CRLF 或 LF）:
  t1r_m:<cnt>          — 采样点数（可与后续行数核对）
  t1r:<idx>,<tick_ms>,<inv> — 单点：序号、HAL_GetTick、digital_inv(0..255)

常量与 Firmware/App/app_tasks.c、gw_gray_sensor.c 对齐。

- `--plot`：八路 **digital_inv** 按位灰度图（时间 × 通道）+ **popcount** 曲线 + **raw / confirmed** 路口台阶；红色竖线为算法识别的路口时刻。
- `--plot-save 路径.png`：保存图像；可与 `--no-show` 联用（仅保存不弹窗）。

用法示例：`python parse_test1rec.py data.txt --plot --plot-save gray.png`

依赖：`pip install matplotlib`（见 requirements.txt）。
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

# 与 app_tasks.c 一致
CROSS_ON_CONFIRM_FRAMES = 4
CROSS_OFF_CONFIRM_FRAMES = 3
GRAY_CROSS_MIN_ACTIVE_BITS = 3
EXPECT_CROSS_COUNT = 3
DIST_CM_X10_MIN = 10   # 1.0 cm
DIST_CM_X10_MAX = 50   # 5.0 cm


def popcount8(v: int) -> int:
    return bin(v & 0xFF).count("1")


def is_cross_by_inv(digital_inv: int) -> bool:
    return popcount8(digital_inv) >= GRAY_CROSS_MIN_ACTIVE_BITS


def dist_cm_x10_from_dt_ms(dt_ms: int) -> int:
    if dt_ms <= 0:
        return DIST_CM_X10_MIN
    d = (dt_ms * 1197 + 5000) // 10000
    if d < DIST_CM_X10_MIN:
        return DIST_CM_X10_MIN
    if d > DIST_CM_X10_MAX:
        return DIST_CM_X10_MAX
    return d


@dataclass
class Sample:
    idx: int
    tick_ms: int
    inv: int


_META_RE = re.compile(r"^\s*t1r_m:(\d+)\s*$")
_ROW_RE = re.compile(r"^\s*t1r:(\d+),(\d+),(\d+)\s*$")


def parse_firewater_lines(lines: Iterable[str]) -> Tuple[Optional[int], List[Sample]]:
    meta_cnt: Optional[int] = None
    samples: List[Sample] = []
    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        m_meta = _META_RE.match(line)
        if m_meta:
            meta_cnt = int(m_meta.group(1))
            continue
        m_row = _ROW_RE.match(line)
        if m_row:
            samples.append(
                Sample(
                    idx=int(m_row.group(1)),
                    tick_ms=int(m_row.group(2)),
                    inv=int(m_row.group(3)) & 0xFF,
                )
            )
            continue
    return meta_cnt, samples


def analyze_cross_series(samples: List[Sample]) -> Tuple[List[int], List[int]]:
    """返回每个采样点上的 (confirmed路口态, raw_cross)；道口时刻见 analyze_cross_ticks。"""
    on_cnt = 0
    off_cnt = 0
    confirmed = 0
    confirmed_list: List[int] = []
    raw_list: List[int] = []

    for s in samples:
        raw = 1 if is_cross_by_inv(s.inv) else 0
        raw_list.append(raw)
        if raw:
            on_cnt = min(on_cnt + 1, 255)
            off_cnt = 0
            if confirmed == 0 and on_cnt >= CROSS_ON_CONFIRM_FRAMES:
                confirmed = 1
        else:
            off_cnt = min(off_cnt + 1, 255)
            on_cnt = 0
            if confirmed != 0 and off_cnt >= CROSS_OFF_CONFIRM_FRAMES:
                confirmed = 0
        confirmed_list.append(confirmed)

    return confirmed_list, raw_list


def analyze_cross_ticks(samples: List[Sample]) -> List[int]:
    """复现 App_AnalyzeTest1RecBuffer：debounce + 至多 3 个路口 tick。"""
    on_cnt = 0
    off_cnt = 0
    confirmed = 0
    latched = 0
    cross_ticks: List[int] = []

    for s in samples:
        raw = 1 if is_cross_by_inv(s.inv) else 0
        if raw:
            on_cnt = min(on_cnt + 1, 255)
            off_cnt = 0
            if confirmed == 0 and on_cnt >= CROSS_ON_CONFIRM_FRAMES:
                confirmed = 1
        else:
            off_cnt = min(off_cnt + 1, 255)
            on_cnt = 0
            if confirmed != 0 and off_cnt >= CROSS_OFF_CONFIRM_FRAMES:
                confirmed = 0

        if confirmed != 0:
            if latched == 0:
                if len(cross_ticks) < EXPECT_CROSS_COUNT:
                    cross_ticks.append(s.tick_ms)
                latched = 1
                if len(cross_ticks) >= EXPECT_CROSS_COUNT:
                    break
        else:
            latched = 0

    return cross_ticks


def inv_bit_matrix(samples: List[Sample]) -> List[List[float]]:
    """8 行 × N 列：行 i = digital_inv 的 bit i（1=该路见黑线）。"""
    n = len(samples)
    m: List[List[float]] = [[0.0] * n for _ in range(8)]
    for j, s in enumerate(samples):
        v = s.inv & 0xFF
        for i in range(8):
            m[i][j] = 1.0 if (v >> i) & 1 else 0.0
    return m


def plot_gray_capture(
    samples: List[Sample],
    cross_ticks: List[int],
    *,
    save_path: Optional[Path] = None,
    show: bool = True,
) -> None:
    """八路灰度按位热力图 + popcount + 单帧/防抖路口，便于直观对照采样。"""
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False

    if not samples:
        return

    t0 = samples[0].tick_ms
    t_rel = [float(s.tick_ms - t0) for s in samples]
    t_lo = t_rel[0] - 5.0
    t_hi = t_rel[-1] + 5.0

    mat = inv_bit_matrix(samples)
    y_pc = [popcount8(s.inv) for s in samples]
    confirmed_ser, raw_ser = analyze_cross_series(samples)

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(12, 9),
        sharex=True,
        gridspec_kw={"height_ratios": [2.2, 1.0, 1.0]},
    )
    ax0, ax1, ax2 = axes

    im = ax0.imshow(
        mat,
        origin="lower",
        aspect="auto",
        interpolation="nearest",
        cmap="gray_r",
        vmin=0.0,
        vmax=1.0,
        extent=[t_lo, t_hi, -0.5, 7.5],
    )
    ax0.set_ylabel("八路灰度 (CH = bit+1)")
    ax0.set_yticks(range(8))
    ax0.set_yticklabels([f"CH{i + 1}" for i in range(8)])
    ax0.set_title("digital_inv 按位展开（列=时间 ms；浅=未见线，深=见黑线）")
    fig.colorbar(im, ax=ax0, fraction=0.025, pad=0.02, label="见线强度")

    ax1.plot(t_rel, y_pc, "b.-", linewidth=1.0, markersize=3, label="popcount(inv)")
    ax1.axhline(
        GRAY_CROSS_MIN_ACTIVE_BITS,
        color="darkorange",
        linestyle="--",
        linewidth=1.0,
        label=f">= {GRAY_CROSS_MIN_ACTIVE_BITS} → raw 路口",
    )
    ax1.set_ylabel("命中路数")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title("全路「见线」路数（路口常明显抬升）")
    ax1.grid(True, alpha=0.3)

    ax2.step(t_rel, raw_ser, where="post", color="steelblue", alpha=0.75, label="raw_cross（单帧）")
    ax2.step(
        t_rel,
        confirmed_ser,
        where="post",
        color="darkgreen",
        linewidth=1.2,
        label="confirmed（ON/OFF 帧防抖）",
    )
    ax2.set_ylabel("0 / 1")
    ax2.set_xlabel("时间 (ms，相对首点)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.set_title("路口逻辑（与固件 App_ProcessCrossDetect 一致）")
    ax2.set_ylim(-0.1, 1.15)
    ax2.grid(True, alpha=0.3)

    cross_rel = [round(ct - t0, 1) for ct in cross_ticks]
    for ax in axes:
        for x in cross_rel:
            ax.axvline(x, color="red", linestyle=":", linewidth=1.0, alpha=0.85)

    fig.suptitle(
        f"TEST1-REC | N={len(samples)} | 路口相对时刻(ms): {cross_rel}",
        fontsize=11,
    )
    fig.tight_layout()

    if save_path is not None:
        sp = Path(save_path)
        sp.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(sp, dpi=150, bbox_inches="tight")
        print(f"已保存图像: {sp}", file=sys.stderr)

    if show:
        plt.show()
    else:
        plt.close(fig)


def l1_l2_cm_from_cross_ticks(ticks: List[int]) -> Tuple[Optional[float], Optional[float]]:
    """与测距日志一致：第 2、3 路口相对前一路口间距（cm，一位小数）。"""
    l1_x10: Optional[int] = None
    l2_x10: Optional[int] = None
    if len(ticks) >= 2:
        l1_x10 = dist_cm_x10_from_dt_ms(ticks[1] - ticks[0])
    if len(ticks) >= 3:
        l2_x10 = dist_cm_x10_from_dt_ms(ticks[2] - ticks[1])
    def to_cm(x: Optional[int]) -> Optional[float]:
        if x is None:
            return None
        return round(x / 10.0, 1)
    return to_cm(l1_x10), to_cm(l2_x10)


def write_samples_csv(path: Path, samples: List[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["idx", "tick_ms", "digital_inv", "popcount", "raw_cross"])
        for s in samples:
            pc = popcount8(s.inv)
            w.writerow([s.idx, s.tick_ms, s.inv, pc, int(is_cross_by_inv(s.inv))])


def main() -> int:
    p = argparse.ArgumentParser(description="分析 TEST1-REC FireWater 串口日志")
    p.add_argument(
        "input",
        nargs="?",
        type=Path,
        help="文本文件路径（缺省则从 stdin 读）",
    )
    p.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="将解析后的采样表写入 CSV",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="仅打印 l1/l2，不打印详情",
    )
    p.add_argument(
        "--plot",
        action="store_true",
        help="绘制八路灰度位图 + popcount + 路口 raw/confirmed（需 matplotlib）",
    )
    p.add_argument(
        "--plot-save",
        type=Path,
        default=None,
        metavar="PATH",
        help="将图表保存为 PNG（可与 --no-show 联用）",
    )
    p.add_argument(
        "--no-show",
        action="store_true",
        help="绘图时不弹窗（通常与 --plot-save 一起用）",
    )
    args = p.parse_args()

    if args.input is not None:
        text = args.input.read_text(encoding="utf-8", errors="replace")
        lines = text.splitlines()
    else:
        if sys.stdin.isatty():
            p.print_help()
            print("\n提示: 请提供文件路径或管道传入日志。", file=sys.stderr)
            return 2
        lines = sys.stdin.read().splitlines()

    meta_cnt, samples = parse_firewater_lines(lines)

    if not args.quiet:
        print(f"解析到采样: {len(samples)} 点", file=sys.stderr)
        if meta_cnt is not None:
            if meta_cnt != len(samples):
                print(
                    f"警告: 元数据 t1r_m 声明 cnt={meta_cnt}，与实际数据行 {len(samples)} 不一致",
                    file=sys.stderr,
                )
            else:
                print(f"与元数据 cnt={meta_cnt} 一致", file=sys.stderr)

    crosses = analyze_cross_ticks(samples)
    l1, l2 = l1_l2_cm_from_cross_ticks(crosses)

    if not args.quiet:
        print(f"路口时刻 tick_ms (共 {len(crosses)} 个): {crosses}", file=sys.stderr)

    default_cm = 2.5
    l1_out = default_cm if l1 is None else l1
    l2_out = default_cm if l2 is None else l2
    print(f"l1 = {l1_out}cm")
    print(f"l2 = {l2_out}cm")

    if args.csv is not None:
        write_samples_csv(args.csv, samples)
        if not args.quiet:
            print(f"已写 CSV: {args.csv}", file=sys.stderr)

    want_plot = args.plot or args.plot_save is not None
    if want_plot:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            print("错误: 绘图需要 matplotlib（pip install matplotlib）", file=sys.stderr)
            return 1
        plot_gray_capture(
            samples,
            crosses,
            save_path=args.plot_save,
            show=not args.no_show,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""
三种任务分配算法性能对比可视化
算法：KM（匈牙利算法）、贪心算法、MTD-HCGA
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm
import numpy as np

# ── 字体准备：只把图表用到的中文字符从 Droid 注入 DejaVu ─────────────────────
_FONT_CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '_merged_font.ttf')
_FONT_NAME  = 'DejaVuSansCJK'

def _build_merged_font():
    from fontTools.ttLib import TTFont

    used_chars = (
        '三种任务分配算法主要性能指标对比完工时间总行驶距离负载方差战略响应'
        '各车行驶距离分布综合雷达图归一化越大越优实测与理论贪心匈牙利本文方法'
        '车辆算法名称时间秒米平方对齐效率理论工作均衡性'
        '一二三四五六七八九十百千万'
        '（）【】、。，：；'
    )
    used_cps = {ord(c) for c in used_chars if ord(c) > 127}

    dv = TTFont('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    dr = TTFont('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    dv_cmap = dv.getBestCmap()
    dr_cmap = dr.getBestCmap()
    new_map = {cp: dr_cmap[cp] for cp in used_cps
               if cp in dr_cmap and cp not in dv_cmap}

    dv_glyf, dr_glyf = dv['glyf'], dr['glyf']
    dv_hmtx, dr_hmtx = dv['hmtx'], dr['hmtx']
    order    = dv.getGlyphOrder()
    existing = set(dv_glyf.glyphs.keys())

    def _copy(gn):
        if gn in existing or gn not in dr_glyf:
            return
        g = dr_glyf[gn]
        if g.isComposite():
            for comp in g.components:
                _copy(comp.glyphName)
        dv_glyf[gn] = g
        if gn in dr_hmtx.metrics:
            dv_hmtx.metrics[gn] = dr_hmtx.metrics[gn]
        order.append(gn)
        existing.add(gn)

    for gn in new_map.values():
        _copy(gn)

    seen, clean = set(), []
    for gn in order:
        if gn not in seen and gn in dv_glyf.glyphs:
            clean.append(gn)
            seen.add(gn)
    dv.setGlyphOrder(clean)

    for t in dv['cmap'].tables:
        if t.format == 4:
            t.cmap.update(new_map)
            break

    # 重命名，避免与系统 DejaVu Sans 冲突
    for record in dv['name'].names:
        if record.nameID in (1, 4, 6):
            record.string = _FONT_NAME.encode('utf-16-be')
            record.platformID = 3
            record.platEncID = 1
            record.langID = 0x409

    dv.save(_FONT_CACHE)

if not os.path.exists(_FONT_CACHE):
    _build_merged_font()

# 把合并字体安装到 matplotlib 字体目录，并清除缓存
import shutil, matplotlib
_mpl_font_dir = os.path.join(matplotlib.get_data_path(), 'fonts', 'ttf')
_mpl_font_dst = os.path.join(_mpl_font_dir, 'DejaVuSansCJK.ttf')
if not os.path.exists(_mpl_font_dst):
    shutil.copy2(_FONT_CACHE, _mpl_font_dst)
    # 删除字体缓存，强制重建
    _cache_dir = matplotlib.get_cachedir()
    for f in os.listdir(_cache_dir):
        if f.startswith('fontlist') and f.endswith('.json'):
            os.remove(os.path.join(_cache_dir, f))
    # 重新初始化字体管理器
    fm._rebuild()

fm.fontManager.addfont(_FONT_CACHE)
plt.rcParams['font.sans-serif'] = [_FONT_NAME, 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 12

# ── 数据 ──────────────────────────────────────────────────────────────────────
alg_short  = ['KM', '贪心', 'MTD-HCGA']
colors     = ['#5B9BD5', '#ED7D31', '#70AD47']
hatches    = ['/', '\\', '']

makespan   = [262.951, 111.364, 59.606]
total_dist = [121.142, 162.953, 61.609]
variance   = [178.5907, 20.3772, 57.6659]
resp_time  = [54.336,  98.677,  32.21]

car1 = [23.686, 50.942, 9.967]
car2 = [41.056, 51.313, 27.467]
car3 = [56.400, 60.698, 24.176]

t_makespan_theo = [57.4095, 28.5497, 32.7659]

x     = np.arange(3)
bar_w = 0.55

# ── 图1：四大主指标柱状图（2×2） ─────────────────────────────────────────────
fig1, axes = plt.subplots(2, 2, figsize=(13, 9))
fig1.suptitle('三种任务分配算法主要性能指标对比', fontsize=15, fontweight='bold', y=1.01)

metrics = [
    (makespan,   '完工时间 (s)',     '完工时间对比'),
    (total_dist, '总行驶距离 (m)',   '总行驶距离对比'),
    (variance,   '负载方差 (m²)',    '工作负载均衡性对比'),
    (resp_time,  '战略响应时间 (s)', '战略响应时间对比'),
]

for ax, (data, ylabel, title) in zip(axes.flat, metrics):
    bars = ax.bar(x, data, width=bar_w, color=colors, edgecolor='white',
                  linewidth=1.2, zorder=3)
    for bar, h, hatch in zip(bars, data, hatches):
        bar.set_hatch(hatch)
        ax.text(bar.get_x() + bar.get_width() / 2, h + max(data) * 0.02,
                f'{h:.2f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(alg_short, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_ylim(0, max(data) * 1.22)
    ax.yaxis.grid(True, linestyle='--', alpha=0.6, zorder=0)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

legend_patches = [mpatches.Patch(facecolor=c, label=l, hatch=h, edgecolor='grey')
                  for c, l, h in zip(colors,
                      ['KM（匈牙利算法）', '贪心算法', 'MTD-HCGA（本文方法）'], hatches)]
fig1.legend(handles=legend_patches, loc='lower center', ncol=3,
            fontsize=11, frameon=True, bbox_to_anchor=(0.5, -0.04))
plt.tight_layout()
fig1.savefig('算法主指标对比.png', dpi=180, bbox_inches='tight')
print("已保存：算法主指标对比.png")

# ── 图2：各车行驶距离分组柱状图 ──────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(10, 6))
bw = 0.22
r1 = np.arange(3)
r2, r3 = r1 + bw, r1 + 2 * bw

b1 = ax2.bar(r1, car1, width=bw, label='车辆1', color='#4472C4', edgecolor='white')
b2 = ax2.bar(r2, car2, width=bw, label='车辆2', color='#ED7D31', edgecolor='white')
b3 = ax2.bar(r3, car3, width=bw, label='车辆3', color='#A9D18E', edgecolor='white')

for bars in [b1, b2, b3]:
    for bar in bars:
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, h + 0.8,
                 f'{h:.1f}', ha='center', va='bottom', fontsize=9)

ax2.set_xticks(r1 + bw)
ax2.set_xticklabels(['KM（匈牙利算法）', '贪心算法', 'MTD-HCGA（本文方法）'], fontsize=11)
ax2.set_ylabel('行驶距离 (m)', fontsize=12)
ax2.set_title('各算法下三辆车行驶距离分布', fontsize=13, fontweight='bold')
ax2.legend(fontsize=11, frameon=True)
ax2.set_ylim(0, 80)
ax2.yaxis.grid(True, linestyle='--', alpha=0.6)
ax2.set_axisbelow(True)
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
plt.tight_layout()
fig2.savefig('各车行驶距离对比.png', dpi=180, bbox_inches='tight')
print("已保存：各车行驶距离对比.png")

# ── 图3：雷达图（归一化多维对比） ────────────────────────────────────────────
categories = ['完工时间', '总行驶距离', '负载方差', '响应时间', '理论完工时间']
N = len(categories)

raw = np.array([
    [262.951, 121.142, 178.5907, 54.336, 57.4095],
    [111.364, 162.953,  20.3772, 98.677, 28.5497],
    [ 59.606,  61.609,  57.6659, 32.210, 32.7659],
])
inv  = 1.0 / raw
norm = inv / inv.max(axis=0)

angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
angles += angles[:1]

fig3, ax3 = plt.subplots(figsize=(7, 7), subplot_kw=dict(polar=True))
radar_colors = ['#5B9BD5', '#ED7D31', '#70AD47']
radar_labels = ['KM（匈牙利算法）', '贪心算法', 'MTD-HCGA（本文方法）']

for row, color, label in zip(norm, radar_colors, radar_labels):
    vals = row.tolist() + row[:1].tolist()
    ax3.plot(angles, vals, 'o-', linewidth=2, color=color, label=label)
    ax3.fill(angles, vals, alpha=0.12, color=color)

ax3.set_thetagrids(np.degrees(angles[:-1]), categories, fontsize=12)
ax3.set_ylim(0, 1.05)
ax3.set_yticks([0.25, 0.5, 0.75, 1.0])
ax3.set_yticklabels(['0.25', '0.50', '0.75', '1.00'], fontsize=8)
ax3.set_title('算法综合性能雷达图\n（归一化，越大越优）', fontsize=13, fontweight='bold', pad=20)
ax3.legend(loc='upper right', bbox_to_anchor=(1.35, 1.15), fontsize=10, frameon=True)
plt.tight_layout()
fig3.savefig('算法综合性能雷达图.png', dpi=180, bbox_inches='tight')
print("已保存：算法综合性能雷达图.png")

# ── 图4：折线图（实测 vs 理论完工时间） ──────────────────────────────────────
fig4, ax4 = plt.subplots(figsize=(9, 5))
xpos = np.arange(3)
ax4.plot(xpos, makespan,        'o-',  color='#5B9BD5', linewidth=2.2, markersize=8,
         label='实测完工时间 (s)')
ax4.plot(xpos, t_makespan_theo, 's--', color='#ED7D31', linewidth=2.2, markersize=8,
         label='理论完工时间 (s)')

for xi, (y1, y2) in enumerate(zip(makespan, t_makespan_theo)):
    ax4.annotate(f'{y1:.1f}', (xi, y1), textcoords='offset points', xytext=(0,  8),
                 ha='center', fontsize=10, color='#5B9BD5')
    ax4.annotate(f'{y2:.1f}', (xi, y2), textcoords='offset points', xytext=(0, -16),
                 ha='center', fontsize=10, color='#ED7D31')

ax4.set_xticks(xpos)
ax4.set_xticklabels(['KM（匈牙利算法）', '贪心算法', 'MTD-HCGA（本文方法）'], fontsize=11)
ax4.set_ylabel('时间 (s)', fontsize=12)
ax4.set_title('实测完工时间与理论完工时间对比', fontsize=13, fontweight='bold')
ax4.legend(fontsize=11, frameon=True)
ax4.yaxis.grid(True, linestyle='--', alpha=0.6)
ax4.set_axisbelow(True)
ax4.spines['top'].set_visible(False)
ax4.spines['right'].set_visible(False)
plt.tight_layout()
fig4.savefig('完工时间实测与理论对比.png', dpi=180, bbox_inches='tight')
print("已保存：完工时间实测与理论对比.png")

print("\n全部图表已生成完毕。")

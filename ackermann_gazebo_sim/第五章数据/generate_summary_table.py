"""
三种算法多指标对比总表（输出为 PNG 图片，适合论文插图）
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

fm.fontManager.addfont(_FONT_CACHE)
plt.rcParams['font.sans-serif'] = [_FONT_NAME, 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ── 表格数据 ──────────────────────────────────────────────────────────────────
col_labels = ['指标', 'KM（匈牙利算法）', '贪心算法', 'MTD-HCGA（本文方法）', '最优算法']

rows = [
    ['完工时间 (s)',       '262.951', '111.364', '59.606',  'MTD-HCGA ↓'],
    ['总行驶距离 (m)',     '121.142', '162.953', '61.609',  'MTD-HCGA ↓'],
    ['负载方差 (m²)',      '178.591', '20.377',  '57.666',  '贪心 ↓'],
    ['战略响应时间 (s)',   '54.336',  '98.677',  '32.210',  'MTD-HCGA ↓'],
    ['车辆1行驶距离 (m)', '23.686',  '50.942',  '9.967',   'MTD-HCGA ↓'],
    ['车辆2行驶距离 (m)', '41.056',  '51.313',  '27.467',  'MTD-HCGA ↓'],
    ['车辆3行驶距离 (m)', '56.400',  '60.698',  '24.176',  'MTD-HCGA ↓'],
    ['理论完工时间 (s)',   '57.410',  '28.550',  '32.766',  '贪心 ↓'],
    ['理论总距离 (m)',     '117.986', '67.694',  '88.945',  '贪心 ↓'],
    ['对齐效率 η',        '-0.3157', '0.4747',  '-0.1508', '贪心 ↑'],
]

best_col = [3, 3, 2, 3, 3, 3, 3, 2, 2, 2]

# ── 绘图 ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 7))
ax.axis('off')

table = ax.table(
    cellText=rows,
    colLabels=col_labels,
    cellLoc='center',
    loc='center',
)
table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1, 2.1)

header_color = '#2E4057'
for j in range(len(col_labels)):
    cell = table[0, j]
    cell.set_facecolor(header_color)
    cell.set_text_props(color='white', fontweight='bold')

row_colors = ['#F2F2F2', '#FFFFFF']
highlight  = '#C6EFCE'
best_text  = '#375623'

for i, (row_data, bc) in enumerate(zip(rows, best_col), start=1):
    for j in range(len(col_labels)):
        cell = table[i, j]
        if j == bc:
            cell.set_facecolor(highlight)
            cell.set_text_props(color=best_text, fontweight='bold')
        elif j == len(col_labels) - 1:
            cell.set_facecolor('#FFF2CC')
            cell.set_text_props(color='#7F6000', fontweight='bold')
        else:
            cell.set_facecolor(row_colors[i % 2])

for i in range(1, len(rows) + 1):
    table[i, 0].set_text_props(fontweight='bold')
    table[i, 0].set_facecolor('#D9E1F2')

legend_items = [
    mpatches.Patch(facecolor=highlight, edgecolor='grey', label='该行最优值（绿色高亮）'),
    mpatches.Patch(facecolor='#FFF2CC', edgecolor='grey', label='最优算法标注列'),
    mpatches.Patch(facecolor='#D9E1F2', edgecolor='grey', label='指标名称列'),
]
ax.legend(handles=legend_items, loc='lower center', ncol=3,
          fontsize=10, frameon=True, bbox_to_anchor=(0.5, -0.06))

ax.set_title('表5-X  三种任务分配算法多指标性能对比总表',
             fontsize=14, fontweight='bold', pad=16)

plt.tight_layout()
fig.savefig('算法多指标对比总表.png', dpi=180, bbox_inches='tight')
print("已保存：算法多指标对比总表.png")

"""
合并 DejaVu Sans（ASCII/Latin）和 Droid Sans Fallback（CJK）
生成 merged_font.ttf，供 matplotlib 使用
"""
from fontTools.ttLib import TTFont

dejavu_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
droid_path  = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
out_path    = '/home/wang/Desktop/wk_space/multi-ackermann_gazebo_sim/ackermann_gazebo_sim/第五章数据/merged_font.ttf'

dv = TTFont(dejavu_path)
dr = TTFont(droid_path)

dv_cmap = dv.getBestCmap()
dr_cmap = dr.getBestCmap()

# 只取 Droid 里 DejaVu 没有的字符（CJK 等）
new_mappings = {cp: gname for cp, gname in dr_cmap.items() if cp not in dv_cmap}
print(f'New glyphs to add: {len(new_mappings)}')

dv_glyf  = dv['glyf']
dr_glyf  = dr['glyf']
dv_hmtx  = dv['hmtx']
dr_hmtx  = dr['hmtx']
dv_order = dv.getGlyphOrder()

added = 0
for cp, gname in new_mappings.items():
    if gname in dr_glyf and gname not in dv_glyf.glyphs:
        dv_glyf[gname] = dr_glyf[gname]
        if gname in dr_hmtx.metrics:
            dv_hmtx.metrics[gname] = dr_hmtx.metrics[gname]
        dv_order.append(gname)
        added += 1

dv.setGlyphOrder(dv_order)
print(f'Glyphs actually added: {added}')

for table in dv['cmap'].tables:
    if table.format == 4:
        table.cmap.update(new_mappings)
        break

dv.save(out_path)
print(f'Saved: {out_path}')

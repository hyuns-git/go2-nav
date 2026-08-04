#!/usr/bin/env python3
"""맵에 사각형 금지구역을 검정으로 칠한다 (책상 밑, 계단 등).

Foxy 에는 Keepout Filter 가 없으므로 맵 이미지에 직접 그리는 것이 유일한 방법.

사용:
  python3 block_area.py ~/maps/office.yaml office_edit.pgm  x1 y1 x2 y2 [x1 y1 x2 y2 ...]

좌표는 map 프레임의 미터. RViz 의 Publish Point 도구로 얻는다:
  ros2 topic echo /clicked_point

원본은 덮어쓰지 않는다.
"""
import os
import sys

import numpy as np
import yaml
from PIL import Image

src_yaml, out_name = sys.argv[1], sys.argv[2]
vals = [float(v) for v in sys.argv[3:]]
if not vals or len(vals) % 4 != 0:
    sys.exit("좌표는 4개씩(x1 y1 x2 y2) 주세요")

info = yaml.safe_load(open(src_yaml))
base = os.path.dirname(os.path.abspath(src_yaml))
img = np.array(Image.open(os.path.join(base, os.path.basename(info['image']))))
res = info['resolution']
ox, oy = info['origin'][0], info['origin'][1]
H, W = img.shape


def to_px(x, y):
    return int((x - ox) / res), int(H - 1 - (y - oy) / res)


for i in range(0, len(vals), 4):
    x1, y1, x2, y2 = vals[i:i + 4]
    c1, r1 = to_px(x1, y1)
    c2, r2 = to_px(x2, y2)
    c0, c9 = sorted((max(0, min(W - 1, c1)), max(0, min(W - 1, c2))))
    r0, r9 = sorted((max(0, min(H - 1, r1)), max(0, min(H - 1, r2))))
    img[r0:r9 + 1, c0:c9 + 1] = 0
    print("막음: (%.2f,%.2f)-(%.2f,%.2f) -> px rows[%d:%d] cols[%d:%d]"
          % (x1, y1, x2, y2, r0, r9, c0, c9))

out_pgm = os.path.join(base, out_name)
Image.fromarray(img).save(out_pgm)
info['image'] = out_name
info.pop('mode', None)          # Foxy map_server 는 mode 키를 못 읽음
with open(out_pgm.replace('.pgm', '.yaml'), 'w') as f:
    yaml.safe_dump(info, f)
print("저장:", out_pgm)

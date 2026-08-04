#!/usr/bin/env python3
"""저장된 맵의 품질을 정량 평가.

사용: python3 eval_map.py ~/maps/office.pgm

벽 평균 이웃수가 가장 중요한 지표.
  2.0~2.6 = 얇고 깔끔 (루프 폐합 성공)
  3.0+    = 이중벽 의심 (루프 폐합 실패)
"""
import sys
import numpy as np
from PIL import Image

p = sys.argv[1]
im = np.array(Image.open(p))
occ = (im < 100).sum()
free = (im > 200).sum()
unk = im.size - occ - free
print("파일:", p, im.shape)
print("  점유 %.2f%%  자유 %.2f%%  미지 %.2f%%"
      % (100 * occ / im.size, 100 * free / im.size, 100 * unk / im.size))
r = occ / max(free, 1)
print("  점유/자유 = %.4f  (정상 0.02~0.06)" % r)

o = (im < 100)
nb = np.zeros_like(o, dtype=np.uint8)
nb[1:, :] += o[:-1, :]
nb[:-1, :] += o[1:, :]
nb[:, 1:] += o[:, :-1]
nb[:, :-1] += o[:, 1:]
t = nb[o].mean() if o.any() else 0
print("  벽 평균 이웃수 %.2f  (2.0~2.6 얇음 / 3.0+ 이중벽 의심)" % t)
print("  판정:", "양호" if (0.015 < r < 0.07 and t < 3.0) else "재확인 필요")
print()
print("  ※ 미지 비율이 0에 가까우면 점유/자유 비율이 높게 나옵니다.")
print("    벽 평균 이웃수가 정상이면 문제 없습니다.")

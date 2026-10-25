"""
Project Eye v5 - yuz takibi v2 (panelde "Simulasyon v2").

face_follow.py ile ayni; tek fark hedef secimi: yeni bir yuze gecince en az
1 sn o yuzde kalir, sonra yine en buyuk yuze gecebilir. Iki kisi ekrandayken
gozun sag-sol titremesini onler. Hedef kadrajdan cikarsa beklemeden kalana gecer.
Yuz kaybolunca canli bakinma gozun durdugu yerden baslar (uzak noktaya sicramaz).
Kilit dosyasi ortak: v1 ile ayni anda calismaz (ayni kamera + seri port).
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import face_follow as ff  # noqa: E402

ff.TARGET_HOLD_S = 1.0
ff.WANDER_FROM_GAZE = True
ff.WINDOW_NAME = "Project Eye v5 - Face Follow v2"

if __name__ == "__main__":
    sys.exit(ff.main())

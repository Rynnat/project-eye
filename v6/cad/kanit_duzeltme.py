"""Duzeltme turu 1 kaniti: yeni kontroller (agent-flow/checks/hafiflik.py, belge.py) DUZELTME ONCESI belgelerde/modelde
hatayi YAKALIYOR, DUZELTME SONRASI geciyor mu?  -> out/kanit_duzeltme.txt
  PYTHONUTF8=1 python -B kanit_duzeltme.py
Duzeltme oncesi kopyalar: out/tur0/ (MONTAJ.md, BOM_FARK.md, params_v6.py, kama.stl - aşama 2 ciktisi, md5 23a58031).
"""
import os, sys
sys.dont_write_bytecode = True
import check as C          # sys.path (agent-flow) + E6 + yardimcilar
from check import E6, K6
from params_v6 import *
from checks import rapor_metni
from checks.hafiflik import kontrol_hafiflik
from checks.belge import kontrol_adim_sirasi, kontrol_sikma_talimati, kontrol_kor_delik, kontrol_bom_belge_envanter

HERE = os.path.dirname(os.path.abspath(__file__)); V6 = os.path.normpath(os.path.join(HERE, ".."))
T0 = os.path.join(HERE, "out", "tur0")
oku = lambda p: open(p, encoding="utf-8").read()
env = oku(C.ENVANTER_MD)
L = []
for etiket, md_dir, kama_args, hedef_fn in (
        ("ONCE (asama 2 / tur0)", T0, dict(duvar=2.4, deri=2.0, nervur=2.0, nervur_z=(-40.0, -15.0, 8.0)), lambda boy: 5.5),
        ("SONRA (duzeltme turu 1)", V6, dict(), lambda boy: None if ADP_PILOT_DERIN >= boy else ADP_PILOT_DERIN)):
    S = []
    a = dict(kama_args)
    v = E6.make_kama(**a).Volume()
    a.update(duvar=MIN_ET, deri=MIN_ET, nervur=MIN_ET)
    v0 = E6.make_kama(**a).Volume()
    S.append(kontrol_hafiflik({"kama (yari)": dict(hacim=v, hacim_min_et=v0)}, oran_maks=KAMA_HAFIF_ORAN, ad="D4_kama_hafiflik"))
    m = oku(os.path.join(md_dir, "MONTAJ.md"))
    S.append(kontrol_adim_sirasi(m, C.KALIBRASYON_SIRASI, ad="D2_kalibrasyon_sirasi"))
    S.append(kontrol_sikma_talimati(m, ad="D7_sikma_talimati"))
    P = E6.build(verbose=False); allp = {n: p for n, p in P.items() if p.kind != "visual"}
    C.QUICK = True
    boy, enb, yer, npoz = C.matkap_olc(allp, C.pozlar(allp))
    S.append(kontrol_kor_delik([dict(ad="frame_L serit O1.6", malzeme_boyu=boy, hedef=hedef_fn(boy), arka_kesisim=enb,
                                     el_hassasiyet=MATKAP_EL_HASSASIYET)], ad="D6_frame_L_matkap"))
    S.append(kontrol_bom_belge_envanter(oku(os.path.join(md_dir, "BOM_FARK.md")), env, C.BOM_ENVANTER_KALEM, ad="D8_bom_envanter"))
    L.append(f"################ {etiket}\n" + rapor_metni(S))
open(os.path.join(HERE, "out", "kanit_duzeltme.txt"), "w", encoding="utf-8").write("\n".join(L))
print("\n".join(l for blk in L for l in blk.splitlines() if l.startswith(("####", "SONUC", "==", "   -"))))

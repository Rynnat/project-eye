"""Project Eye v5 - dugmeli kalibrasyon paneli (kullanici istegi 2026-09-30).

    ..\\..\\..\\..\\.venv-ctl\\Scripts\\pythonw.exe kalibrasyon_paneli.py [--port COM3]

Karti kalibrasyon moduna alir ("C 1": dogal hareket kapali, sinirlar her yone 20 derece genis),
dugmelerle goz sag-sol / yukari-asagi ve kapaklari oynatir, bulunan acilari control/kalibrasyon.json'a kaydeder.
Kapaninca "C 0" ile normal mekanik sinirlara doner. Kaydedilen degerler firmware'e ELLE (Claude) islenir.
Yonler modelden: yaw servo acisi artinca gozler (robota karsidan bakana gore) SOLA, pitch artinca YUKARI,
kapak acisi artinca KAPANIR. Ters ise panelde gorursun; kayitlar aci degeri oldugu icin yine dogru olur.
"""
import argparse
import json
import os
import time
import tkinter as tk
from tkinter import ttk

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
KAYIT = os.path.join(HERE, "kalibrasyon.json")
NOTR = [90.0, 90.0, 82.6]


class Kart:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.5)
        t0, b = time.time(), b""
        while time.time() - t0 < 4:
            b += self.s.read(100)
            if b"READY" in b:
                break
        self.hazir = b"READY" in b

    def komut(self, x):
        self.s.reset_input_buffer()
        self.s.write((x + "\n").encode())
        r = self.s.readline().decode(errors="replace").strip()
        if r.startswith("ERR"):                      # seri parazit: bir kez tekrar
            self.s.write((x + "\n").encode())
            r = self.s.readline().decode(errors="replace").strip()
        return r

    def kapat(self):
        try:
            self.komut("C 0")
        finally:
            self.s.close()


class Panel:
    def __init__(self, kok, kart):
        self.k, self.kart = kok, kart
        self.aci = list(NOTR)
        self.kayit = {}
        try:
            with open(KAYIT, encoding="utf-8") as f:
                self.kayit = json.load(f)
        except Exception:
            pass
        kok.title("Project Eye v5 - Kalibrasyon")
        kok.configure(bg="#1A0A2E")
        st = ttk.Style(kok)
        st.theme_use("clam")
        st.configure(".", background="#1A0A2E", foreground="#F4D5FD", font=("Segoe UI", 11))
        st.configure("TButton", background="#3a2560", foreground="#F4D5FD", padding=8, borderwidth=0)
        st.map("TButton", background=[("active", "#00F0FF")], foreground=[("active", "#1A0A2E")])
        st.configure("Kayit.TButton", background="#24123D", font=("Segoe UI", 9), padding=4)
        st.configure("Deger.TLabel", font=("Consolas", 18, "bold"), foreground="#00F0FF")
        st.configure("Baslik.TLabel", font=("Segoe UI", 12, "bold"))
        st.configure("Kucuk.TLabel", font=("Segoe UI", 9), foreground="#a58bc0")

        cer = ttk.Frame(kok, padding=14)
        cer.grid(sticky="nsew")
        ust = ttk.Frame(cer)
        ust.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))
        ttk.Label(ust, text="Adım:").pack(side="left")
        self.adim = tk.DoubleVar(value=1.0)
        for d in (0.5, 1.0, 2.0, 5.0):
            ttk.Radiobutton(ust, text=f"{d:g}°", value=d, variable=self.adim).pack(side="left", padx=4)

        self.etiket = []
        satirlar = [
            ("Göz sağ-sol", 0, ("◀ Sol", +1), ("Sağ ▶", -1), [("Merkez", "yaw_merkez"), ("Sol sınır", "yaw_sol"), ("Sağ sınır", "yaw_sag")]),
            ("Göz yukarı-aşağı", 1, ("▲ Yukarı", +1), ("▼ Aşağı", -1), [("Merkez", "pitch_merkez"), ("Yukarı sınır", "pitch_yukari"), ("Aşağı sınır", "pitch_asagi")]),
            ("Kapaklar", 2, ("Aç", -1), ("Kapat", +1), [("Tam açık", "kapak_acik"), ("Tam kapalı", "kapak_kapali")]),
        ]
        for i, (ad, ch, (t1, y1), (t2, y2), kayitlar) in enumerate(satirlar):
            r = 1 + i * 2
            ttk.Label(cer, text=ad, style="Baslik.TLabel").grid(row=r, column=0, sticky="w", pady=(8, 0))
            ttk.Button(cer, text=t1, width=10, command=lambda ch=ch, y=y1: self.oynat(ch, y)).grid(row=r, column=1, padx=4, pady=(8, 0))
            lb = ttk.Label(cer, text="", style="Deger.TLabel", width=7, anchor="center")
            lb.grid(row=r, column=2, pady=(8, 0))
            self.etiket.append(lb)
            ttk.Button(cer, text=t2, width=10, command=lambda ch=ch, y=y2: self.oynat(ch, y)).grid(row=r, column=3, padx=4, pady=(8, 0))
            kf = ttk.Frame(cer)
            kf.grid(row=r + 1, column=0, columnspan=4, sticky="w")
            ttk.Label(kf, text="bu açıyı kaydet:", style="Kucuk.TLabel").pack(side="left", padx=(0, 6))
            for yazi, anahtar in kayitlar:
                ttk.Button(kf, text=yazi, style="Kayit.TButton", command=lambda ch=ch, a=anahtar: self.kaydet(ch, a)).pack(side="left", padx=2)

        alt = ttk.Frame(cer)
        alt.grid(row=8, column=0, columnspan=4, sticky="w", pady=(14, 4))
        ttk.Button(alt, text="Kırpma testi", command=self.kirp).pack(side="left", padx=3)
        ttk.Button(alt, text="Nötr", command=self.notr).pack(side="left", padx=3)
        ttk.Button(alt, text="Serbest bırak", command=lambda: self.durum(self.kart.komut("D"))).pack(side="left", padx=3)

        self.kayit_lb = ttk.Label(cer, text="", style="Kucuk.TLabel", justify="left")
        self.kayit_lb.grid(row=9, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.durum_lb = ttk.Label(cer, text="", style="Kucuk.TLabel")
        self.durum_lb.grid(row=10, column=0, columnspan=4, sticky="w")
        ttk.Label(cer, text="Kısayol: ← → sağ-sol · ↑ ↓ yukarı-aşağı · PgUp aç / PgDn kapat", style="Kucuk.TLabel").grid(row=11, column=0, columnspan=4, sticky="w", pady=(6, 0))

        kok.bind("<Left>", lambda e: self.oynat(0, +1)); kok.bind("<Right>", lambda e: self.oynat(0, -1))
        kok.bind("<Up>", lambda e: self.oynat(1, +1)); kok.bind("<Down>", lambda e: self.oynat(1, -1))
        kok.bind("<Prior>", lambda e: self.oynat(2, -1)); kok.bind("<Next>", lambda e: self.oynat(2, +1))
        kok.protocol("WM_DELETE_WINDOW", self.cik)

        self.durum(f"A 0 {self.kart.komut('A 0')} · kalibrasyon modu {self.kart.komut('C 1')} (sınırlar ±20°)")
        self.gonder()
        self.kayit_goster()

    def gonder(self):
        r = self.kart.komut("S %.1f %.1f %.1f" % tuple(self.aci))
        for lb, a in zip(self.etiket, self.aci):
            lb.configure(text=f"{a:.1f}°")
        if r != "OK":
            self.durum(f"kart cevabı: {r!r}")
        q = self.kart.komut("?")
        if q.startswith("STATE"):
            gercek = [float(x) for x in q.split()[1:4]]
            if any(abs(g - a) > 0.05 for g, a in zip(gercek, self.aci)):
                self.durum(f"kart sınırda: istenen {self.aci} → uygulanan {gercek}")
                self.aci = gercek
                for lb, a in zip(self.etiket, self.aci):
                    lb.configure(text=f"{a:.1f}°")

    def oynat(self, ch, yon):
        self.aci[ch] = round(self.aci[ch] + yon * self.adim.get(), 2)
        self.gonder()

    def kaydet(self, ch, anahtar):
        self.kayit[anahtar] = self.aci[ch]
        self.kayit["_zaman"] = time.strftime("%Y-%m-%d %H:%M")
        with open(KAYIT, "w", encoding="utf-8") as f:
            json.dump(self.kayit, f, indent=2, ensure_ascii=False)
        self.durum(f"kaydedildi: {anahtar} = {self.aci[ch]:.1f}° → kalibrasyon.json")
        self.kayit_goster()

    def kayit_goster(self):
        ks = {k: v for k, v in self.kayit.items() if not k.startswith("_")}
        self.kayit_lb.configure(text="Kayıtlı: " + (", ".join(f"{k} {v:.1f}°" for k, v in ks.items()) or "henüz yok"))

    def kirp(self):
        kapali = self.kayit.get("kapak_kapali", 102.3)
        once = self.aci[2]
        self.kart.komut("S %.1f %.1f %.1f" % (self.aci[0], self.aci[1], kapali))
        self.k.after(200, lambda: (self.kart.komut("S %.1f %.1f %.1f" % (self.aci[0], self.aci[1], once)), self.durum(f"kırpma: {once:.1f}° → {kapali:.1f}° → {once:.1f}° (200 ms)")))

    def notr(self):
        self.aci = [self.kayit.get("yaw_merkez", 90.0), self.kayit.get("pitch_merkez", 90.0), NOTR[2]]
        self.gonder()

    def durum(self, t):
        self.durum_lb.configure(text=t)

    def cik(self):
        try:
            self.kart.kapat()
        finally:
            self.k.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    a = ap.parse_args()
    kart = Kart(a.port)
    kok = tk.Tk()
    Panel(kok, kart)
    kok.mainloop()


if __name__ == "__main__":
    main()

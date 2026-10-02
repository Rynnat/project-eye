# Project Eye v5 — montaj sırası

Bu sıra `cad/check_montaj.py` ile **vida vida** denetlenir: her vidanın kafası kendi ekseni boyunca, o adıma kadar takılı
parçalara çarpmadan yerine sürülebiliyor ve arkasında ≥ 30 mm düz tornavida yolu var (menteşe vidaları hariç, bkz. 4).
Sonuç (2026-09-30): **35 vida, 0 imkânsız, 0 tornavida engeli**, 3 uyarı (M3 pan kafa servo gövdesine ~0.15 mm biniyor).
Göz alt montajının kafeslere arkadan kayması ayrıca 0–100 mm boyunca taranıp serbest bulundu (kapak açık ve kapalı).

**Takım:** yıldız tornavida (≥ 30 mm düz yol), **kısa uç** (≤ 20 mm, 1/4" uç parmakla ya da kısa uç tutucu — menteşe vidaları
için), küçük tornavida (servo kolu vidaları), pense/cımbız (somunlar). İsteğe bağlı: vida sabitleyici, PLA yapıştırıcısı.

**Genel kurallar**
- Somunlar parçalar ayrıyken cebine konur (oturmazsa havyayla ısıtıp bastır).
- Servo kulak vidası (M3 + somun) her serVoda **milden UZAK** kulaktan (mile yakın uçta kablo yarığı var).
- Servo kolu (çift kol) önce basılı parçaya küçük vidalarla vidalanır, **sonra** servo miline geçirilir.

## Adımlar

1. **Taban:** `base_L` ile `base_R`'yi ortadaki 40 mm bindirmeden yapıştır (sağ yarının alt dili, sol yarının üst dilinin altına).
2. **Pitch servosu → taban:** servoyu `base_R`'deki tutucuya tak; M3×10 + somun, milden uzak kulak.
3. **Pitch braketi → taban:** alttan 2× M4×16 havşa (somunlar braket ayağında).
4. **GÖZ ALT MONTAJI (kafeslerin dışında):**
   - iris + bebek gözlere bastır;
   - her göz kolunu M3×10 ile göze kilitle (alttan; somun gözün içinde);
   - kol ucu M4 somunlarını kolların uç ceplerine koy (cep üstten açık — göz üstüne gelince somun hapsolur);
   - bağlantı çubuğunu (`coupler`) iki kolun ucuna **alttan** M4×10 ile vidala (eklem serbest kalana kadar sık).
   Artık iki göz + iki kol + bağlantı çubuğu tek parça.
5. **Kafes L — kapaklar (göz YOKKEN):** alt + üst kapak göbeklerini plakaya hizala; 2× M4×16 havşa **göz tarafından**
   (kafesin içinden) üst göbeğin havşasından geçir, plakadaki somuna vidala. Karşı kutup yaklaşık 40 mm ötede:
   **kısa uç** kullan. Kapaklar serbest dönene kadar geri al (+ vida sabitleyici).
6. **Kapak lamaları (sol kafeste, göz ve sağ kafes YOKKEN):**
   - üst lamayı U pimine: M4×10, **sol taraftan** (kafes içinden), üst kapak kolundaki kanaldan geçerek; vida kapak tırnağına kendi dişini açar;
   - alt lamayı L pimine: M4×10 (+ alt kapak tırnağındaki somun).
   Lamaların krank uçları **boşta** kalır (sonra A pimi).
7. **Kafes R:** pitch servo kolunu dış plakaya küçük vidalarla vidala (**servo yokken**).
8. **Kafes R — kapaklar (göz yokken):** 5. adım gibi.
9. **Kafesleri yan yana koy — kapak bindirme vidaları (gözlerden ÖNCE, kapaklar KAPALI):** üst bindirme 1× M3×10,
   alt bindirme 2× M3×10, dışarıdan; somunlar sol kapakların iç katında. İsteğe bağlı: üst bindirme yüzeyine yapıştırıcı.
10. **Gözler:** 4. adımdaki alt montajı **iki kafese birlikte arkadan** kaydır. Üst göz pimleri üstten (M4×16, somun
    köprüde), alt pimler alttan kirişteki erişim deliğinden (M4×16, somun alt köprünün üst yüzünde).
11. **Kapak servosu → kanat (gözlerden SONRA):** lamaları yukarı çevirip yolu aç; servoyu gövde altı önde, kanadın cebine
    geçir (kulaklar kanadın iç yüzüne oturur); M3×10 + somun, milden uzak kulak.
12. **Kapak krankı alt montajı:** servo kolunu krank'a vidala (servo yokken); A pimi somununu krank göbeğine koy.
13. **Kapak krankı → servo** (kapalıyken 102.3°); iki lamanın ucunu krank ucuna M4×16 havşa ile bağla (A pimi).
14. **Yaw servosu → yaw_mount:** M3×10 + somun, milden uzak kulak.
15. **Yaw krankı alt montajı:** servo kolunu kranka vidala (servo yokken); krank ucu somununu cebe koy.
16. **yaw_mount → kafes altı:** kirişin altından 2× M4×16 havşa (somunlar bacaklarda).
17. **Yaw krankı → servo** (90°), krank ucunu bağlantı çubuğuna M4×10 ile vidala.
18. **Çerçeve → taban:** `frame_R`'nin pitch kolunu pitch servo miline geçir; sol pimi (M4×10) dıştan brakete, somun yatakta.
19. **Maske (en son):** alttan 2× M4×16 havşa.

## Denetim
```
cd v5/cad
PYTHONUTF8=1 ../../../../.venv-cad/Scripts/python.exe check_montaj.py            # sıralı (bu belge)
PYTHONUTF8=1 ../../../../.venv-cad/Scripts/python.exe check_montaj.py --detay    # adım adım her vida
PYTHONUTF8=1 ../../../../.venv-cad/Scripts/python.exe check_montaj.py --takili   # her şey takılıyken (bilgi)
```
Denetlenmeyenler: somunların cebe konma yolu, kabloların yönlendirilmesi, 11. adımda lamaların yukarı çevrildiği poz
(lamalar hariç tutularak tarandı; krank uçları boşta olduğu için pimleri etrafında döner), elle tutma/görme zorluğu.

# Modèles 3D / bibliothèques composants — téléchargés le 2026-10-13

## Contenu
| Dossier | Fichiers | Source | Licence | Usage Fusion |
|---|---|---|---|---|
| ESP32_Espressif/3D | ESP32-S3-WROOM-1, S3-WROOM-1U, WROOM-32E, WROOM-32UE (.STEP) | github.com/espressif/kicad-libraries | CC-BY-SA 4.0 + dérogation designs électroniques | Insérer le STEP (voie A) |
| TFT_Adafruit_2090 | .f3d, .step, .stl, .jpg | github.com/adafruit/Adafruit_CAD_Parts | MIT | Ouvrir le .f3d ou insérer le STEP |
| TFT_Adafruit_EAGLE | .brd/.sch 2.8" (1770 v2, 2090 rev D) | github.com/adafruit/*-PCB | CC BY-SA | Ouvrir dans Fusion Electronics |
| GPS_NEO6M_KiCad | ublox_NEO.kicad_mod (empreinte puce nue) | KiCad/kicad-footprints | CC-BY-SA 4.0 + exception | Référence 2D (format KiCad, non importable tel quel) |

## Attention
- WROOM-32E ≠ WROOM-32 simple (même boîtier 18 x 25,5 x 3,1 mm, puce différente) : vérifier à la datasheet.
- Le STEP S3-WROOM-1 couvre toutes les variantes N4/N8/N16/R2/R8 (même taille).
- Non trouvés en libre : GY-GPS6MV2 (carte), Ra-02, TFT « carte rouge » -> à modéliser (cotes dans le rapport) ou SnapMagic (compte requis).
- Aucune vérification des cotes dans Fusion n'a encore été faite.

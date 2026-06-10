# DH Transformationen Web-App

Statische Web-App zur Berechnung und Visualisierung von Transformationsmatrizen nach DH-Konvention.

## Funktionen

- Dynamische Anzahl von Gelenken
- Revolute und prismatische Gelenke
- Klassische und modifizierte DH-Konvention
- Berechnung von Einzelmatrizen und Gesamtmatrix `T0,n`
- 3D-Visualisierung der Gelenke, Links und Koordinatenframes
- Slider und Play/Pause zur Simulation der Vorwaertskinematik

## GitHub Pages

1. Dateien in ein GitHub-Repository hochladen.
2. In GitHub unter `Settings -> Pages` als Source den Branch mit `index.html` auswaehlen.
3. Die erzeugte Pages-URL oeffnen, zum Beispiel:

```text
https://DEIN-NAME.github.io/DEIN-REPO/
```

## Google Sites Einbettung

In Google Sites:

1. `Einfuegen -> Einbetten`
2. Reiter `URL`
3. GitHub-Pages-URL einfuegen
4. Groesse des eingebetteten Bereichs anpassen

Die App nutzt Three.js ueber CDN. Fuer die 3D-Visualisierung braucht der Browser deshalb Internetzugriff.

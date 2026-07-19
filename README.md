# KA Destruction Suite – Gemeinsame Entwicklungsdokumentation

**Dokumentationsstand:** 19. Juli 2026  
**Geltungsbereich:** KA Simulation Core, KA Fracture, KA Rigid Dynamics, KA Particles, KA Volumes und KA Destruction Suite

Diese README ist das gemeinsame Entwicklungs- und Architektur-Dokument der gesamten KA-Simulationsfamilie. Dieselbe Datei soll in jedem Teil-Add-on mitgeführt werden, damit Zielarchitektur, Schnittstellen, Entwicklungsreihenfolge und Forschungsgrundlage nicht auseinanderlaufen.

> **Wichtige Abgrenzung:** Die einzelnen Add-ons werden zunächst unabhängig entwickelt, installiert und getestet. Das abschließende Sammel-Add-on koordiniert sie zu einem Gesamtworkflow. Es soll die Quellcodes nicht unkontrolliert zu einem Monolithen verschmelzen. Eine spätere gemeinsame Suite-Auslieferung kann alle Module bündeln, intern bleiben die Verantwortlichkeiten und Schnittstellen jedoch getrennt.

> **Aktueller Implementierungsstand:** KA Rigid Dynamics 0.7.6 enthält `SimulationScene v1`, persistente Body-/Collider-IDs, Jolt/Culverin für Rigid Bodies, eine native-freie Windows-Compound-Zerlegung sowie absturzisoliertes CoACD auf anderen Systemen und einen binären Transform-Cache. Ein optionaler ABI-v2-Bridge für Jolt 5.6.0 erzeugt echte Compound-Shapes; ohne kompilierte Bridge bleibt Culverin automatisch als kompatibler Fallback aktiv. PhysX, Blast, PBD-Partikel, NVIDIA Flow, Rauch und Feuer sind noch nicht implementiert. KA Fracture wird als eigenständiges Add-on entwickelt. KA Simulation Core, KA Particles, KA Volumes und das koordinierende Suite-Add-on sind geplante beziehungsweise aufzubauende Module.

In jedem Einzel-Add-on bleiben `ARCHITECTURE.md` und `CHANGELOG.md` modulspezifisch. Diese gemeinsame `README.md` beschreibt dagegen das Gesamtsystem und soll möglichst identisch gehalten werden.

### Implementiert in KA Rigid Dynamics 0.7.6

- `SimulationScene v1` als solverneutraler Szenenvertrag und Quelle des Jolt-Payloads,
- persistente UUIDs für Szene und Bodies sowie deterministische Collider-/Child-IDs,
- optionaler Jolt-5.6-ABI-v2-Bridge mit einem echten `StaticCompoundShape` pro Compound-Convex-Body,
- automatische Rückfallebene auf Culverin 0.13.2, falls keine kompilierte Bridge verfügbar ist,
- CMake- und Buildskripte für Windows x64 und Linux x64,
- 31 Regressionstests für den ausgelieferten Culverin-Pfad und den neutralen Datenvertrag, einschließlich starrer Compound-Bond-Inseln, exakter Boden-Ruhepose und automatischem Aufwecken durch Einschläge.
- konservative Innenraum-Compound-Proxys unter Windows mit automatischem Erstkontakt-Überlappungswächter,
- exakte Speicherung der ursprünglichen Blender-Transformationen in Cache-Frame 1,
- reduzierte Standardreibung für erkannte Bruchstücke und 1-mm-Penetration-Slop gegen seitliches Niedriggeschwindigkeits-Kleben.
- persistenter solverneutraler Bond-Graph für beliebige aktivierte Mesh-Bodies,
- automatische Proximity-Bond-Erzeugung über Weltkoordinaten und stabile Body-UUIDs,
- starre Compound-Actors für `Rigid`, native Fixed Constraints für `Flexible` sowie kraft- und drehmomentabhängiges Lösen während des Substeps,
- `BOND_BREAK`-Ereignisse und finale Bond-Zustände im binären Simulationscache.
- starrer Standardmodus für intakte Bond-Inseln ohne sichtbare gummiartige Relativbewegung,
- deterministisches Actor-Splitting nach tatsächlicher Trennung des Bond-Graphen,
- mitbewegte Bond-Anker und lokale Kontaktzuordnung innerhalb starrer Compound-Inseln.
- bereits auf verwaltetem Boden abgestellte, geschwindigkeitslose `Rigid`-Inseln starten exakt in ihrer Autorenpose schlafend und werden durch externe Einschläge automatisch geweckt.

Die Quell- und Buildintegration des nativen Bridge ist enthalten. Eine vorkompilierte Bridge-Binärdatei ist nicht Bestandteil dieses Pakets; bis sie separat gebaut und installiert wurde, arbeitet das Add-on mit Culverin und dessen Single-Body-Compound-Fallback.

Breakable Bonds verwenden in 0.7.6 den Culverin-Pfad, weil ABI-v2 noch keine externen Constraints bereitstellt. Die Bruchlast wird aus den Kontaktimpulsen pro Substep geschätzt; echte Jolt-Constraint-Reaktionsimpulse bleiben eine spätere ABI-Erweiterung.
Im Modus `Rigid` besitzt eine intakte Bond-Insel nur einen nativen Actor und daher keine internen Fragmentkontakte. Nach einer topologischen Trennung des Bond-Graphen werden neue Actors beziehungsweise Einzelkörper erzeugt, die wieder normal miteinander kollidieren.
Der Standardmodus `Rigid` hält jede noch zusammenhängende Bond-Insel geometrisch starr. `Flexible` lässt zum Vergleich ausschließlich das native Fixed-Constraint-Netz arbeiten.

---

## 1. Entwicklungs- und Dokumentationsstrategie

Das Gesamtsystem wird nicht als ein einziges großes Add-on begonnen. Die Funktionsbereiche werden zunächst als eigenständige Add-ons entwickelt:

1. **KA Fracture** – Fragmentierung und Destruction-Asset-Erzeugung.
2. **KA Rigid Dynamics** – Rigid Bodies, Kollision, Zusammenhalt, Schaden und Ereignisse.
3. **KA Particles** – Splitter, Granulat und grobe Staubpartikel.
4. **KA Volumes** – feiner Staub, Rauch, Feuer und andere volumetrische Fluideffekte.
5. **KA Destruction Suite** – gemeinsamer Workflow, zentrale UI und koordinierter Bake.

Zusätzlich wird ein kleines gemeinsames Fundament benötigt:

6. **KA Simulation Core** – Datenverträge, IDs, Materialprofile, Ereignisschema, Cache-Metadaten und Versionsprüfung.

Die Einzel-Add-ons müssen auch separat nutzbar bleiben. Das spätere Sammel-Add-on verbindet sie über definierte Schnittstellen und gemeinsame Daten, nicht über kopierte Klassen oder direkte Zugriffe auf interne Implementierungsdetails.

### Regeln für diese gemeinsame README

- Die gemeinsame README wird in allen Modulen mit demselben Dokumentationsstand gespeichert.
- Änderungen an der Gesamtarchitektur werden zuerst in dieser Master-Datei vorgenommen und anschließend in alle Module übernommen.
- Modulspezifische Bedienung, Implementierungsdetails und bekannte Fehler gehören in die jeweilige `ARCHITECTURE.md`, `CHANGELOG.md` oder eine zusätzliche Modul-Dokumentation.
- Noch nicht implementierte Funktionen werden ausdrücklich als geplant markiert.
- Eine Dokumentationsänderung allein erhöht nicht zwingend die Funktionsversion eines Add-ons.

---

## 2. Add-on-Familie und Verantwortlichkeiten

### 2.1 KA Simulation Core

Der Core enthält keine eigentliche Simulation und möglichst keine umfangreiche Benutzeroberfläche. Er definiert die gemeinsamen Verträge:

- stabile Szenen-, Objekt-, Body-, Fragment-, Bond-, Material- und Event-IDs,
- SI-Einheiten und Koordinatenkonventionen,
- `DestructionAsset`-, `SimulationScene`- und `SimulationEvent`-Schemata,
- Material- und Schadensprofile,
- Cache-Manifest und Abhängigkeitsgraph,
- Versions- und Kompatibilitätsprüfung,
- Logging- und Diagnose-Schnittstellen,
- Registrierung verfügbarer Module und Fähigkeiten.

Der Core darf keine zwingende Abhängigkeit von Jolt, PhysX, Blast, CoACD, Flow oder einer bestimmten UI besitzen.

### 2.2 KA Fracture

Zuständig für:

- Point-, Voronoi-, Boolean- und Cut-Fracturing,
- Bruchflächen, Innen-/Außenflächen, Materialien und UVs,
- Rough-, Polish- und High-Resolution-Oberflächen,
- hierarchische Fragmentierung,
- stabile Fragment-IDs,
- Nachbarschafts- und gemeinsame Flächenerkennung,
- Low-Poly-Collision-Proxys,
- Export eines solverunabhängigen Destruction Assets.

KA Fracture berechnet keine eigentliche dynamische Zerstörung. Es erzeugt die Geometrie und die strukturellen Ausgangsdaten.

### 2.3 KA Rigid Dynamics

Zuständig für:

- Rigid-Body-Simulation,
- Jolt und später PhysX als austauschbare Backends,
- Convex Hull, Compound Convex und statische Mesh-Collider,
- Masse, Trägheit, Reibung, Restitution, Dämpfung und CCD,
- Constraints und später Bond-/Blast-basierte Zusammenhaltung,
- Kontakt-, Aufprall-, Gleit-, Trennungs- und Bruchereignisse,
- Transform-, Zustands- und Event-Cache.

KA Rigid Dynamics muss auch mit beliebigen Blender-Objekten ohne KA Fracture funktionieren. Liegt ein standardisiertes Destruction Asset vor, nutzt es dessen Fragment- und Bond-Daten.

### 2.4 KA Particles

Zuständig für:

- sekundäre kleine Rigid-Body-Splitter,
- Sand, Kies, Granulat und grobes Pulver,
- schwere und repräsentative Staubpartikel,
- material- und ereignisabhängige Emission,
- PhysX-PBD oder einen alternativen Partikelsolver,
- Lebensdauer, Sleeping, LOD und Partikelcache,
- manuelle Emitter unabhängig vom Destruction-System.

KA Particles verarbeitet den gemeinsamen Event Stream. Es greift nicht direkt auf interne Jolt- oder PhysX-Klassen von KA Rigid Dynamics zu.

### 2.5 KA Volumes

Arbeitsname für das getrennte Fluid-/Volumen-Add-on. Zuständig für:

- feinen luftgetragenen Staub,
- Rauch,
- Feuer,
- Dichte-, Geschwindigkeits-, Temperatur-, Brennstoff- und Verbrennungsfelder,
- Turbulenz, Auftrieb, Luftwiderstand und Sinkverhalten,
- NVIDIA Flow oder einen alternativen Sparse-Volume-Solver,
- OpenVDB-/NanoVDB-Ausgabe und Volume-Cache.

Echte Flüssigkeiten können später als eigener Bereich oder eigenes Modul ergänzt werden. Staub, Rauch und Feuer werden zunächst als gasförmige beziehungsweise volumetrische Fluide behandelt.

### 2.6 KA Destruction Suite

Das Sammel-Add-on koordiniert die vorhandenen Module:

- gemeinsame Benutzeroberfläche,
- Workflow von Fracture über Rigid Dynamics zu Particles und Volumes,
- Material- und Qualitäts-Presets,
- Modul- und Versionsprüfung,
- gemeinsamer Bake-Plan,
- Cache-Abhängigkeiten und Invalidierung,
- Fortschrittsanzeige, Fehlerprüfung und Diagnoseübersicht,
- Übergabe standardisierter Daten zwischen den Modulen.

Das Suite-Add-on darf fehlende Module erkennen und den verbleibenden Workflow weiterhin nutzbar halten. Beispiel: Fracture und Rigid Dynamics müssen auch ohne Particles und Volumes funktionieren.

---

## 3. Abhängigkeits- und Integrationsmodell

Die gewünschte Abhängigkeitsrichtung lautet:

```text
                    KA Simulation Core
                  gemeinsame Datenverträge
                            ↑
        ┌───────────────────┼───────────────────┐
        │                   │                   │
   KA Fracture       KA Rigid Dynamics    KA Particles
        │                   │                   │
        └────────────── Event/Asset ────────────┤
                                                │
                                           KA Volumes

                    KA Destruction Suite
             koordiniert alle installierten Module
```

Praktisch dürfen alle Funktionsmodule den Core verwenden. Sie sollten sich untereinander möglichst nur über serialisierbare Core-Daten austauschen.

### Erlaubte Kopplung

- KA Fracture schreibt ein `DestructionAsset`.
- KA Rigid Dynamics liest dieses Asset und schreibt `SimulationEvent`- und Rigid-Cache-Daten.
- KA Particles liest Events und schreibt Partikel-Cache-Daten.
- KA Volumes liest Events und optional Partikeldaten und schreibt VDB-Cache-Daten.
- KA Destruction Suite erstellt und überwacht den gesamten Abhängigkeitsgraphen.

### Zu vermeidende Kopplung

- direkte Imports interner UI-Klassen eines anderen Add-ons,
- Zugriff auf private Solverobjekte eines anderen Moduls,
- Datenaustausch ausschließlich über Blender-Objektnamen,
- mehrfach implementierte Material- oder Eventklassen,
- gemeinsame globale Zustände ohne Versionierung,
- Kopieren desselben Solvercodes in mehrere Add-ons.

### Gemeinsames Ereignisbeispiel

```json
{
  "schema": "ka.simulation_event/1",
  "event_id": "event_000042",
  "event_type": "BOND_BREAK",
  "frame": 42,
  "time_seconds": 1.68,
  "position_m": [1.2, 0.4, 2.8],
  "fragment_a": "fragment_0012",
  "fragment_b": "fragment_0013",
  "material_id": "concrete_default",
  "energy_j": 38.5,
  "released_area_m2": 0.027,
  "relative_velocity_m_s": 4.8
}
```

Dasselbe Ereignis kann neue Rigid Bodies aktivieren, Splitter erzeugen, granulare Partikel emittieren, eine Staubwolke speisen und später Audio- oder Kameraeffekte auslösen.

---

## 4. Projektziel und Gesamtpipeline

Das langfristige System soll eine durchgängige Destruction-Pipeline bereitstellen:

1. Meshes kontrolliert in Bruchstücke zerlegen.
2. Render-Geometrie und vereinfachte Kollisionsgeometrie getrennt verwalten.
3. Bruchstücke zunächst als zusammenhängende Struktur behandeln.
4. Zusammenhalt über Material-, Kontaktflächen- und Energieparameter berechnen.
5. Getrennte Fragmente als Rigid Bodies simulieren.
6. Aus Bruch, Aufprall, Trennung und Reibung physikalisch begründete Ereignisse erzeugen.
7. Daraus sekundäre Splitter, granulare Partikel und Staub emittieren.
8. Feinen luftgetragenen Staub als Volumen simulieren.
9. Später Rauch und Feuer mit demselben Ereignis- und Materialsystem ergänzen.
10. Sämtliche Ergebnisse reproduzierbar und Blender-kompatibel cachen.

Die angestrebte Gesamtpipeline lautet:

```text
KA Fracture
Geometrie + Fragmente + Bruchflächen + Proxys
        ↓
KA Simulation Core
Destruction Asset + IDs + Materialien + Bonds
        ↓
KA Rigid Dynamics
PhysX/Blast als geplanter Hauptpfad, Jolt als CPU-Fallback
        ↓
Solverunabhängiger Event Stream
Bruch + Aufprall + Trennung + Reibung + Energie
        ├───────────────────────┐
        ↓                       ↓
KA Particles               KA Volumes
Splitter + Granulat        Staub + Rauch + Feuer
        └───────────┬───────────┘
                    ↓
KA Destruction Suite
Bake-Steuerung + Cache + Playback + Diagnose
```

---

## 5. Statusübersicht der Module

| Modul | Status zum Dokumentationsstand | Hauptaufgabe |
|---|---|---|
| KA Simulation Core | `SimulationScene v1` in Rigid Dynamics implementiert; gemeinsames Paket geplant | gemeinsame Datenverträge und Cache-Schemata |
| KA Fracture | eigenständige aktive Entwicklung | Geometrie, Fragmente und Destruction Asset |
| KA Rigid Dynamics | Version 0.7.6 vorhanden | Jolt-Rigid-Bodies, SimulationScene und Compound Convex |
| KA Particles | geplant | Splitter, Granulat und grober Staub |
| KA Volumes | geplant | feiner Staub, Rauch und Feuer |
| KA Destruction Suite | Abschluss-/Integrationsphase geplant | zentraler Workflow und gemeinsame UI |

### In KA Rigid Dynamics 0.7.6 bereits vorhanden

- Native Jolt-Rigid-Body-Simulation über Culverin 0.13.2.
- Dynamische Convex-Hull-Collider.
- CoACD-basierte Compound-Convex-Collider.
- Statische Triangle-Mesh-Collider.
- Masse über direkten Wert oder Dichte und Volumen.
- Reibung, Restitution, lineare und angulare Dämpfung.
- CCD, Sleeping, adaptive Substeps und frühes Bake-Ende.
- Low-Poly-Collision-Proxys getrennt vom Render-Mesh.
- Persistenter Collider-Cache.
- Direkter binärer Float32-Transform-Cache.
- Kontakt-, Payload- und Side-Stick-Diagnosen.
- Regressionstests für Fall, Stapel, Reibung, CCD, Compound Convex, Cache und Determinismus.

### Im Gesamtsystem noch nicht vorhanden

- ausgelagertes gemeinsames KA-Simulation-Core-Paket über den bereits implementierten `SimulationScene-v1`-Vertrag hinaus,
- vollständiger PhysX-Backendpfad,
- native PhysX-GPU-Rigid-Bodies,
- NVIDIA Blast und Bond-/Support-Graph-Auswertung,
- PhysX-PBD-Partikelsystem,
- gemeinsames Material- und Schadensmodell,
- solverunabhängiger Event Stream,
- volumetrischer Staub,
- NVIDIA Flow,
- Rauch- und Feuerberechnung,
- OpenVDB-/NanoVDB-Ausgabe,
- koordinierendes KA-Destruction-Suite-Add-on.

---

## 6. Grundprinzip: Mehrere gekoppelte Skalen statt eines einzigen Solvers

Ein einzelner Solver sollte nicht gleichzeitig große Fragmente, feine Körner und luftgetragenen Staub als identische Objekte behandeln. Das wäre entweder zu ungenau oder unnötig teuer.

Das Gesamtsystem wird deshalb in vier Größenklassen getrennt:

| Größenklasse | Darstellung | Geplanter Solver |
|---|---|---|
| Große Bruchstücke | Rigid Bodies | PhysX oder Jolt |
| Kleine sichtbare Splitter | kleine Rigid Bodies | PhysX oder Jolt |
| Sand, Kies, grobes Pulver | granulare Partikel | PhysX PBD |
| Feiner Staub, Rauch, Feuer | sparse Volumenfelder | NVIDIA Flow oder eigener Sparse-Volume-Solver |

Die Kopplung erfolgt über Ereignisse, Impulse, Energie und Materialparameter. Nicht jedes Staubkorn muss einzeln mit jedem Rigid Body kollidieren.

---

## 7. KA Fracture: Geometry Authoring und Fracturing

KA Fracture soll weiterhin die visuelle und geometrische Erzeugung der Bruchstücke übernehmen:

- Point-/Voronoi-Fracturing.
- gerichtete Schnitte und Boolean-basierte Brüche.
- Innen- und Außenflächen.
- Bruchmaterialien und UVs.
- Rough-/Polish-Oberflächen.
- hierarchische Unterteilung.
- Render-Geometrie.
- vereinfachte glatte Collision-Proxys.

Die Fracture-Stufe soll künftig nicht nur lose Mesh-Objekte erzeugen, sondern ein vollständiges **Destruction Asset**.

### Fragment-Datensatz

Jedes Fragment benötigt dauerhaft stabile Daten:

```text
Fragment
├─ eindeutige Fragment-ID
├─ Render-Mesh
├─ glattes Ausgangs-Mesh
├─ Collision Proxy
├─ Volumen
├─ Schwerpunkt
├─ Masse
├─ Trägheit
├─ Material-ID
├─ Hierarchiestufe
├─ Parent-/Child-Beziehungen
└─ Liste benachbarter Fragmente
```

### Warum das glatte Ausgangs-Mesh wichtig ist

Die Kollisionsgeometrie sollte möglichst vor Rough Surface, hochauflösender Unterteilung und kleinen Oberflächenverformungen erzeugt werden. Sonst werden visuelle Details zu physikalischer Komplexität und erhöhen:

- Collider-Erzeugungszeit,
- Zahl der Hull-Vertices,
- Zahl der Compound-Teile,
- Kontaktfluktuation,
- Solverlast,
- Risiko von Verhaken und Side-Stick.

---

## 8. Collision-Proxys

### 8.1 Single Convex Hull

Der schnellste und stabilste Standard. Geeignet für nahezu konvexe Fragmente und Massensimulationen.

Nachteile:

- überbrückt Einbuchtungen,
- kann sichtbare Abstände erzeugen,
- kann Kontaktflächen vergrößern,
- kann das Stapelverhalten verändern.

### 8.2 Compound Convex

Mehrere konvexe Teilkörper bilden eine konkave Form näherungsweise nach. Dies ist die bevorzugte präzise Kollisionsform für dynamische Bruchstücke.

KA Rigid Dynamics 0.7.5 verwendet CoACD zur Zerlegung. Mit der optionalen Jolt-5.6-ABI-v2-Bridge werden die Teil-Hulls als ein nativer `StaticCompoundShape` und damit als genau ein Jolt-Body simuliert. Ohne kompilierte Bridge verwendet der gebündelte Culverin-Pfad einen einzigen stabilen Compound-Body aus konservativen, innenliegenden Box-Teilformen; es entstehen keine internen Fixed-Constraints.
Single-Hull-Fallbacks innerhalb einer starren Bond-Insel werden in 0.7.5 durch innenliegende Kugel-Primitive angenähert. Dadurch können veraltete oder größere Source-Bounds nicht mehr unter den Boden ragen und die komplette Insel beim ersten Simulationsschritt anheben.
Seit 0.7.6 starten geschwindigkeitslose starre Bond-Inseln, die bereits auf der verwalteten Bodenebene stehen, direkt schlafend in der unveränderten Autorenpose. Ein externer Einschlag weckt den Compound-Actor automatisch.

Empfohlene Qualitätsstufen:

| Preset | Maximale Teile | Verwendung |
|---|---:|---|
| Fast | 4 | leichte Konkavität |
| Balanced | 8 | Produktionsstandard |
| Accurate | 16 | sichtbare Nahaufnahme |
| Custom | benutzerdefiniert | gezielte Problemlösung |

### 8.3 Triangle Mesh

- Für statische Umgebung sehr genau und sinnvoll.
- Für dynamische Körper solverabhängig und problematisch.
- Dynamisches Mesh-gegen-Mesh ist in Jolt nicht als allgemeiner Produktionsweg geeignet.
- Ein späterer PhysX-Pfad kann zusätzliche Optionen wie GPU-SDF-Collider bereitstellen.

### 8.4 Geplante automatische Auswahl

Ein späterer `Auto`-Modus soll pro Körper entscheiden:

```text
nahezu konvex              → Single Convex Hull
deutlich konkav            → Compound Convex
statische komplexe Umgebung → Triangle Mesh
gezielter PhysX-GPU-Fall    → SDF
```

Die automatische Entscheidung soll auf gemessener Formabweichung, Konkavität, Objektgröße, Sichtbarkeit und erwarteter Kontaktrelevanz beruhen.

---

## 9. Destruction Asset und Bond Graph

Die Nachbarschaft zwischen Fragmenten muss direkt beim Fracturing bestimmt werden. Zwei Fragmente gelten als verbunden, wenn sie eine gemeinsame ursprüngliche Bruchfläche besitzen.

### Bond-Datensatz

```text
Bond
├─ eindeutige Bond-ID
├─ Fragment A
├─ Fragment B
├─ Mittelpunkt der gemeinsamen Fläche
├─ Flächennormale
├─ gemeinsame Fläche
├─ Material-ID
├─ Zugfestigkeit
├─ Druckfestigkeit
├─ Scherfestigkeit
├─ Torsionsfestigkeit
├─ Bruchenergie
├─ aktueller Schaden
└─ Status: intakt / geschädigt / gebrochen
```

### Warum ein Bond Graph besser ist als viele Fixed Constraints

Die naive Variante wäre, jedes Fragment ab Frame 1 als eigenen Rigid Body zu simulieren und alle Nachbarn mit Fixed Constraints zu verbinden. Das führt bei großen Assets zu:

- sehr vielen aktiven Bodies,
- sehr vielen Constraints,
- unnötiger Broadphase- und Solverlast,
- Drift und Instabilität,
- komplexer Massen- und Trägheitsverteilung.

Besser ist ein Inselmodell:

1. Solange Bonds intakt sind, wird eine zusammenhängende Insel als ein Actor behandelt.
2. Schaden verändert zunächst nur den Bond Graph.
3. Erst wenn gebrochene Bonds den Graphen trennen, entstehen neue Actors.
4. Masse, Trägheit, Schwerpunkt und Geschwindigkeit werden für jede neue Insel berechnet.
5. Nur tatsächlich getrennte Inseln erhöhen die Body-Zahl.

NVIDIA Blast ist genau für diese Chunk-, Bond-, Support-Graph- und Actor-Aufteilung vorgesehen und deshalb ein sinnvoller geplanter Baustein.

---

## 10. Gemeinsames Material- und Schadensmodell

Das System benötigt eigene Materialprofile, die unabhängig vom Solver funktionieren.

### Grundparameter

```text
Material Profile
├─ Dichte
├─ Reibung
├─ Restitution
├─ lineare Dämpfung
├─ angulare Dämpfung
├─ Zugfestigkeit
├─ Druckfestigkeit
├─ Scherfestigkeit
├─ Torsionsfestigkeit
├─ Bruchenergie
├─ Schadensakkumulation
├─ Abriebkoeffizient
├─ Staubertrag pro Bruchfläche
├─ Staubertrag pro dissipierter Energie
├─ Partikelgrößenverteilung
├─ Temperaturverhalten
├─ Brennbarkeit
└─ Rauch-/Rußausbeute
```

### Beispielhafte Materialcharakteristik

**Beton**

- hohe Druckfestigkeit,
- geringe Zugfestigkeit,
- mittlere Scherfestigkeit,
- sprödes Versagen,
- starke Staub- und Splitterbildung.

**Glas**

- geringe Schadenstoleranz,
- schnelle Bruchausbreitung,
- scharfe kleine Fragmente,
- geringe schwere Staubmenge.

**Holz**

- anisotrop,
- höhere Festigkeit entlang der Fasern,
- geringere Festigkeit quer zur Faser,
- faserige Splitter und leichter Staub.

**Metall**

- hohe Energieaufnahme,
- plastische Verformung vor Bruch,
- geringe Staubproduktion,
- Bruch erst bei hohen lokalen Energien.

### Schadenskriterien

Ein Bond sollte nicht nur über eine einzige Kraftschwelle brechen. Sinnvoll ist eine kombinierte Bewertung aus:

- Normalspannung,
- Scherspannung,
- Torsion,
- Impuls,
- dissipierter Energie,
- Dauer und Wiederholung der Belastung,
- Bond-Fläche,
- Materialstreuung,
- bestehendem Vorschaden.

Für spröde Materialien ist eine Kombination aus Spannungsgrenze und notwendiger Bruchenergie besonders sinnvoll.

---

## 11. KA Rigid Dynamics: Solverstrategie

### 11.1 Jolt

Jolt bleibt sinnvoll als:

- schneller CPU-Rigid-Body-Solver,
- Fallback ohne NVIDIA-GPU,
- leichter Modus für kleine und mittlere Szenen,
- Referenzsolver für Regression und Vergleiche,
- produktiver Rigid-Body-only-Pfad.

### 11.2 PhysX

PhysX ist als zukünftiger Hauptpfad interessant, weil eine gemeinsame Plattform Folgendes verbinden kann:

- CPU- und GPU-Rigid-Bodies,
- Broadphase und Kontakterzeugung auf der GPU,
- Constraint-Solver,
- Compound- und SDF-Kollision,
- PBD-Partikel,
- FEM-/deformierbare Systeme,
- Kontakt-, Sleep-, Wake- und Constraint-Break-Events,
- direkte GPU-Datenpfade.

PhysX ist nicht automatisch in jeder kleinen Szene schneller. Der GPU-Pfad lohnt sich vor allem bei hoher Body-, Kontakt- oder Partikelzahl. Daher müssen CPU und GPU mit identischen Szenen gemessen werden.

### 11.3 Blast

Blast soll nicht KA Fracture ersetzen. Geplant ist:

- KA Fracture erzeugt Geometrie, Fragment-IDs, Nachbarschaften und Bond-Daten.
- Blast verwaltet Support Graph, Schaden, Bond-Bruch und Actor-Splitting.
- PhysX simuliert die daraus entstehenden Actors.

### 11.4 Warp und Newton

NVIDIA Warp ist als Werkzeug für eigene GPU-Kernels interessant, etwa für:

- Erosion,
- Partikelklassifikation,
- Materialabhängige Emission,
- Raster-/Partikelkopplung,
- benutzerdefinierte Feldoperationen,
- NanoVDB-Verarbeitung.

Newton ist derzeit eher eine Forschungs- und Robotikplattform mit mehreren Solvertypen. Es sollte beobachtet, aber nicht als primäre Produktionsgrundlage angenommen werden.

---

## 12. Solverunabhängiger Event Stream

Der wichtigste eigene Systemteil ist eine neutrale Ereignisschicht zwischen Simulation und Sekundäreffekten.

### Ereignistypen

```text
CONTACT_BEGIN
CONTACT_PERSIST
CONTACT_END
IMPACT
SLIDE
SCRAPE
BOND_DAMAGE
BOND_BREAK
FRAGMENT_SEPARATION
HIGH_ACCELERATION
BODY_SLEEP
BODY_WAKE
PARTICLE_IMPACT
IGNITION
EXTINGUISH
```

### Ereignisdaten

```text
Simulation Event
├─ Zeit und Frame
├─ Ereignistyp
├─ Weltposition
├─ Normale
├─ Body-/Fragment-IDs
├─ Material A und B
├─ relative Normalgeschwindigkeit
├─ relative Tangentialgeschwindigkeit
├─ Kontaktimpuls
├─ effektive Masse
├─ kinetische Energie
├─ dissipierte Energie
├─ Bond-Fläche
├─ neu freigelegte Bruchfläche
├─ Temperatur
└─ Qualitäts-/Vertrauenswert
```

### Verarbeitung

Solver-Callbacks dürfen keine komplexen Szenenänderungen unmittelbar durchführen. Der sichere Ablauf ist:

1. Solver erzeugt kompakte native Events.
2. Events werden in einem Ringbuffer gesammelt.
3. Nach Abschluss des Simulationsschritts werden sie sortiert und zusammengeführt.
4. Doppelte oder unbedeutende Events werden gefiltert.
5. Das Materialsystem berechnet Schaden und Emission.
6. Änderungen werden für den nächsten Schritt geplant.

Der Event Stream muss unabhängig davon funktionieren, ob die Daten von Jolt oder PhysX kommen.

---

## 13. KA Particles: Sekundäre Splitter und granulare Partikel

### 13.1 Sekundäre Rigid-Body-Splitter

Sichtbare größere Splitter sollen als echte kleine Rigid Bodies behandelt werden. Sie können:

- vorab als Child-Fragmente vorbereitet,
- bei Bond-Bruch aktiviert,
- aus einer hierarchischen Fracture-Stufe erzeugt,
- abhängig von Energie und Material ausgewählt werden.

### 13.2 Granulare Partikel

PhysX-PBD-Partikel sind für folgende Stoffe vorgesehen:

- Sand,
- Kies,
- grobes Pulver,
- Steinmehl,
- herunterfallende Ablagerungen,
- kleine, nicht einzeln gerenderte Splitter.

PBD-Partikel sollen nicht als Ersatz für alle Rigid Bodies dienen. Große sichtbare Splitter benötigen weiterhin Rotation, Form und präzise Kollision.

### 13.3 Größenverteilung

Die Partikelgröße sollte materialabhängig aus einer Verteilung erzeugt werden, nicht als einheitlicher Radius. Relevante Parameter:

- mittlere Korngröße,
- minimale und maximale Größe,
- Verteilungsform,
- Fragmentierungsenergie,
- Bruchfläche,
- Oberflächenzustand,
- Feuchtigkeits-/Bindungsparameter,
- Render-LOD.

---

## 14. Physikalisch begründete Emission

Eine reine Änderung der Geschwindigkeit darf nicht automatisch Staub erzeugen. Ein elastisch springender Metallkörper kann eine hohe Beschleunigung haben, aber kaum Staub freisetzen.

Drei Emissionsmodelle werden getrennt behandelt.

### 14.1 Bruchstaub

Entsteht beim Brechen von Bonds und Freilegen neuer Oberfläche:

```text
Staubmasse_break =
neu freigelegte Fläche
× materialspezifischer Staubertrag
× Funktion der Bruchenergie
× Feingutanteil
```

Relevante Werte:

- Bond-Fläche,
- Bruchenergie,
- Material,
- Bruchgeschwindigkeit,
- Rauheit,
- Hierarchiestufe.

### 14.2 Aufprallstaub

Die verfügbare Aufprallenergie kann näherungsweise aus effektiver Masse und relativer Normalgeschwindigkeit berechnet werden:

```text
E_impact = 0.5 × m_eff × v_normal²
```

Nur der dissipierte Anteil oberhalb einer Materialschwelle erzeugt Staub oder Splitter.

### 14.3 Reibungs- und Schleifstaub

Für längere tangentiale Kontakte:

```text
E_scrape ≈ Tangentialkraft × Gleitstrecke
```

Zusätzliche Faktoren:

- Gleitgeschwindigkeit,
- Kontaktdauer,
- Härteverhältnis,
- Abriebkoeffizient,
- Kontaktdruck,
- Oberflächenrauheit.

Dadurch kann ein Betonfragment beim Aufschlag eine kurze Wolke und beim anschließenden Rutschen eine schwächere Staubspur erzeugen.

### 14.4 Budgetierung

Die physikalisch berechnete Masse wird nicht zwingend als identische Zahl realer Simulationspartikel dargestellt. Stattdessen wird zwischen physikalischer Masse und Darstellung getrennt:

```text
physikalische Staubmasse
        ↓
Simulationsbudget / LOD
        ↓
repräsentative Partikel + Volumendichte
```

---

## 15. KA Volumes: Feiner Staub als Volumen

Feiner luftgetragener Staub verhält sich eher wie Rauch als wie eine Menge klar getrennter Kugeln. Er benötigt:

- Dichtefeld,
- Geschwindigkeitsfeld,
- Luftwiderstand,
- Turbulenz,
- Sinkgeschwindigkeit,
- Diffusion,
- Temperatur,
- Wechselwirkung mit Hindernissen.

### Hybridmodell

```text
repräsentative Staubpartikel
Größe + Masse + Geschwindigkeit + Sinkrate
        ↓ übertragen Dichte und Impuls
sparse Volumenfeld
Dichte + Geschwindigkeit + Temperatur + Turbulenz
```

Die Partikel können grobe ballistische Bewegung abbilden. Das Volumen erzeugt die zusammenhängende Wolke.

---

## 16. Rauch und Feuer

### 16.1 NVIDIA Flow

NVIDIA Flow ist als geplanter Sparse-Voxel-Solver für folgende Effekte vorgesehen:

- Staubwolken,
- Rauch,
- heiße Gase,
- Feuer,
- Brennstoff- und Verbrennungsfelder.

Sparse bedeutet, dass nur aktive Bereiche des Volumens Speicher und Rechenzeit benötigen.

### 16.2 Gemeinsame Volumenkanäle

```text
Staub
- density
- velocity
- settling parameters

Rauch
- smoke density
- temperature
- velocity

Feuer
- fuel
- temperature
- burn
- soot/smoke yield
- velocity
```

### 16.3 Feuer als spätere Stufe

Feuer benötigt zusätzlich:

- Brennstoffmenge,
- Zündtemperatur,
- Verbrennungsrate,
- Wärmefreisetzung,
- Sauerstoffmodell oder vereinfachte Annahme,
- Rauch-/Rußbildung,
- Materialklassifizierung,
- Wärmeübertragung auf Bodies und Partikel.

Die Architektur soll Feuer von Beginn an berücksichtigen, die Implementierung beginnt jedoch erst nach stabiler Staub- und Rauchsimulation.

---

## 17. Gemeinsame Cache-Architektur

Jede Simulationsdomäne benötigt einen eigenen versionierten Cache.

```text
Simulation Cache
├─ scene_manifest.json
├─ rigid_transforms.karc
├─ rigid_velocities.karv        optional
├─ contacts.kaev                optional
├─ destruction_events.kaev
├─ bond_states.kabd
├─ particle_frames.kapc
├─ particle_events.kaev
├─ volumes/
│  ├─ dust_####.vdb
│  ├─ smoke_####.vdb
│  └─ fire_####.vdb
└─ diagnostics.json
```

### Anforderungen

- stabile Fragment-, Bond- und Body-IDs,
- Schema- und Add-on-Version,
- Solver- und Bibliotheksversion,
- Geometriesignatur,
- Materialsignatur,
- Szeneneinstellungen,
- getrennte Invalidierung einzelner Cache-Stufen,
- atomisches Schreiben,
- Cold-/Warm-Cache-Messung,
- klare Erkennung veralteter Daten.

Ein geänderter Shader soll keinen Rigid-Body-Rebake auslösen. Eine geänderte Bruchgeometrie muss dagegen alle davon abhängigen Stufen invalidieren.

---

## 18. Geplante Paket- und Repository-Struktur

Die Funktionsbereiche bleiben in getrennten Add-ons beziehungsweise Repositories:

```text
ka_simulation_core/
├─ ids.py
├─ units.py
├─ schemas/
│  ├─ destruction_asset.py
│  ├─ simulation_scene.py
│  ├─ material_profile.py
│  └─ simulation_event.py
├─ cache_manifest.py
├─ compatibility.py
└─ registry.py

ka_fracture/
├─ fracture/
├─ surfaces/
├─ adjacency/
├─ collision_proxy/
├─ asset_export/
└─ blender/

ka_rigid_dynamics/
├─ scene_adapter/
├─ collision/
├─ destruction/
├─ backends/
│  ├─ jolt/
│  ├─ physx/
│  └─ blast/
├─ events/
├─ cache/
└─ blender/

ka_particles/
├─ emission/
├─ rigid_debris/
├─ granular/
├─ backends/
├─ cache/
└─ blender/

ka_volumes/
├─ dust/
├─ smoke/
├─ combustion/
├─ flow_bridge/
├─ vdb_cache/
└─ blender/

ka_destruction_suite/
├─ module_registry/
├─ workflow/
├─ dependency_graph/
├─ presets/
├─ diagnostics/
└─ blender/
```

### Auslieferungsformen

Es sollen zwei Formen möglich sein:

1. **Einzelmodule:** jedes Add-on separat installierbar und nutzbar.
2. **KA Destruction Suite:** gemeinsame Auslieferung beziehungsweise Koordination aller kompatiblen Module.

Auch bei einer gemeinsamen Suite-Auslieferung bleiben interne Python-Pakete, native Bibliotheken, Cache-Schemata und Tests modular getrennt. Das verhindert, dass eine Änderung am Volume-Solver den Rigid-Body-Kern oder das Fracture-Add-on unnötig destabilisiert.

---

## 19. Versions- und Kompatibilitätsmodell

Jedes Modul wird unabhängig versioniert. Beispiel:

```text
KA Simulation Core       1.0.0
KA Fracture              1.3.0
KA Rigid Dynamics        0.8.0
KA Particles             0.4.0
KA Volumes               0.2.0
KA Destruction Suite     0.1.0
```

Die Suite und jedes datenlesende Modul prüfen nicht nur Add-on-Versionen, sondern auch Schema-Versionen:

```text
DestructionAsset schema  1
SimulationScene schema   1
SimulationEvent schema   1
MaterialProfile schema   1
CacheManifest schema     1
```

Ein Modul darf neuere unbekannte Pflichtfelder nicht stillschweigend ignorieren. Für ältere Daten werden kontrollierte Migrationen oder klare Fehlermeldungen benötigt.

Beispiel einer Suite-Anforderung:

```text
KA Simulation Core >= 1.0
KA Fracture >= 1.3
KA Rigid Dynamics >= 0.8
KA Particles >= 0.4       optional
KA Volumes >= 0.2         optional
```

Fehlende optionale Module deaktivieren nur die zugehörige Workflow-Stufe. Das gesamte System darf dadurch nicht unbrauchbar werden.

---

## 20. Entwicklungsphasen

### Phase 0 – Gemeinsamen Vertrag definieren

- KA Simulation Core als kleines eigenständiges Paket anlegen.
- SI-Einheiten, Achsen, Skalierung und ID-Regeln festlegen.
- `DestructionAsset`, `SimulationScene`, `MaterialProfile`, `SimulationEvent` und `CacheManifest` versionieren.
- Modulregistrierung und Capability-Abfrage definieren.
- diese gemeinsame README als Master-Dokument etablieren.

**Abnahmekriterium:** Zwei unabhängige Testmodule können ein Asset beziehungsweise Event ohne direkte gegenseitige Imports schreiben und lesen.

### Phase 1 – KA Fracture als Datenproduzent

- stabile Fragment-IDs,
- gemeinsame Bruchflächen und Nachbarschaften,
- glatte Ausgangsmeshes und Collision-Proxys,
- Fragment- und Bond-Graph,
- exportierbares Destruction Asset,
- Validierung und Vorschau.

**Abnahmekriterium:** Ein Fracture-Ergebnis kann gespeichert, neu geladen und von einem neutralen Testleser eindeutig rekonstruiert werden.

### Phase 2 – KA Rigid Dynamics auf neutralen Core umstellen

- vorhandenen Jolt-Pfad auf `SimulationScene` umstellen,
- Cache und Events an stabile IDs binden,
- bestehende Ergebnisse vor und nach der Umstellung vergleichen,
- Compound Convex weiter stabilisieren,
- anschließend PhysX-Rigid-Body-Prototyp entwickeln.

**Abnahmekriterium:** Die gleiche Testszene kann ohne Änderung des Blender-Assets mit Jolt und später PhysX gebacken werden.

### Phase 3 – Zusammenhalt und dynamische Zerstörung

- Bonds und Support Graph,
- Material- und Schadensparameter,
- Blast evaluieren und integrieren,
- Schadensakkumulation,
- Inseltrennung,
- korrekte Masse, Trägheit und Geschwindigkeit neuer Actors.

**Abnahmekriterium:** Ein Körper zerfällt nur an physikalisch beziehungsweise parametrisch ausgelösten Verbindungen.

### Phase 4 – Event Stream

- Kontakt-, Impact-, Slide-, Scrape- und Separation-Events,
- Bond-Damage- und Bond-Break-Events,
- Energie- und Flächenberechnung,
- native Ringbuffer,
- Filterung, Aggregation und Event-Cache.

**Abnahmekriterium:** Events können von einem unabhängigen Test-Consumer verarbeitet werden, ohne den Solver zu kennen.

### Phase 5 – KA Particles

- manuelle und eventbasierte Emitter,
- hierarchische Rigid-Body-Splitter,
- PhysX-PBD oder alternativer granularer Solver,
- materialabhängige Größenverteilungen,
- Sleeping, Lebensdauer, LOD und Partikelcache.

### Phase 6 – KA Volumes

- Staubemission aus Event Stream,
- Dichte-, Impuls- und Temperaturübergabe,
- Sparse-Volume-Solver,
- Hindernisfelder aus Rigid Bodies,
- OpenVDB-/NanoVDB-Ausgabe,
- danach Rauch und Feuer.

### Phase 7 – KA Destruction Suite

- Modul- und Versionsprüfung,
- gemeinsame UI,
- Bake-Abhängigkeitsgraph,
- Presets und Materialbibliothek,
- einheitliches Cache-Management,
- Fortschritt, Fehlerprüfung und Diagnose,
- gemeinsame Auslieferung ohne Verlust der Modulgrenzen.

---

## 21. Test- und Benchmarkplan

### 21.1 Rigid Bodies

- 100, 500, 1.000 und 5.000 Bodies.
- Cold und Warm Collider Cache.
- Jolt gegen PhysX CPU gegen PhysX GPU.
- Single Hull gegen Compound Convex.
- kleine, mittlere und extreme Massenverhältnisse.
- Stapel, Schüttung, Fall, Kollision, dünne Hindernisse und CCD.
- Sleeping und frühes Bake-Ende.

### 21.2 Bonds und Zerstörung

- einzelner Zug-, Druck-, Scher- und Torsionstest.
- wiederholte Belastung und Schadensakkumulation.
- Vergleich kleiner und großer Bond-Flächen.
- symmetrische und asymmetrische Inseltrennung.
- Impuls- und Energieerhaltung beim Splitting.
- Beton-, Glas-, Holz- und Metallprofile.

### 21.3 Partikel

- Emissionsmasse gegen berechnete Bruch-/Dissipationsenergie.
- Korngrößenverteilung.
- Kontakt und Ablagerung.
- Sleeping und Partikelverdichtung.
- 10.000, 100.000 und 1.000.000 repräsentative Partikel, soweit Backend und GPU dies erlauben.

### 21.4 Staub und Volumen

- Dichteerhaltung.
- Impulsübertragung.
- Sinkgeschwindigkeit.
- Hindernisinteraktion.
- Sparse-Domänenwachstum.
- VDB-Dateigröße und Schreibzeit.
- visuelle Übereinstimmung bei identischen Events.

### 21.5 Reproduzierbarkeit

- identischer Doppel-Bake.
- Neustart mit Warm Cache.
- Windows/Linux-Vergleich.
- CPU/GPU-Abweichung dokumentieren.
- Solverversionen im Cache speichern.

---

## 22. UI-Grundsätze

### Einzel-Add-ons

Jedes Einzel-Add-on zeigt nur seine eigenen produktionsrelevanten Funktionen und bleibt ohne Suite bedienbar.

**KA Fracture** zeigt Fragmentierung, Oberflächen, Proxy-Erzeugung und Asset-Export.  
**KA Rigid Dynamics** zeigt Solver, Collider, Bodies, Zusammenhalt, Bake und Rigid-Cache.  
**KA Particles** zeigt Emitter, Partikelmaterial, Solver, LOD und Partikelcache.  
**KA Volumes** zeigt Quellen, Volumenkanäle, Qualität, Domain und VDB-Cache.

### KA Destruction Suite

Die Suite zeigt eine reduzierte, durchgängige Prozessoberfläche:

- Fracture,
- Destruction Asset,
- Rigid Dynamics,
- Particles,
- Volumes,
- Gesamt-Bake,
- Cache und Playback,
- Diagnose.

Die Suite darf Einstellungen nicht duplizieren. Sie ruft die öffentlichen Operatoren und Datenverträge der Einzelmodule auf.

### Allgemeine UI-Regeln

- wenige Presets in der normalen Oberfläche,
- technische Einzelwerte in aufklappbaren Advanced-Bereichen,
- nicht installierte oder nicht implementierte Fähigkeiten klar kennzeichnen,
- keine scheinbar funktionsfähigen Backend-Optionen ohne vollständigen Pfad,
- Abhängigkeiten und nötige Rebakes sichtbar machen,
- Modulgrenzen für Benutzer möglichst verständlich, aber nicht störend darstellen.

---

## 23. Entwicklungsregeln

1. Render-Geometrie und Collision-Geometrie bleiben getrennt.
2. Alle Rigid-Body-Backends erhalten dieselbe neutrale Szenenbeschreibung.
3. Solverabhängige Daten dürfen nicht unkontrolliert in UI oder Cacheformat durchsickern.
4. Material- und Emissionsmodelle bleiben solverunabhängig.
5. Neue Effekte werden aus dem Event Stream gespeist, nicht aus direkter UI-Verkabelung.
6. Jede teure Vorverarbeitung wird persistent gecacht.
7. Jede Cache-Stufe kann unabhängig invalidiert werden.
8. Standardwerte müssen ohne Diagnosemodus produktiv sein.
9. Diagnosen verändern keine Simulationsergebnisse.
10. Neue Backends werden zuerst mit kleinen isolierten Tests und danach mit realen Fracture-Szenen geprüft.
11. GPU-Unterstützung wird gemessen, nicht pauschal als schneller angenommen.
12. Forschungskomponenten werden erst nach belastbarer Validierung zu Produktionsabhängigkeiten.
13. Einzelmodule bleiben unabhängig installierbar und testbar.
14. Funktionsmodule kommunizieren über Core-Verträge statt über private Klassen.
15. Das Suite-Add-on koordiniert, dupliziert aber keine Simulationslogik.
16. Die gemeinsame README wird synchron gehalten; modulbezogene Details bleiben in modulspezifischen Dateien.
17. Ein Modulfehler darf nach Möglichkeit nicht die Registrierung aller anderen Module verhindern.

---

## 24. Risiken und offene Entscheidungen

### Native Bibliotheken

PhysX, Blast, Flow und gegebenenfalls Warp benötigen kontrollierte native Builds für unterstützte Blender-/Python-/Betriebssystemversionen. Abhängigkeiten, CUDA-Versionen und Lizenzen müssen reproduzierbar dokumentiert werden.

### NVIDIA-Abhängigkeit

Ein GPU-Hauptpfad darf den CPU-Fallback nicht vollständig verdrängen. Jolt bleibt für Systeme ohne passende NVIDIA-GPU und für kleine Szenen relevant.

### Physikalische Genauigkeit

Rigid-Body- und Bond-Modelle approximieren echte Rissausbreitung. Phase-Field-, FEM-, MPM- oder Finite-Discrete-Element-Verfahren können genauer sein, sind aber wesentlich teurer. Sie sind eine mögliche spätere wissenschaftliche Qualitätsstufe, nicht die erste Produktionsbasis.

### Zwei-Wege-Kopplung

Vollständige Rückkopplung zwischen Millionen Partikeln, Volumenfeldern und Rigid Bodies ist teuer. Die erste Produktionsversion soll überwiegend gerichtete Kopplung verwenden:

```text
Rigid/Destruction → Events → Partikel/Volumen
```

Gezielte Rückwirkung, etwa Partikeldruck oder Wärme, wird nur dort ergänzt, wo sie visuell oder physikalisch relevant ist.

### Einheitensystem

Alle internen Daten müssen eindeutig in SI-Einheiten definiert werden:

- Meter,
- Kilogramm,
- Sekunden,
- Newton,
- Joule,
- Pascal,
- Kelvin.

Blender-Skalierung und Objekttransformationen werden vor Solverübergabe normalisiert.

### Versionsdrift zwischen Add-ons

Getrennte Entwicklung kann zu inkompatiblen Datenständen führen. Deshalb sind Schema-Versionen, Capability-Abfragen, Mindestversionen und Migrationstests zwingend.

### Doppelte Benutzeroberflächen

Einzelmodule und Suite dürfen dieselben Parameter nicht in voneinander unabhängigen Property-Sätzen speichern. Die Suite soll vorhandene Modul-Properties bedienen oder auf gemeinsame Core-Properties verweisen.

### Gemeinsame README

Eine identische Master-README kann veralten, wenn sie nicht in alle Repositories übernommen wird. Der Dokumentationsstand sollte deshalb bei Releases geprüft und idealerweise durch ein Synchronisationsskript aktualisiert werden.

---

## 25. Externe Grundlagen und Referenzen

Die folgenden Projekte und Dokumentationen bilden die derzeit geplante technische Grundlage. Sie sind keine Garantie, dass jede Komponente unverändert integriert wird.

### Produktionsbibliotheken

- NVIDIA PhysX, Blast und Flow: <https://github.com/NVIDIA-Omniverse/PhysX>
- PhysX GPU Rigid Bodies: <https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/GPURigidBodies.html>
- PhysX Particle System: <https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/ParticleSystem.html>
- PhysX Simulation Events: <https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Simulation.html>
- NVIDIA Blast SDK: <https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html>
- NVIDIA Flow: <https://nvidia-omniverse.github.io/PhysX/flow/index.html>
- Jolt Physics: <https://jrouwe.github.io/JoltPhysicsDocs/>
- CoACD: <https://github.com/SarahWeiii/CoACD>
- NVIDIA Warp: <https://nvidia.github.io/warp/stable/>
- Newton: <https://newton-physics.github.io/newton/stable/guide/overview.html>
- Blender OpenVDB Volumes: <https://docs.blender.org/manual/en/latest/modeling/volumes/index.html>

### Beobachtete Forschung

- Convex Primitive Decomposition: <https://onlinelibrary.wiley.com/doi/10.1111/cgf.70411>
- VisACD: <https://3dlg-hcvc.github.io/visacd/>
- Learning Convex Decomposition via Feature Fields: <https://arxiv.org/abs/2603.09285>
- Cohesive Voronoi/DEM fracture model: <https://www.sciencedirect.com/science/article/pii/S0020768319300629>
- Multi-scale debris and dust generation: <https://link.springer.com/article/10.1007/s00371-009-0319-3>

---

## 26. Unmittelbar nächste Entwicklungsschritte

Die nächste Arbeit soll die Modultrennung technisch absichern, bevor weitere große Solver integriert werden:

1. Gemeinsame Schemata für `DestructionAsset`, `SimulationScene`, `MaterialProfile`, `SimulationEvent` und `CacheManifest` entwerfen.
2. Entscheiden, ob KA Simulation Core als eigenes Blender-Add-on, eingebettetes versioniertes Python-Paket oder beides ausgeliefert wird.
3. Stabile IDs und SI-Einheiten in KA Fracture und KA Rigid Dynamics vereinheitlichen.
4. KA Fracture um Fragment-Graph, gemeinsame Bruchflächen und Asset-Export ergänzen.
5. KA Rigid Dynamics so umbauen, dass es dieses neutrale Asset lesen kann, ohne KA Fracture importieren zu müssen.
6. Den vorhandenen Jolt-Pfad auf die neutrale `SimulationScene` umstellen und Regressionen vergleichen.
7. Erst danach den PhysX-/Blast-Prototyp beginnen.
8. KA Particles und KA Volumes zunächst mit aufgezeichneten Test-Events entwickeln, damit sie nicht von einem unfertigen PhysX-Pfad blockiert werden.
9. KA Destruction Suite erst aufbauen, sobald mindestens Fracture und Rigid Dynamics stabile öffentliche Schnittstellen besitzen.

Diese Reihenfolge ermöglicht parallele und unabhängige Entwicklung, ohne die spätere Integration dem Zufall zu überlassen.

---

## 27. Verwendung dieser README in den Einzel-Add-ons

Diese Datei wird unverändert oder automatisiert synchronisiert in folgende Pakete aufgenommen:

```text
KA-Simulation-Core/README.md
KA-Fracture/README.md
KA-Rigid-Dynamics/README.md
KA-Particles/README.md
KA-Volumes/README.md
KA-Destruction-Suite/README.md
```

Jedes Add-on ergänzt zusätzlich:

- `ARCHITECTURE.md` – aktueller interner Aufbau des jeweiligen Moduls,
- `CHANGELOG.md` – konkrete Versionsänderungen,
- optional `USER_GUIDE.md` – Bedienung des jeweiligen Add-ons,
- optional `DEVELOPMENT.md` – Build-, Test- und Abhängigkeitsanweisungen.

Beim Kopieren dieser README darf nicht der Eindruck entstehen, alle beschriebenen Funktionen seien bereits im jeweiligen Modul vorhanden. Maßgeblich ist immer die Statusübersicht sowie die modulspezifische Dokumentation.

### Aktuelles Paket: KA Rigid Dynamics 0.7.6

Für die derzeitige Rigid-Dynamics-Version gilt weiterhin:

1. Add-on-ZIP in Blender über `Edit > Preferences > Get Extensions > Install from Disk` installieren.
2. Nach einem fehlgeschlagenen Registrierungsversuch Blender vollständig neu starten.
3. Frühere Versionen mit derselben Add-on-ID vorher entfernen, falls Blender die Installation nicht ersetzt.
4. Nach Änderungen an Collider-Schema oder nativen Bibliotheken Collider- und Simulationscache löschen.
5. Für normale Bakes Detaildiagnosen deaktiviert lassen.

Aktuelle Rigid-Body-Basis: **Jolt + CoACD + binärer Cache**.  
Geplante Gesamtbasis: **modulare KA-Add-ons + gemeinsamer Core + PhysX/Blast + Jolt-Fallback + PBD-Partikel + Flow/OpenVDB**.

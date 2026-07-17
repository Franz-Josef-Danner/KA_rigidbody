# KA Rigid Dynamics 0.5.1

Native Rigid-Body-Simulation und Entwicklungsgrundlage für ein zukünftiges, gekoppeltes Zerstörungs-, Partikel-, Staub-, Rauch- und Feuersystem in Blender.

> **Wichtige Abgrenzung:** Version 0.5.1 enthält aktuell Jolt/Culverin für Rigid Bodies, CoACD für Compound-Convex-Collider und einen binären Transform-Cache. PhysX, Blast, PBD-Partikel, NVIDIA Flow, Rauch und Feuer sind in dieser Version noch nicht implementiert. Die folgenden Kapitel dokumentieren die geplante Zielarchitektur und die Entwicklungsreihenfolge.

Die bisherige Versionshistorie wurde in [`CHANGELOG.md`](CHANGELOG.md) ausgelagert. Die technische Ist-Architektur steht in [`ARCHITECTURE.md`](ARCHITECTURE.md).

---

## 1. Projektziel

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
KA Fracture / Geometry Authoring
        ↓
Destruction Asset
Fragmente + Collider + Nachbarschaften + Bonds + Materialien
        ↓
Simulation Core
PhysX/Blast als geplanter Hauptpfad, Jolt als CPU-Fallback
        ↓
Solverunabhängiger Event Stream
Bruch + Aufprall + Trennung + Reibung + Energie
        ↓
Sekundärsimulationen
Rigid Debris + granulare PBD-Partikel + Staubemission
        ↓
Sparse Volume Simulation
Staub + Rauch + Feuer
        ↓
Blender Cache und Rendering
Transforme + Partikel + Events + OpenVDB/NanoVDB
```

---

## 2. Aktueller Stand von Version 0.5.1

### Bereits vorhanden

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

### Noch nicht vorhanden

- Vollständiger PhysX-Backendpfad.
- Native PhysX-GPU-Rigid-Bodies.
- NVIDIA Blast und Bond-/Support-Graph-Auswertung.
- Dynamische Triangle-Mesh-zu-Triangle-Mesh-Kollision.
- PhysX-PBD-Partikelsystem.
- Eigenes Material- und Schadensmodell.
- Solverunabhängiger Event Stream.
- Volumetrischer Staub.
- NVIDIA Flow.
- Rauch- und Feuerberechnung.
- OpenVDB-/NanoVDB-Ausgabe.

---

## 3. Grundprinzip: Mehrere gekoppelte Skalen statt eines einzigen Solvers

Ein einzelner Solver sollte nicht gleichzeitig große Fragmente, feine Körner und luftgetragenen Staub als identische Objekte behandeln. Das wäre entweder zu ungenau oder unnötig teuer.

Das System wird deshalb in vier Größenklassen getrennt:

| Größenklasse | Darstellung | Geplanter Solver |
|---|---|---|
| Große Bruchstücke | Rigid Bodies | PhysX oder Jolt |
| Kleine sichtbare Splitter | kleine Rigid Bodies | PhysX oder Jolt |
| Sand, Kies, grobes Pulver | granulare Partikel | PhysX PBD |
| Feiner Staub, Rauch, Feuer | sparse Volumenfelder | NVIDIA Flow oder eigener Sparse-Volume-Solver |

Die Kopplung erfolgt über Ereignisse, Impulse, Energie und Materialparameter. Nicht jedes Staubkorn muss einzeln mit jedem Rigid Body kollidieren.

---

## 4. Geometry Authoring und Fracturing

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

## 5. Collision-Proxys

### 5.1 Single Convex Hull

Der schnellste und stabilste Standard. Geeignet für nahezu konvexe Fragmente und Massensimulationen.

Nachteile:

- überbrückt Einbuchtungen,
- kann sichtbare Abstände erzeugen,
- kann Kontaktflächen vergrößern,
- kann das Stapelverhalten verändern.

### 5.2 Compound Convex

Mehrere konvexe Teilkörper bilden eine konkave Form näherungsweise nach. Dies ist die bevorzugte präzise Kollisionsform für dynamische Bruchstücke.

Version 0.5.1 verwendet CoACD zur Zerlegung. Aufgrund der aktuellen Culverin-Grenzen wird ein logischer Compound-Körper intern noch als mehrere Jolt-Bodies mit festen Verbindungen dargestellt. Langfristig soll eine native Jolt-`StaticCompoundShape` beziehungsweise ein echter PhysX-Compound-Actor verwendet werden.

Empfohlene Qualitätsstufen:

| Preset | Maximale Teile | Verwendung |
|---|---:|---|
| Fast | 4 | leichte Konkavität |
| Balanced | 8 | Produktionsstandard |
| Accurate | 16 | sichtbare Nahaufnahme |
| Custom | benutzerdefiniert | gezielte Problemlösung |

### 5.3 Triangle Mesh

- Für statische Umgebung sehr genau und sinnvoll.
- Für dynamische Körper solverabhängig und problematisch.
- Dynamisches Mesh-gegen-Mesh ist in Jolt nicht als allgemeiner Produktionsweg geeignet.
- Ein späterer PhysX-Pfad kann zusätzliche Optionen wie GPU-SDF-Collider bereitstellen.

### 5.4 Geplante automatische Auswahl

Ein späterer `Auto`-Modus soll pro Körper entscheiden:

```text
nahezu konvex              → Single Convex Hull
deutlich konkav            → Compound Convex
statische komplexe Umgebung → Triangle Mesh
gezielter PhysX-GPU-Fall    → SDF
```

Die automatische Entscheidung soll auf gemessener Formabweichung, Konkavität, Objektgröße, Sichtbarkeit und erwarteter Kontaktrelevanz beruhen.

---

## 6. Destruction Asset und Bond Graph

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

## 7. Material- und Schadensmodell

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

## 8. Solverstrategie

### 8.1 Jolt

Jolt bleibt sinnvoll als:

- schneller CPU-Rigid-Body-Solver,
- Fallback ohne NVIDIA-GPU,
- leichter Modus für kleine und mittlere Szenen,
- Referenzsolver für Regression und Vergleiche,
- produktiver Rigid-Body-only-Pfad.

### 8.2 PhysX

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

### 8.3 Blast

Blast soll nicht KA Fracture ersetzen. Geplant ist:

- KA Fracture erzeugt Geometrie, Fragment-IDs, Nachbarschaften und Bond-Daten.
- Blast verwaltet Support Graph, Schaden, Bond-Bruch und Actor-Splitting.
- PhysX simuliert die daraus entstehenden Actors.

### 8.4 Warp und Newton

NVIDIA Warp ist als Werkzeug für eigene GPU-Kernels interessant, etwa für:

- Erosion,
- Partikelklassifikation,
- Materialabhängige Emission,
- Raster-/Partikelkopplung,
- benutzerdefinierte Feldoperationen,
- NanoVDB-Verarbeitung.

Newton ist derzeit eher eine Forschungs- und Robotikplattform mit mehreren Solvertypen. Es sollte beobachtet, aber nicht als primäre Produktionsgrundlage angenommen werden.

---

## 9. Solverunabhängiger Event Stream

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

## 10. Sekundäre Splitter und granulare Partikel

### 10.1 Sekundäre Rigid-Body-Splitter

Sichtbare größere Splitter sollen als echte kleine Rigid Bodies behandelt werden. Sie können:

- vorab als Child-Fragmente vorbereitet,
- bei Bond-Bruch aktiviert,
- aus einer hierarchischen Fracture-Stufe erzeugt,
- abhängig von Energie und Material ausgewählt werden.

### 10.2 Granulare Partikel

PhysX-PBD-Partikel sind für folgende Stoffe vorgesehen:

- Sand,
- Kies,
- grobes Pulver,
- Steinmehl,
- herunterfallende Ablagerungen,
- kleine, nicht einzeln gerenderte Splitter.

PBD-Partikel sollen nicht als Ersatz für alle Rigid Bodies dienen. Große sichtbare Splitter benötigen weiterhin Rotation, Form und präzise Kollision.

### 10.3 Größenverteilung

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

## 11. Physikalisch begründete Emission

Eine reine Änderung der Geschwindigkeit darf nicht automatisch Staub erzeugen. Ein elastisch springender Metallkörper kann eine hohe Beschleunigung haben, aber kaum Staub freisetzen.

Drei Emissionsmodelle werden getrennt behandelt.

### 11.1 Bruchstaub

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

### 11.2 Aufprallstaub

Die verfügbare Aufprallenergie kann näherungsweise aus effektiver Masse und relativer Normalgeschwindigkeit berechnet werden:

```text
E_impact = 0.5 × m_eff × v_normal²
```

Nur der dissipierte Anteil oberhalb einer Materialschwelle erzeugt Staub oder Splitter.

### 11.3 Reibungs- und Schleifstaub

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

### 11.4 Budgetierung

Die physikalisch berechnete Masse wird nicht zwingend als identische Zahl realer Simulationspartikel dargestellt. Stattdessen wird zwischen physikalischer Masse und Darstellung getrennt:

```text
physikalische Staubmasse
        ↓
Simulationsbudget / LOD
        ↓
repräsentative Partikel + Volumendichte
```

---

## 12. Feiner Staub als Volumen

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

## 13. Rauch und Feuer

### 13.1 NVIDIA Flow

NVIDIA Flow ist als geplanter Sparse-Voxel-Solver für folgende Effekte vorgesehen:

- Staubwolken,
- Rauch,
- heiße Gase,
- Feuer,
- Brennstoff- und Verbrennungsfelder.

Sparse bedeutet, dass nur aktive Bereiche des Volumens Speicher und Rechenzeit benötigen.

### 13.2 Gemeinsame Volumenkanäle

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

### 13.3 Feuer als spätere Stufe

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

## 14. Cache-Architektur

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

## 15. Geplante Modulstruktur

```text
ka_rigid_dynamics/
├─ geometry/
│  ├─ fracture_asset.py
│  ├─ adjacency.py
│  ├─ collision_proxy.py
│  └─ mass_properties.py
├─ destruction/
│  ├─ fragment_graph.py
│  ├─ bond_graph.py
│  ├─ materials.py
│  ├─ damage.py
│  └─ island_split.py
├─ simulation/
│  ├─ scene_description.py
│  ├─ events.py
│  ├─ jolt_backend.py
│  ├─ physx_backend.py
│  └─ blast_bridge.py
├─ particles/
│  ├─ emission.py
│  ├─ rigid_debris.py
│  ├─ pbd_backend.py
│  └─ particle_cache.py
├─ volumes/
│  ├─ dust.py
│  ├─ flow_bridge.py
│  ├─ vdb_cache.py
│  └─ combustion.py
├─ cache/
│  ├─ manifest.py
│  ├─ rigid_cache.py
│  ├─ event_cache.py
│  └─ invalidation.py
└─ blender/
   ├─ properties.py
   ├─ operators.py
   ├─ ui.py
   └─ playback.py
```

Die tatsächliche Umstrukturierung soll schrittweise erfolgen, damit der vorhandene Jolt-Pfad nach jedem Schritt funktionsfähig bleibt.

---

## 16. Entwicklungsphasen

### Phase 1 – Solverunabhängiges Fundament

- `SimulationScene` und neutrale Body-/Shape-Datentypen definieren.
- stabile Fragment-, Bond-, Material- und Event-IDs einführen.
- Fragment Graph und Bond Graph implementieren.
- gemeinsame Bruchflächen aus KA Fracture übernehmen.
- Materialprofile und Schadensparameter definieren.
- Event-Schema und Cache-Manifest festlegen.
- vorhandenen Jolt-Pfad auf die neutrale Szene umstellen.

**Abnahmekriterium:** Der bestehende Jolt-Bake liefert vor und nach der Umstellung identische beziehungsweise toleranzgleichwertige Ergebnisse.

### Phase 2 – PhysX-Rigid-Body-Prototyp

- kontrollierte native C++-Bridge für Windows und Linux.
- CPU- und optionaler CUDA-GPU-Modus.
- statische Triangle Meshes.
- dynamische Convex Hulls.
- native Compound Convex Actors.
- Masse, Trägheit, Reibung, Restitution und Dämpfung.
- Transform- und Kontaktdaten.
- identischer Cachevertrag wie Jolt.

**Abnahmekriterium:** Gleiche Testszene kann ohne Blender-Datenumbau mit Jolt oder PhysX gebacken werden.

### Phase 3 – Blast und Zusammenhalt

- Blast-Asset aus Fragmenten und Bonds erzeugen.
- Support Graph definieren.
- Schadensakkumulation.
- Stress-Solver evaluieren.
- Actor-Splitting bei Graphtrennung.
- korrekte Masse, Trägheit und Geschwindigkeit neuer Inseln.

**Abnahmekriterium:** Ein zunächst zusammenhängender Körper zerfällt nur dort, wo Material- und Belastungsparameter dies auslösen.

### Phase 4 – Event Stream

- Kontakt-, Impact-, Slide- und Separation-Events.
- Bond-Damage- und Bond-Break-Events.
- Energie- und Flächenberechnung.
- native Ringbuffer.
- Filterung und zeitliche Zusammenfassung.
- Event-Cache und Diagnoseansicht.

**Abnahmekriterium:** Identische Ereignisdaten können unabhängig vom späteren Partikel- oder Volume-Solver ausgewertet werden.

### Phase 5 – Sekundäre Splitter und PBD-Partikel

- hierarchische kleine Rigid-Body-Splitter.
- PhysX-PBD-Partikelsystem.
- materialabhängige Größenverteilungen.
- kontakt- und bruchabhängige Emission.
- Sleeping, Lebensdauer und LOD.
- Partikelcache.

### Phase 6 – Volumetrischer Staub

- Staubemission aus Event Stream.
- Übergabe von Dichte, Impuls und Temperatur an Sparse Volume.
- Luftwiderstand und Sinkgeschwindigkeit.
- Rigid-Body-Kollision beziehungsweise Hindernisfelder.
- OpenVDB-/NanoVDB-Ausgabe.

### Phase 7 – Rauch und Feuer

- Rauchquellen.
- Temperatur- und Brennstoffkanäle.
- vereinfachte Verbrennung.
- Wärmeübertragung.
- Materialabhängige Rauch- und Rußausbeute.
- Rendering-Presets in Blender.

---

## 17. Test- und Benchmarkplan

### 17.1 Rigid Bodies

- 100, 500, 1.000 und 5.000 Bodies.
- Cold und Warm Collider Cache.
- Jolt gegen PhysX CPU gegen PhysX GPU.
- Single Hull gegen Compound Convex.
- kleine, mittlere und extreme Massenverhältnisse.
- Stapel, Schüttung, Fall, Kollision, dünne Hindernisse und CCD.
- Sleeping und frühes Bake-Ende.

### 17.2 Bonds und Zerstörung

- einzelner Zug-, Druck-, Scher- und Torsionstest.
- wiederholte Belastung und Schadensakkumulation.
- Vergleich kleiner und großer Bond-Flächen.
- symmetrische und asymmetrische Inseltrennung.
- Impuls- und Energieerhaltung beim Splitting.
- Beton-, Glas-, Holz- und Metallprofile.

### 17.3 Partikel

- Emissionsmasse gegen berechnete Bruch-/Dissipationsenergie.
- Korngrößenverteilung.
- Kontakt und Ablagerung.
- Sleeping und Partikelverdichtung.
- 10.000, 100.000 und 1.000.000 repräsentative Partikel, soweit Backend und GPU dies erlauben.

### 17.4 Staub und Volumen

- Dichteerhaltung.
- Impulsübertragung.
- Sinkgeschwindigkeit.
- Hindernisinteraktion.
- Sparse-Domänenwachstum.
- VDB-Dateigröße und Schreibzeit.
- visuelle Übereinstimmung bei identischen Events.

### 17.5 Reproduzierbarkeit

- identischer Doppel-Bake.
- Neustart mit Warm Cache.
- Windows/Linux-Vergleich.
- CPU/GPU-Abweichung dokumentieren.
- Solverversionen im Cache speichern.

---

## 18. UI-Grundsätze

Die normale Oberfläche soll wenige produktionsrelevante Entscheidungen zeigen:

- Solver: Auto / Jolt / PhysX.
- Qualitätsprofil: Fast / Balanced / Accurate.
- Materialprofil.
- Collider: Auto / Convex / Compound / Static Mesh.
- Zusammenhalt: None / Bonded / Hierarchical.
- Partikel und Staub: Off / Material Driven / Custom.
- Bake, Cache und Playback.

Technische Einzelparameter gehören in aufklappbare Bereiche:

- Advanced Rigid Body,
- Advanced Fracture,
- Advanced Bonds,
- Advanced Particles,
- Advanced Volumes,
- Diagnostics & Developer.

Nicht implementierte Solver dürfen nicht als scheinbar funktionierende Produktionsoption sichtbar sein.

---

## 19. Entwicklungsregeln

1. Render-Geometrie und Collision-Geometrie bleiben getrennt.
2. Alle externen Solver erhalten dieselbe neutrale Szenenbeschreibung.
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

---

## 20. Risiken und offene Entscheidungen

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

---

## 21. Externe Grundlagen und Referenzen

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

## 22. Unmittelbar nächster Entwicklungsschritt

Vor einer Partikel-, Rauch- oder Feuerimplementierung wird das solverunabhängige Fundament aufgebaut:

1. `SimulationScene` als neutrale Szenenbeschreibung.
2. stabile Fragment-, Bond-, Material- und Event-IDs.
3. Fragment Graph und Bond Graph.
4. gemeinsame Bruchflächen aus KA Fracture.
5. Materialprofile und Schadensparameter.
6. Event-Schema und versionierter Event-Cache.
7. Umstellung des bestehenden Jolt-Pfads auf diese Datenstruktur.
8. Danach ein kleiner PhysX-Rigid-Body-Prototyp.

Diese Reihenfolge verhindert, dass Collider, Materialien, Cache, UI und Ereignislogik für PhysX, Partikel und Flow mehrfach neu gebaut werden müssen.

---

## 23. Installation der aktuellen Version

1. Das Add-on-ZIP in Blender über `Edit > Preferences > Get Extensions > Install from Disk` installieren.
2. Blender nach einem fehlgeschlagenen Registrierungsversuch vollständig neu starten.
3. Frühere Versionen mit derselben Add-on-ID vorher entfernen, falls Blender die Installation nicht ersetzt.
4. Nach Änderungen an Collider-Schema oder nativen Bibliotheken Collider- und Simulationscache löschen.
5. Für normale Bakes Detaildiagnosen deaktiviert lassen.

Aktuelle Produktionsbasis: **Jolt + CoACD + binärer Cache**.

Geplante Zielbasis: **KA Fracture + solverunabhängiger Destruction Core + PhysX/Blast + Jolt-Fallback + PBD-Partikel + Flow/OpenVDB**.

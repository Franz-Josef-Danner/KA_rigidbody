# Architektur 0.5.1

> Dieses Dokument beschreibt die implementierte Ist-Architektur. Die langfristige Destruction-, PhysX-, Blast-, Partikel-, Staub-, Rauch- und Feuerarchitektur ist vollständig in [`README.md`](README.md) dokumentiert.

## Geplante Zielschichten

```text
Geometry Authoring
        ↓
Destruction Asset
Fragment Graph + Bond Graph + Material Profiles
        ↓
Solver-neutrale SimulationScene
        ├─ Jolt Backend
        └─ PhysX/Blast Backend (geplant)
        ↓
Solver-neutraler Event Stream
        ↓
Rigid Debris + PBD Particles + Sparse Volumes (geplant)
        ↓
Versionierte Blender Caches
```

Die Erweiterung erfolgt schrittweise. Der aktuelle Jolt-Pfad muss nach jeder Umstrukturierung funktionsfähig und regressionsgeprüft bleiben.

```text
Blender UI / Properties
        |
        v
Evaluated Mesh Extractor
  |           |
Convex Hull / Voxel Box Compound   Static Triangles
        |
        v
Coordinate + COM Adapter
Blender Z-up <-> Jolt Y-up
        |
        v
Backend Registry
  |                 |
Jolt Native       Reference
Culverin 0.13.2   Testsolver
        |
        v
Backend-neutrale Welttransformationen
        |
        v
Binary Float32 Cache -> frame_change_post -> matrix_world
```

## Verantwortlichkeiten

### Blender-Python

- UI und Properties
- Auswertung von Modifier-Stacks
- Convex-Hull-Erzeugung über BMesh
- Export statischer Dreiecksmeshes
- Volumen- und Dichteberechnung
- Cache-Verwaltung
- Timeline-Wiedergabe
- strukturierte Logs

### Jolt/Culverin

- Broadphase und Narrowphase
- Kontaktmanifolds
- Impuls- und Reibungslösung
- Trägheitstensor und Rotationsdynamik
- CCD
- Multi-Core-Stepping
- natives Sleeping

## Geometrie und Schwerpunkt

Blender-Objektursprung und physikalischer Massenschwerpunkt müssen nicht übereinstimmen. Der Export verschiebt die Kollisionsgeometrie um einen lokalen `shape_center`. Nach der Jolt-Erstellung wird der tatsächlich zurückgegebene Center-of-Mass-Offset kalibriert. Bei jeder Cache-Aufnahme wird daraus wieder die korrekte Blender-Objektmatrix rekonstruiert.

Damit verschiebt ein asymmetrisches Fracture-Fragment beim Start nicht seinen sichtbaren Objektursprung.

## Koordinatensystem

Blender verwendet ein rechtshändiges Z-up-System. Culverin/Jolt verwendet Y-up. Die Konvertierung lautet:

```text
Blender (x, y, z) -> Jolt (x, z, -y)
```

Quaternionen werden durch einen Basiswechsel konvertiert und im Cache wieder in Blenders Reihenfolge `(w, x, y, z)` gespeichert.

## Masse und Trägheit

Bei `Mass Source = Density` wird die Masse aus dem ausgewerteten geschlossenen Blender-Mesh berechnet. Diese Masse wird dem nativen Convex Hull übergeben. Jolt erzeugt daraus den zur Kollisionsform passenden Trägheitstensor.

KA-Fracture-Objekte werden standardmäßig mit dichtebasierter Masse importiert, damit große und kleine Bruchstücke nicht dieselbe Trägheit besitzen.

## Sleeping

Jolt besitzt eigenes Island-Sleeping. Der Produktionsstandard `Native Jolt` verwendet ausschließlich den nativen Inselzustand. Die optionalen Modi `Hybrid Experimental` und `Custom Thresholds` können Low-Motion-Deaktivierungen anfordern; ein Body gilt dabei erst nach erneuter Bestätigung durch `get_active_indices()` als schlafend.

## Diagnostics

Der Jolt-Adapter aggregiert pro Frame:

- aktive und schlafende Körper
- maximale lineare und angulare Geschwindigkeit
- Contact Added/Persisted/Removed
- stärksten Kontaktimpuls
- häufigste und stärkste Objektpaare
- stärksten Kontaktpartner jedes Körpers

Detaillierte Kontakte werden über Culverins strukturierten 128-Byte-Zero-Copy-Buffer einmal pro gerendertem Frame konsumiert. Damit stehen auch Penetrationstiefe und Subshape-Informationen zur Verfügung, ohne pro Kontakt Python-Dictionaries zu erzeugen. Ohne `Detailed Contact Diagnostics` bleibt die Kontaktanalyse deaktiviert. `Log Ausgaben` steuert ausschließlich das Schreiben und aktiviert keine Kontakt- oder Payload-Auswertung.

## Native Runtime

`backends/culverin_loader.py` lädt abhängig von Plattform und Python-Version aus:

- `vendor/culverin/win_amd64_cp313`
- `vendor/culverin/linux_x86_64_cp313`

Die Dateien unter `native/` bleiben ein optionales Scaffold für eine spätere direkte C-ABI-Bridge, werden von Version 0.3.2 aber nicht ausgeführt.

## Nächste Ausbaustufen

1. solverneutrale `SimulationScene` und stabile IDs einführen
2. Fragment Graph, Bond Graph und Materialprofile implementieren
3. vorhandenen Jolt-Pfad auf die neutrale Datenstruktur umstellen
4. solverneutralen Event Stream und Event-Cache ergänzen
5. PhysX-Rigid-Body-Prototyp mit nativen Compound-Collidern aufbauen
6. Blast für Bond-Schaden und Inseltrennung evaluieren
7. PBD-Partikel, volumetrischen Staub, Rauch und Feuer schrittweise ergänzen

Die vollständige Zielarchitektur und der Testplan stehen in [`README.md`](README.md).


## Backend-Auswahl 0.3.2

Persistierte Blender-Properties überleben Add-on-Updates. Deshalb konnte eine mit 0.2.0 gespeicherte Szene trotz neuem Standardwert weiterhin `REFERENCE` verwenden. Die Migration und der Bake-Guard stellen nun sicher, dass Convex-Hull-/Mesh-Szenen tatsächlich über Jolt laufen.


## Stabilization Layer 0.3.4

Vor jeder Simulation läuft `preflight_scene`. Sie verhindert Backend-inkompatible Körperkombinationen und kann sichere Korrekturen direkt an den Blender-Properties durchführen. Dynamische beziehungsweise kinematische Triangle-Mesh-Collider werden dabei zu Convex Hull.

`build_scene_payload` trennt Rohwerte von effektiven Solverwerten. Kleine Körper können unverändert simuliert, durch Mindestmasse/Box-Proxy stabilisiert oder vollständig aus der Solver-Payload ausgeschlossen werden. Die Entscheidung wird pro Körper im Payload dokumentiert.

Convex Hulls werden zunächst vollständig erzeugt. Überschreiten sie das konfigurierte Vertex-Limit, wird eine deterministische Support-Point-Auswahl über Achsen, Raumdiagonalen und sphärisch verteilte Richtungen durchgeführt und daraus erneut eine konvexe Hülle gebildet.

CCD bleibt ein Body-Schalter. Bei aktivem Adaptive CCD wird der Body-Schalter durch Radius- und Geschwindigkeitskriterien eingeschränkt.

Jolt nutzt standardmäßig natives Island-Sleeping. Das frühere zusätzliche Python-Deaktivieren ist nur noch im expliziten `CUSTOM`-Modus aktiv. Kontakt-Einzelereignisse werden nur gelesen und aggregiert, wenn `Detailed Contact Diagnostics` aktiviert wurde.


## Collider Preparation Cache 0.3.5

Version 0.3.5 introduces an in-memory LRU cache for evaluated collision geometry. The cache key is derived from evaluated vertex coordinates, triangulated topology and the body-local scale matrix. Object location, rotation, material values and solver parameters do not invalidate geometry proxies.

The cached entry contains body-local vertices, triangle indices, bounds, closed-mesh volume and lazily generated convex hull variants per vertex limit. Preflight mass validation and payload generation use the same entry. The cache is limited to 512 evaluated geometries and can be cleared from the Stability panel.

Payload profiling records mesh read, vertex transform, volume, hull, mass, signature and total extraction times. Full per-body payload diagnostics remain opt-in.


## Determinism and Quality Layer 0.3.6

Version 0.3.6 introduces a deterministic execution mode. Enabled bodies are sorted by their full Blender name before payload generation, hull support sampling remains directionally deterministic, and the effective Jolt worker count is forced to one. The scene signature now includes add-on version, signature schema, cache schema and bundled Culverin runtime version.

Every completed bake receives a stable SHA-256 digest of frame transforms. If an existing cache has the same scene signature, the new result is compared component-by-component with a configurable tolerance before the cache is replaced. Structural differences, maximum numeric deviation and the exact transform path are stored in the cache and diagnostic log.

Convex proxy quality is now error-driven. The complete hull is sampled with deterministic support directions. Candidate proxies are evaluated through directional support loss in Blender length units. Point budgets are increased until the preset tolerance is reached or the preset maximum is exhausted. The geometry cache key includes all quality parameters, so changing a quality preset invalidates only the corresponding hull variant.

The built-in regression suite creates isolated backend payloads and does not create or modify Blender objects. It covers falling/settling, restitution, stack stability, friction, CCD and repeated deterministic simulation. Reports are written atomically to `ka_rigid_regression.json` and include pass-state changes and runtime differences relative to the previous suite run.


## Stability Hotfix 0.3.7

Version 0.3.7 corrects two production stability regressions introduced by 0.3.6. The Balanced collider preset no longer accepts 24- or 48-point proxies; it retains the proven 64-point baseline because directional support error alone does not preserve contact-face topology. Deterministic body ordering now creates static bodies first, followed by kinematic and dynamic bodies, keeping stable body IDs without moving the ground behind the debris set.


## Automatic Compound Layer 0.4.1

Die Single-Hull-Pipeline bleibt der stabile Basispfad. Bei `AUTO` wird zunächst der vollständige Convex Hull berechnet und seine richtungsabhängige Abweichung gemessen. Nur wenn die Fehlergrenze überschritten wird, erzeugt die Compound-Schicht ein niedrig aufgelöstes, deterministisches Innenvoxelbild. Zusammenhängende Zellen werden zu Boxen verdichtet und bei Bedarf über eine deterministische Clusterung auf das Part-Limit reduziert.

Jede Box wird um `Compound Inset` verkleinert. Die Qualitätsprüfung kombiniert Voxelabdeckung mit einer volumenbewerteten 3×3×3-Abtastung pro Box. Paritätsstrahlen klassifizieren Innen- und Außenpunkte; ein BVH liefert die Entfernung zur Mesh-Oberfläche. Der Proxy wird nur akzeptiert, wenn Innenabdeckung, maximales Außenvolumen, Oberflächenabweichung und Mindestverbesserung gegenüber dem Single Hull gleichzeitig erfüllt sind.

Culverin `create_compound_body` erzeugt aus den Boxen einen einzelnen Jolt-Body mit gemeinsamer Masse und Trägheit. Der bestehende COM-Kalibrierungspfad rekonstruiert weiterhin den Blender-Objektursprung. Die Geometrie-Cache-Signatur enthält Algorithmus, Auflösung, Part-Limit, Inset und Abdeckungsgrenze.

Die Kontaktanalyse aggregiert pro gerendertem Frame die maximale Gleitgeschwindigkeit eines Kontaktpaares. Nur ununterbrochene Phasen mit überwiegend horizontaler Normale und durchgehend niedriger Gleitgeschwindigkeit gelten als Side-Stick. Wenn der Runtime Guard aktiv ist und beide Bodies Compound-Proxies verwenden, werden diese Bodies auf ihre vorhandenen Convex Hulls zurückgesetzt und der Bake einmal vollständig wiederholt.


## Stability and Throughput Layer 0.4.4

Der Balanced-Hull-Pfad endet bei 64 Support-Punkten. Dies reduziert wechselnde Kontaktmanifolds in ruhenden Fracture-Haufen; `Accurate` bleibt für Fälle mit höherem geometrischem Bedarf verfügbar. Ein Versionswechsel des persistenten Hull-Caches erzwingt die einmalige Neuerzeugung der Proxies.

Die automatische Workerzahl wird aus der Zahl dynamischer Bodies abgeleitet. Kleine Szenen werden nicht mehr mit der maximal verfügbaren CPU-Threadzahl übersättigt. Die adaptive Substep-Steuerung bewertet lineare Wegstrecke, Winkeländerung, Collider-Größe und CCD, statt alle dichten Szenen pauschal mit maximaler Frequenz zu rechnen.

Im `STABILIZE`-Modus kann die Payload sehr kleine Solver-Massen relativ zum schwersten dynamischen Body anheben. Die Rohmasse bleibt im Payload und am Blender-Objekt erhalten; nur die an Jolt übergebene effektive Masse wird konditioniert.

Positions-, Rotations- und Geschwindigkeitsbuffer sowie `get_active_indices()` bilden gemeinsam einen konsolidierten Zustandsdurchlauf pro Frame. Detaillierte Kontakte werden gesammelt und nur einmal nach allen Substeps eines gerenderten Frames ausgelesen.


## Managed Ground Invariant 0.4.7

Der mit `KA_Physics_Ground` markierte Boden ist eine geschützte Systemkomponente. Solange er aktiviert ist, gilt auf drei Ebenen dieselbe Invariante:

1. Blender-Operator: `Add Selected Bodies` hält ihn unabhängig von der Sammelauswahl auf `STATIC + PLANE`.
2. Preflight/Payload: abweichende gespeicherte Einstellungen werden repariert; das Payload trägt `managed_ground=true`.
3. Jolt-Adapter: unmittelbar vor der Body-Erzeugung werden Managed Grounds nochmals zu einem statischen unendlichen Plane normalisiert.

Damit kann eine zwischenzeitliche Dynamic-Zuweisung nicht mehr dazu führen, dass eine spätere Static-Zuweisung einen dünnen, potenziell einseitigen Triangle-Mesh-Boden erzeugt.

## Bulk Frame and Cache Layer 0.4.5

Der Jolt-Adapter liest pro gerendertem Frame die Positions-, Rotations-, linearen und angularen Shadow Buffers sowie die aktiven Indizes gemeinsam. Derselbe Durchlauf erzeugt den Blender-Snapshot, aktualisiert Geschwindigkeits- und Energiepeaks, wendet Dämpfung an, bewertet optionale Hybrid-/Custom-Sleep-Kandidaten und schreibt die sieben Float32-Transformkomponenten je Body direkt in einen dichten Frameblock.

Die adaptive Substep-Steuerung verwendet die Zustandsprobe des vorherigen Frames. Dadurch entfällt der bisherige zusätzliche Scan vor jedem Frame. Native Jolt Sleeping benötigt nach dem gemeinsamen Pass keinen zweiten Body-Durchlauf; nur explizite Aktivierungs- oder Deaktivierungsbefehle in den experimentellen Modi werden nach `world.step(0.0)` nochmals gegen `get_active_indices()` bestätigt.

Der Transformcache verwendet Schema 3 mit der Signatur `KARD045`. Das Backend liefert Framefolge, stabile Body-Reihenfolge, Skalierungen und den bereits aufgebauten Float32-Block. Der Writer muss die verschachtelten Playback-Dictionaries dadurch nicht erneut traversieren. Schema 2 aus Version 0.4.4 bleibt lesbar.

Persistente Hulls verwenden `ka_rigid_hulls_v3.kahc`. Metadaten und Float64-Hullkoordinaten werden getrennt mit zlib Level 1 komprimiert und atomar ersetzt. Der frühere gzip-JSON-Cache wird beim ersten Zugriff gelesen und anschließend in das neue Format konvertiert.

Die Thread-Heuristik ist bewusst konservativ: 2 Worker bis 32 dynamische Bodies, 4 bis 750, 6 bis 3.000, 8 bis 10.000 und darüber maximal 12. Ein expliziter Benutzerwert bleibt erhalten; `STRICT` verwendet weiterhin genau einen Worker.

Die native Runtime bleibt Culverin 0.13.2. Deren öffentliches `WorldSettings`-Interface stellt weiterhin nur Gravity, Penetration Slop, Kapazitäten und Threadzahl bereit. Eine Aktualisierung der nativen Jolt/Culverin-Bridge sowie Velocity-/Position-Iterationszahlen, native Sleep-Schwellen und weitere `PhysicsSettings` erfordern neue Windows- und Linux-Binaries und sind daher nicht durch Python-Code austauschbar.



## 0.4.9 Binary bake and independent diagnostics

- Every normal Blender/Jolt bake keeps only the backend-direct Float32 transform block in memory. Python frame dictionaries are available only through an internal regression override.
- Contact collection, side-stick evaluation and detailed body payloads are independent flags. None of them changes the frame-cache representation.
- `Log Ausgaben` controls file/console output only. It does not enable contact collection, side-stick evaluation, body peak tracking or per-body payload serialization.
- The internal Compound Runtime Guard may collect contact data for safety even when user diagnostics are off. Those forced contacts do not enable per-frame contact logging.

## 0.4.8 Collider pipeline

- Convex proxy selection is driven by directional support-plane error. The primary point budget can escalate to a bounded rescue budget before correctness falls back to the complete hull.
- Effective hull tolerance is `max(absolute_tolerance, relative_tolerance * bounding_box_diagonal)`.
- A body may reference a separate low-poly collision-proxy object. Its evaluated vertices are transformed into the body's rotation-local world-unit frame, while mass remains derived from the render object.
- Culverin 0.13.2 still does not expose Jolt velocity/position iteration counts, native per-body damping settings, broadphase optimization commands or explicit batch-add controls. The adapter records these as native-bridge requirements rather than simulating unsupported controls in Python.


## 0.5.0 CoACD Compound Convex pipeline

`COMPOUND_CONVEX` ist eine explizite Body-Einstellung und ersetzt nicht automatisch den schnellen `CONVEX_HULL`-Standard. Die Payload-Erzeugung liest das evaluierte Mesh oder das zugewiesene Collision-Proxy-Mesh, trianguliert es und übergibt die Geometrie an die gebündelte CoACD-1.0.11-Bibliothek. Die reale Fehlertoleranz ist das Maximum aus absolutem Ziel und relativem Anteil der Bounding-Box-Diagonale.

Jedes CoACD-Ergebnis wird erneut konvex verhüllt, support-fehlergesteuert auf das Vertexlimit reduziert und leicht nach innen versetzt. Der Inset verhindert interne Kontakte zwischen benachbarten Teilformen. Cache-Schlüssel enthalten Geometriesignatur, CoACD-Version, Preset, Toleranzen, Auflösungen, Part-Limit, Vertexlimit und Inset. Der persistente `KACL4`-Cache speichert die Float64-Punkte aller Teil-Hulls.

Culverin 0.13.2 stellt `create_compound_body` nur für primitive Teilformen bereit. Konvexe Hull-Kinder können damit nicht zuverlässig als ein nativer `StaticCompoundShape` erstellt werden. Der 0.5.0-Adapter bildet deshalb einen logischen Compound-Convex-Body als Cluster aus mehreren dynamischen Convex-Hull-Bodies ab. Der volumenanteilige Massenwert wird auf die Teile verteilt; der größte Teil ist der Playback-Root und alle weiteren Teile werden durch `CONSTRAINT_FIXED` gebunden. Alle nativen Handles werden demselben logischen Namen zugeordnet, interne Geschwisterkontakte werden aus Diagnosen ausgeblendet, und nur der Root-Transform wird in den Blender-Cache geschrieben.

Diese Darstellung ist funktional und in der Regression geprüft, erzeugt aber mehr native Bodies und Constraints als ein echter Jolt-Compound. Eine spätere Culverin-Erweiterung sollte mehrere Convex-Hull-Kinder direkt in einen Jolt `StaticCompoundShape` überführen; Payload, CoACD-Cache und UI können dabei unverändert bleiben.

## 0.5.1 Blender Enum registration fix

`collision_shape` verwendet eine statische Enum-Liste mit expliziten numerischen IDs. Ein dynamischer `items`-Callback darf in Blender nicht gleichzeitig einen String-`default` besitzen; genau diese Kombination verhinderte in 0.5.0 die Registrierung der `KA_RIGID_BodySettings`. Die IDs 0–4 entsprechen der Reihenfolge bis 0.4.9, `COMPOUND_CONVEX` verwendet die neue ID 5.

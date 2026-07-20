# Architektur 0.7.16

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

Alle Bodies werden unabhängig von ihrer Herkunft konfiguriert. Dynamische Meshes können wahlweise eine feste Masse oder eine dichtebasierte Masse verwenden.

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

Jedes CoACD-Ergebnis wird erneut konvex verhüllt, support-fehlergesteuert auf das Vertexlimit reduziert und leicht nach innen versetzt. Der Inset verhindert interne Kontakte zwischen benachbarten Teilformen. Cache-Schlüssel enthalten Geometriesignatur, CoACD-Version, Preset, Toleranzen, Auflösungen, Part-Limit, Vertexlimit und Inset. Der persistente `KACL5`-Cache speichert die Float64-Punkte aller Teil-Hulls.

Culverin 0.13.2 stellt `create_compound_body` nur für primitive Teilformen bereit. Der historische 0.5.0-Adapter verwendete deshalb mehrere Convex-Hull-Bodies mit `CONSTRAINT_FIXED`. Dieser Cluster-Pfad wurde in 0.6.3 entfernt. Seit 0.6.4 erzeugt der Windows-Fallback keine Boxen mehr aus offenen Oberflächen-Clustern. Stattdessen werden ausschließlich konservative, im geschlossenen Quellmesh geprüfte Innenraum-Boxen an einen einzigen `create_compound_body`-Aufruf übergeben. Damit entstehen auch ohne ABI-v2-Bridge genau ein nativer Body, eine gemeinsame Masse/Trägheit und keine internen Constraints.

Mit installierter ABI-v2-Bridge bleiben die eigentlichen Convex-Hull-Kinder erhalten und werden in einem Jolt `StaticCompoundShape` zusammengefasst. Ohne Bridge ist die OBB-Darstellung etwas gröber, dafür stabil und frei von Geschwisterkontakten und Constraint-Drift.


## 0.6.0 SimulationScene und nativer Jolt-Bridge

`core/simulation_scene.py` definiert den ersten verbindlichen solverneutralen Vertrag. Blender extrahiert weiterhin die bewährten Geometriedaten, speichert sie anschließend aber als `ka.simulation_scene` Version 1. Die Backends erhalten ihren Kompatibilitäts-Payload ausschließlich über den Adapter `solver_payload()`. Damit ist das neue Schema bereits die Quelle der Solver-Eingabe, ohne den bestehenden Bake-/Cache-Loop gleichzeitig vollständig ersetzen zu müssen.

Persistente UUIDs werden als Blender-Custom-Properties auf Szene und Bodies gespeichert. Collider- und Compound-Child-IDs werden deterministisch aus der Body-ID und ihrem Inhalt abgeleitet. Objektname und Anzeigename bleiben Metadaten; die Einfügereihenfolge in den Solver basiert auf Body-Typ und UUID. Kopierte doppelte UUIDs werden beim Szenenaufbau repariert.

Der optionale native Bridge verwendet eine kleine C-ABI Version 2 zwischen Python und Jolt 5.6.0. Er unterstützt Primitive, Convex Hull, statische Triangle Meshes und echte Compound-Convex-Shapes. Bei einem CoACD-Compound werden alle Child-Hulls in ein `StaticCompoundShapeSettings` geschrieben und als genau ein Jolt-Body erzeugt. Masse und Trägheit werden damit über die Gesamtform berechnet; interne Fixed Constraints entfallen.

Die Runtime sucht zuerst nach einer konfigurierten oder gebündelten ABI-v2-Bibliothek. Ist keine valide Bibliothek vorhanden, wird Culverin 0.13.2 geladen. Seit 0.6.4 erzeugt dieser Fallback genau einen Compound-Body aus konservativen Innenraum-Boxen; der frühere Fixed-Constraint-Cluster und die überfüllenden Oberflächen-OBBs werden nicht mehr verwendet. Diese Unterscheidung wird in Status und Logs ausdrücklich ausgewiesen.

Der native Quellpfad liegt unter `native/jolt/`. CMake pinnt Jolt auf Tag `v5.6.0`; alternativ kann ein lokaler Source-Checkout über `KA_JOLT_SOURCE_DIR` verwendet werden. Kompilierte Bibliotheken gehören ausschließlich unter `vendor/jolt_bridge/<platform>/` oder werden über die Add-on-Einstellungen ausgewählt.

## 0.5.1 Blender Enum registration fix

`collision_shape` verwendet eine statische Enum-Liste mit expliziten numerischen IDs. Ein dynamischer `items`-Callback darf in Blender nicht gleichzeitig einen String-`default` besitzen; genau diese Kombination verhinderte in 0.5.0 die Registrierung der `KA_RIGID_BodySettings`. Die IDs 0–4 entsprechen der Reihenfolge bis 0.4.9, `COMPOUND_CONVEX` verwendet die neue ID 5.


### 0.6.4 Initial-Overlap-Schutz

Der Collider-Aufbau akzeptiert native-freie Compound-Proxys nur, wenn ihr berechnetes Runtime-Volumen höchstens 102 % des geschlossenen Quellvolumens beträgt. Zusätzlich sammelt der Jolt-Adapter im ersten simulierten Frame immer Kontaktstatistiken für Compound-Szenen. Überschreiten neue Kontakte oder Kontakt-Ereignisse eine körperzahlabhängige Schwelle, verwirft der Operator den ersten Lauf und backt die Compound-Bodies automatisch mit ihren vorbereiteten Single-Hull-Fallbacks neu. Der ausgegebene Cache enthält deshalb keine bekannte initiale Überlappungs-Explosion.

Cache-Frame 1 wird nicht aus Culverins anfangs noch leeren Transform-Puffern gelesen, sondern direkt aus den unveränderlichen Blender-Eingangstransformationen aufgebaut.

### 0.6.5 Anti-Stick-Materialprofil

Die 0.6.4-Innenraum-Proxys verhindern anfängliche Collider-Explosionen, beseitigen aber nicht automatisch statische Reibungsverbände in einem dichten Fragmenthaufen. Der reale Testcache zeigte lang anhaltende Kontakte mit fast horizontalen Normalen und nur wenigen Millimetern pro Sekunde Relativbewegung. Solche Paare konnten als gemeinsame Jolt-Insel einschlafen und visuell wie verklebt wirken.

Es gibt keine namens- oder tagbasierte Materialerkennung. Reibung, Masse und Dichte werden ausschließlich pro Body eingestellt. Der Standardwert für `mPenetrationSlop` beträgt 1 mm statt 5 mm.



## 0.7.0 Breakable Bonds MVP

Breakable cohesion is stored as a solver-neutral graph in `SimulationScene.constraints`. Every bond has a stable UUID, two stable body UUIDs, a world-space anchor and normal, estimated contact area, independent force and torque limits, optional accumulated damage, and a deterministic intact/broken state. The Blender scene persists the authored graph as compact JSON; object names are metadata only.

The authoring operator builds proximity bonds from evaluated world-space mesh vertices. AABB sweep pruning limits candidate pairs. A valid connection requires at least three nearby samples and measurable non-collinear surface support, so meshes touching only at one point or along a single edge are not intentionally bonded.

In `Flexible` mode Culverin 0.13.2 creates a bounded native Jolt Fixed-constraint network. In `Rigid` mode every connected intact dynamic bond component is instead represented by one native compound actor, while the complete authored graph remains solver-neutral runtime state. Culverin does not expose constraint reaction lambdas, therefore both modes evaluate external contact impulse per solver substep. The impulse is converted to an estimated force using the substep duration, distributed across the intact bonds of the impacted logical fragment, projected against the bond orientation and converted to an estimated torque around the bond anchor. Exceeding either threshold marks the bond broken and emits a deterministic `BOND_BREAK` event. Optional damage accumulation integrates repeated subcritical loading above a fixed activation ratio.

The ABI-v2 bridge currently has no external-constraint ABI. Bond scenes therefore select the bundled Culverin runtime even when ABI-v2 is installed. Exact reaction-force fracture remains a later ABI extension.

## 0.7.7 Collision Coverage and Motion-Aware CCD

The production collider path no longer treats any non-empty interior-box decomposition as a valid dynamic contact representation. Such under-approximations can be stable for the solver while allowing the visible fragment to extend deeply through a plane. Without the optional ABI-v2 convex-compound bridge, a decomposed body therefore falls back to its complete fitted convex hull. Intact rigid bond components similarly use one component-wide outer convex hull built from authored collider support points rather than an interior primitive cloud.

This fallback deliberately prefers collision coverage over concavity accuracy. It can bridge cavities, but it cannot produce the severe visible underfill caused by proxies representing only a small fraction of source volume. True multi-hull concavity remains available through the compiled native bridge.

Requested dynamic CCD bodies are now created with Jolt `LinearCast` regardless of their start-frame velocity. Jolt performs its own movement thresholding, which is essential for sleeping fracture pieces that are accelerated only after an impact. Adaptive substeps estimate both translational travel and angular surface travel and compare the result with the smallest relevant collider feature length.

Collider cache schema `KACL8` invalidates earlier interior-proxy entries. Simulation and collider caches must both be rebuilt after migration.

## 0.7.6 Authored Ground Rest

A zero-velocity `Rigid` bond component can already be authored in a valid resting pose on a managed horizontal ground plane. Creating that actor active allowed gravity and conservative proxy contacts to make it settle and rock for several frames before Jolt's native sleep threshold was reached. The visible mesh could consequently sink or tilt even though the intended initial pose was already stable.

The backend now evaluates the lowest world-space collider support against managed plane height during the initial rigid-cluster build. A component within the narrow support tolerance is created with zero velocity, assigned Culverin's geometrically recentered transform, and immediately deactivated. Logical fragment transforms therefore remain exactly equal to cache frame 1. This path is used only during initial construction; actors rebuilt after bond fracture remain active. Jolt automatically wakes the sleeping compound when another active body strikes it.

Diagnostics count these initial deactivations as `bond_supported_cluster_deactivations`. Regression coverage verifies both exact pose retention and impact wake-up.

## 0.7.5 Safe Single-Hull Children in Rigid Islands

A rigid bond island is still represented by one native compound actor. Culverin cannot attach arbitrary convex-hull children to that actor, so a `CONVEX_HULL` fallback must be converted to primitive children. Version 0.7.4 used the source mesh half-extents for this conversion. Those extents can be much larger than the fitted/inset hull and may cross the ground even when the actual convex collider does not.

Version 0.7.5 reconstructs deterministic support planes from the fallback hull vertices. It places a central sphere and inward-offset vertex spheres whose radii are limited by the nearest support plane. Every generated primitive is therefore contained by the convex hull. The original convex body is still used when a fragment becomes a standalone actor after fracture; the sphere cloud is limited to the temporary one-body rigid-island representation.

## 0.7.4 Rigid Compound Bond Islands

`Rigid` cohesion no longer approximates a solid object with a network of iterative constraints. Every connected intact dynamic bond component is merged into one native dynamic compound actor. Logical fragments retain their own transforms, stable IDs, masses, collider parts and bond endpoints, but their visible transforms are reconstructed from immutable local frames inside the actor. Relative translation and rotation are therefore mathematically zero while the graph remains connected.

Contact events emitted for the compound actor are assigned to the nearest logical member at the reported contact position. The existing impulse-based fracture estimator can therefore continue to load local bonds. After one or more bonds break, connected components are recomputed, the old actor is destroyed, and every resulting component is rebuilt as either a new compound actor or a normal singleton body. Linear and angular velocity are transferred to the new actors.

Culverin recentres compound actor poses to the volume-weighted centre of their primitive children. The backend compensates logical member frames for this shift while keeping the supplied actor position at the island mass centre. This prevents jumps for unequal fragment masses and during graph splits.

The 256-constraint world limit now applies only to `Flexible`; `Rigid` creates no Fixed constraints. Internal collision filtering and coordinated multi-body sleep are unnecessary in `Rigid`, because one native actor has neither sibling contacts nor partial sleeping members.

## 0.7.3 Native Bond Islands and Coordinated Sleep

Version 0.7.3 removes the post-step transform projection introduced for rigid bond islands. Repositioning every member after each contact solve created a feedback loop between contact correction, Fixed constraints and the projection pass; on dense asymmetric compound colliders this injected angular and linear energy into the complete object.

`Rigid` now relies exclusively on the deterministic native Fixed-constraint spanning forest plus reinforcement edges. The intact graph receives collision filtering as before, but no member transform or velocity is overwritten after a solver step. Once a connected intact component is supported only by static or kinematic contacts and its aggregate motion remains below the settle limits, every dynamic member is zeroed and deactivated in one batch. This avoids the unstable partial-sleep state in which active constraints pull against already sleeping fragments. `Flexible` retains native constraints without this coordinated whole-island sleep gate.

Projection diagnostic fields remain in bake totals for cache/log compatibility and stay at zero.

## 0.7.2 Bond-Island Collision Filtering

Intact fracture fragments must not solve contacts against other members of the same intact bond component. Otherwise the contact solver separates overlapping or merely touching collider approximations, while the rigid post-step projection immediately restores the authored relative pose. Repeating those opposing corrections injects momentum into the complete cluster and prevents reliable rest.

The Culverin fallback therefore assigns every intact multi-body bond component a deterministic free collision-category bit and rebuilds all masks from the original per-body layer/mask compatibility. Members of one intact component exclude their own component category. External bodies and different components remain compatible. When bond damage disconnects the graph, filters are rebuilt immediately; the newly separate components regain mutual collision. Singletons revert to their original category.

The current 16-bit filter space supports up to fifteen simultaneous filtered multi-body components when the normal default category occupies one bit. Overflow components retain their original filters rather than disabling unrelated collisions.

## 0.7.1 Rigid Bond Islands

Culverin 0.13.2 has a hard limit of 256 constraints per world. Dense fracture assets can contain substantially more authored bonds, so simply creating constraints in UUID order produces arbitrary holes in the mechanical graph. Version 0.7.1 first builds a maximum-area spanning forest over every authored bond island and then spends the remaining native budget on the largest unused interfaces. All authored bonds remain runtime records even when they do not own a native constraint.

Version 0.7.1 initially added a post-step rigid transform projection to suppress visible joint stretch. Version 0.7.3 removes that mechanism because solver/projection feedback could inject global drift into dense compound fracture assets. Native Fixed constraints now provide the mechanical cohesion, while `Rigid` adds coordinated whole-island settling.

Bond anchors and normals are stored author-side in Blender world coordinates but converted at bake initialization into local frames for both endpoint bodies. Force alignment and torque levers use the current transformed anchor, not the original world point. Contact samples whose other body belongs to the same intact bond component are classified as internal and excluded from the external fracture-load estimate.

## 0.7.8 Sharp Ground Contact

Jolt convex hulls use a rounded convex radius by default. The optional native ABI bridge now disables that radius for authored fracture hulls. Culverin 0.13.2 does not expose the setting, so its fallback keeps the native simulation untouched and corrects only exported Blender transforms when sharp source-hull vertices penetrate KA's managed horizontal ground plane. Rigid bond-cluster members share one correction to preserve cohesion.



## 0.7.11 Rigid Static-Anchor Contact Filtering

A Rigid Dynamic-Dynamic island is represented by one complete outer convex hull. This coverage-safe hull can fill authored gaps and therefore overlap a Static support that is already attached through a Dynamic-Static Fixed anchor. Solving both the overlap contact and the Fixed constraint on the same actor pair produced an immediate depenetration and rotation between cache frames 1 and 2.

Every anchored dynamic actor now receives a free collision-category bit. Masks are rebuilt from the original compatibility rules while excluding only intact dynamic-actor/static-endpoint pairs. The Static body continues to collide with unrelated bodies, and the dynamic island continues to collide with all non-anchor statics. Whenever an anchor breaks and the rigid topology is rebuilt, the filter is rebuilt as well; normal collision is restored for the released pair. If the 16-bit category space is exhausted, overflow actors retain normal collision and the limitation is reported in diagnostics rather than disabling unrelated contacts.

## 0.7.9 Rigid Dynamic-Static Anchors

Rigid cohesion continues to represent every intact Dynamic-Dynamic bond island as one native actor. Intact Dynamic-Static bonds are now recreated as native Fixed constraints after every island rebuild, so a rigid fracture cluster can be mechanically attached to a static support while retaining authored break force and torque. Explicit Selected Only generation may include the managed ground; global generation still excludes it to prevent accidental scene-wide anchoring.


## 0.7.13 Component Mass Conditioning and Mass-Aware Bond Loads

When a scene contains authored Dynamic-Dynamic bonds, solver-only mass conditioning is scoped to each connected dynamic bond component. Independent projectiles are separate components and keep their authored mass. Scenes without authored dynamic bonds retain the legacy global loose-pile conditioning path.

For each bond-monitored contact substep, the backend captures actor position, linear velocity and angular velocity before Jolt resolves contact. Contact-point velocities are reconstructed for both participants, projected onto the contact normal and combined with reduced collision mass. Bond loading uses the larger of the raw native contact impulse and this pre-solver momentum impulse. This avoids mass-invariant fracture behavior when the Culverin contact record reports a bounded impulse.

After a bond break, the Rigid graph is rebuilt from the current solver poses and velocities. Surviving components and Static anchors are recreated at the impact state, preventing released fragments from being reconstructed at their authored start pose.

## 0.7.12 Anchored Neighbouring-Static Rest Filtering

Pairwise filtering of only the authored Dynamic-Static endpoint is not sufficient when Culverin represents a multi-fragment rigid island by one complete outer convex hull. The outer hull can overlap a neighbouring Static fragment that is not directly bonded to the island, even though the original authored meshes merely meet along irregular fracture surfaces. Resolving that overlap on frame 2 rotates and translates the anchored island.

During every rigid topology rebuild, the backend now computes world-space actor bounds from the authored collider points before simulation advances. For each dynamic actor with at least one intact Static anchor, any additional Static actor whose authored bounds overlap the rigid-island envelope is added to the temporary anchor collision exclusions. These support-neighbour exclusions share the same dedicated category bit as the authored anchor pair, require no additional filter bits, and remain active only while the actor has an intact Static anchor. The next rebuild after the last anchor breaks restores the original collision compatibility.

## 0.7.14 Authored Rope and Rod Constraints

Authored mechanical constraints are represented by dedicated Blender Empty objects with persistent constraint UUIDs. A Distance record references two persistent body UUIDs and stores either a unilateral Rope range (`min = 0`, `max = length`) or a fixed Rod range (`min = max = length`).

Culverin 0.13.2 creates these joints after rigid bond-island rebuilding so a constrained fragment resolves to its final native actor. The active compatibility API binds native body centers. For a freely placeable suspension point, the add-on creates a tiny Static sphere at the 3D cursor with collision mask zero; the Dynamic body then swings around that exact anchor location.

The native world limit remains 256 total constraints. Existing compound and bond constraints keep their established priority; authored Distance constraints use the remaining native capacity. ABI-v2 currently has no external-joint interface, so any scene containing authored constraints deliberately selects bundled Culverin.

## 0.7.16 Constraint Lifecycle During Fracture Rebuilds

Rigid fracture islands are rebuilt whenever a bond breaks. Dynamic bodies that are not endpoints of the authored bond graph are now excluded from this topology rebuild, so projectiles, wrecking balls and other independent actors retain their native generational handles.

If a Distance constraint endpoint does belong to a rebuilt fracture island, the backend destroys the native Distance constraint before replacing any endpoint actor and recreates it immediately afterwards from the persistent body UUIDs. A non-empty UUID never falls back to an object name, preventing silent rebinding to another object after duplication or topology changes.

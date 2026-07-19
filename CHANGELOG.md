## Version 0.7.6

- Zero-velocity `Rigid` bond islands that are already supported by a managed horizontal ground plane now start deactivated at their exact authored pose instead of settling under gravity before native sleep.
- The Culverin compound actor receives its geometrically recentered world transform before deactivation, preventing a centre-of-mass offset jump in the initial sleeping state.
- Sleeping rigid islands remain fully dynamic and are automatically awakened by external impacts; a dedicated projectile regression verifies normal wake-up and motion.
- Bake diagnostics now report `bond_supported_cluster_deactivations`.
- Added `Rigid authored ground rest` and `Rigid authored ground wake`; the complete regression suite now passes 31 of 31 tests.
- Add-on, manifest, migration and signature schema were raised to 0.7.6 / schema 24. Existing simulation caches must be cleared and rebaked; the collider cache can remain.

## Version 0.7.5

- Rigid bond compounds no longer convert `CONVEX_HULL` fallback fragments into their full source-mesh bounding boxes.
- Culverin rigid islands now approximate single-hull fallback children with deterministic interior sphere clouds derived from convex support planes. The proxies cannot protrude beyond the authored convex hull.
- This removes the start-frame depenetration jump caused by fallback boxes extending below the ground; the reported 116-fragment scene drops from roughly 11 cm upward displacement to a small physical settling motion of a few millimetres.
- Added the `Rigid hull fallback ground contact` regression, which reproduces the former 15 cm jump and verifies zero unintended contact or upward displacement.
- Add-on, manifest, migration and signature schema were raised to 0.7.5 / schema 23. Existing simulation caches must be cleared and rebaked; the collider cache can remain.

## Version 0.7.4

- `Rigid` simuliert jede zusammenhängende intakte Bond-Insel jetzt als genau einen nativen dynamischen Compound-Actor. Zwischen den enthaltenen Fragmenten existiert deshalb keine solverbedingte Relativbewegung und keine sichtbare Gummi-Dehnung.
- Der vollständige Bond-Graph bleibt unabhängig vom Actor erhalten. Sobald Bruchereignisse den Graphen trennen, wird der bisherige Actor deterministisch zerstört und für jede neue Insel beziehungsweise jedes Einzelstück neu aufgebaut.
- Der bisherige Culverin-Grenzwert von 256 Fixed Constraints betrifft nur noch `Flexible`. `Rigid` verwendet keine Fixed Constraints und kann daher alle 472 Bonds des Referenzobjekts ohne mechanische Lücken abbilden.
- Kontaktpunkte eines Compound-Actors werden dem räumlich nächsten logischen Fragment zugeordnet, damit äußere Einschläge weiterhin lokale Bond-Lasten und Bruchereignisse erzeugen können.
- Eine Kompensation für Culverins volumenbasierte Compound-Rezentrierung verhindert Positionssprünge bei stark asymmetrischen Massenverteilungen und nach Actor-Splits.
- Diagnoseausgaben melden `RIGID_COMPOUND_ISLANDS`, Actor-Neuaufbauten, Clusteranzahl und die Anzahl logisch enthaltener Fragmente.
- Add-on-, Manifest-, Migrations- und Signaturschema wurden auf 0.7.4 beziehungsweise Schema 22 erhöht. Alte Bakes müssen gelöscht und neu erzeugt werden.
- Die Regression umfasst 28 Tests; alle 28 bestehen. Zusätzlich wurde das Split/Rebuild-Verhalten mit tatsächlichen Bond-Brüchen geprüft.

## Version 0.7.3

- Die permanente Post-Step-Transformation intakter Bond-Inseln wurde vollständig entfernt. Diese Projektion konnte bei dichten, asymmetrischen Compound-Collidern Kontaktkorrekturen in globale Translation und Rotation zurückspeisen.
- `Rigid` verwendet jetzt ausschließlich den deterministischen nativen Fixed-Constraint-Spanning-Forest samt Verstärkungskanten; keine Fragment-Transformation und keine Geschwindigkeit wird nach einem Solver-Schritt überschrieben.
- Intakte, statisch gestützte Bond-Inseln werden nach Unterschreiten der aggregierten Bewegungsgrenzen als vollständige Komponente gemeinsam gestoppt und deaktiviert. Partielles Einschlafen einzelner verbundener Fragmente wird dadurch vermieden.
- `Flexible` bleibt der reine native Fixed-Constraint-Pfad ohne koordiniertes Insel-Sleeping.
- Diagnoseausgaben melden die Strategie `NATIVE_FIXED_ISLAND_SLEEP`; alte Projektionsfelder bleiben aus Kompatibilitätsgründen erhalten und stehen auf null.
- Add-on-, Manifest-, Migrations- und Signaturschema wurden auf 0.7.3 beziehungsweise Schema 21 erhöht. Alte Bakes müssen gelöscht und neu erzeugt werden.

## Version 0.7.2

- Behebt die globale Drift intakter Bond-Objekte. Fragmente innerhalb derselben intakten Bond-Insel kollidieren während der Simulation nicht mehr gegeneinander; dadurch kann der Kontaktlöser sie nicht auseinanderdrücken, während die starre Inselprojektion sie im selben Substep wieder zusammenzieht.
- Die Kollisionsfilter werden nach jedem tatsächlichen Bond-Bruch aus dem verbleibenden Bond-Graphen neu aufgebaut. Neu getrennte Inseln und einzelne Fragmente kollidieren dadurch wieder normal miteinander.
- Die Filterzuweisung respektiert die ursprünglichen Collision-Layer/-Masken gegenüber externen Bodies und verwendet deterministisch freie 16-Bit-Kategorien für zusammenhängende Mehrkörper-Inseln.
- Neue Diagnosewerte protokollieren gefilterte Inseln, betroffene Bodies, Filter-Neuaufbauten und einen möglichen Kategorien-Overflow.
- Eine neue Regression prüft absichtlich überlappende, starr verbundene Bodies auf null interne Kontakte und null spontane Verschiebung.
- Add-on-, Manifest-, Migrations- und Signaturschema wurden auf 0.7.2 beziehungsweise Schema 20 erhöht.

## Version 0.7.1

- Behebt die gummiartige Verformung intakter Fracture-Objekte. Im neuen Standardmodus `Rigid` wird jede zusammenhängende intakte Bond-Insel nach jedem nativen Substep auf eine gemeinsame starre Transformation projiziert.
- Die Projektion passt die Inselbewegung massengewichtet an alle aktuellen Fragmentpositionen und -geschwindigkeiten an. Dadurch wird kein beliebiges Fragment als alleiniger Bewegungsgeber verwendet und Einschlagimpuls wird nicht einfach verworfen.
- Culverin 0.13.2 erlaubt maximal 256 Constraints. Statt Bonds nach UUID abzuschneiden, erzeugt der Backendpfad nun zuerst einen flächengewichteten Spannbaum für jede Bond-Insel und nutzt das restliche Budget für Verstärkungsverbindungen. Der vollständige Bond-Graph bleibt für Zusammenhalt und Bruchtopologie erhalten.
- Weltanker und Bond-Normalen werden nun in lokale Fragmentkoordinaten überführt und während der Simulation mitgeführt. Drehmoment wird daher nicht mehr gegen den ursprünglichen Weltpunkt gemessen, wenn sich die Statue bereits bewegt oder gedreht hat.
- Kontakte zwischen Fragmenten derselben noch intakten Bond-Insel werden nicht mehr als äußere Bruchlast ausgewertet.
- Cache- und Log-Metadaten unterscheiden jetzt vollständige Graph-Bonds, tatsächlich angelegte native Constraints, Spannbaum- und Verstärkungsverbindungen sowie die maximale starre Korrektur.
- Neuer UI-Schalter `Cohesion Mode`: `Rigid` für Stein, Beton und Statuen; `Flexible` behält ausschließlich das frühere native Fixed-Constraint-Verhalten.
- Add-on-, Manifest-, Migrations- und Signaturschema wurden auf 0.7.1 beziehungsweise Schema 19 erhöht.

## Version 0.7.0

- Neuer Bereich `Breakable Cohesion` im KA-Fracture-Panel mit `Enable Bonds`, Verbindungsdistanz, Bruchkraft, Bruchmoment und optionaler Schadensakkumulation.
- `Generate Bonds` erzeugt einen deterministischen Nachbarschaftsgraphen zwischen aktivierten Mesh-Bodies. `Selected Only` ersetzt nur die Verbindungen innerhalb der Auswahl; andere gespeicherte Bonds bleiben bestehen.
- Bond-Datensätze verwenden persistente UUIDs, Mittelpunkt, Normale, geschätzte Kontaktfläche, Schwellenwerte, Schaden und Status und werden als `constraints` in `SimulationScene v1` gespeichert und validiert.
- Der Jolt/Culverin-Backendpfad erzeugt echte Fixed Constraints und prüft Kontaktimpulse nach jedem Substep. Überschreitet die geschätzte Kraft oder das geschätzte Drehmoment den Grenzwert, wird der Constraint sofort entfernt und beide Bodies werden aktiviert.
- Brüche werden als `BOND_BREAK` mit Frame, Substep, Impuls, geschätzter Kraft, Drehmoment und beteiligten Bodies protokolliert. Cache-Metadaten enthalten außerdem den finalen Zustand jedes Bonds.
- Bond-Szenen verwenden automatisch Jolt. Solange ABI-v2 keine externen Constraints anbietet, wird für solche Szenen gezielt auf den gebündelten Culverin-Pfad zurückgefallen.
- Die aktuelle Bruchlast ist ein Kontaktimpuls-basiertes Näherungsmodell. Echte Constraint-Reaktionsimpulse und ein Actor-/Insel-Splitting nach Bond-Graph bleiben die nächste Qualitätsstufe.
- Add-on-, Manifest-, Migrations- und Signaturschema wurden auf 0.7.0 beziehungsweise Schema 18 erhöht.

## Version 0.6.5

- Behebt verbleibendes seitliches Aneinanderkleben von Bruchstücken nach der 0.6.4-Overlap-Korrektur. Der ausgewertete Testcache enthielt bis zu 58 zusammenhängende Frames nahezu horizontaler Niedriggeschwindigkeitskontakte, bevor die Bodies gemeinsam einschliefen.
- Erkannte KA-Fracture-Bodies erhalten bei neuer Zuweisung standardmäßig `Fracture Friction = 0.20` statt des allgemeinen Body-Standards 0.50. Dadurch können Teile aus seitlichem Druckkontakt abrutschen, ohne die Bodenreibung statischer Objekte zu verändern.
- Neue Szeneneinstellung `Fracture Friction` im KA-Fracture-Panel. Sie wird bei `Add Selected Bodies` und `Import KA Fracture Pieces` auf erkannte Bruchstücke angewendet.
- Die Jolt-Standardeinstellung `Penetration Slop` wurde für die kleinen Bruchstücke von 5 mm auf 1 mm reduziert, damit seitliche Kontaktmanifolds nicht unnötig tief bestehen bleiben.
- Die Migration 0.6.5 ändert nur noch unangetastete Altwerte: Fracture-Bodies mit exakt 0.50 Reibung und Szenen mit exakt 5 mm Slop werden aktualisiert. Abweichende benutzerdefinierte Werte bleiben erhalten.
- Signaturschema, Add-on-, Manifest- und Runtime-Version wurden aktualisiert. Die Regression umfasst nun 25 Tests einschließlich eines dauerbelasteten seitlichen Anti-Stick-Kontakttests.

## Version 0.6.4

- Behebt die anfängliche Explosionsreaktion von `Compound Convex`: Der bisherige Windows-Fallback bildete offene Dreiecksgruppen als orientierte Boxen ab und konnte dadurch ein Vielfaches des tatsächlichen Mesh-Volumens belegen.
- Die native-freie Windows-Zerlegung erzeugt jetzt ausschließlich konservative Innenraum-Boxen. Jede Rasterzelle wird nur übernommen, wenn ihr Mittelpunkt und alle acht Ecken innerhalb des geschlossenen Ausgangsmeshes liegen; bei Unsicherheit wird sie verworfen.
- Ein strikter Volumenwächter verwirft Compound-Proxys, sobald ihre Runtime-Teilformen das Quellvolumen um mehr als zwei Prozent überschreiten; das Objekt fällt dann auf den vorbereiteten einzelnen Convex-Hull zurück.
- Ein automatischer Initial-Overlap-Guard sammelt im ersten Simulationsframe Kontaktstatistiken. Bei einem unplausiblen Kontakt-Burst werden alle betroffenen Compound-Bodies automatisch als Single Hull neu gebacken, bevor der Cache übernommen wird.
- Frame 1 des binären Transform-Caches verwendet nun exakt die Blender-Eingangstransformationen. Culverins noch nicht initialisierte Zero-Copy-Puffer können dadurch nicht mehr alle Bodies am Ursprung speichern.
- Der Collider-Cache wurde auf `ka_rigid_colliders_v7.kahc` erhöht; alle älteren Compound-Proxys werden neu aufgebaut.
- Signaturschema, Add-on-, Manifest- und Runtime-Version wurden aktualisiert. Die Regression umfasst 24 Tests; alle 24 bestehen.

## Version 0.6.3

- Windows Compound Convex no longer launches the CoACD DLL by default. The DLL can display a modal MSVC assertion and permanently block Blender.
- Added deterministic native-free spatial compound decomposition for Windows.
- Added invalid, duplicate and zero-area triangle cleanup before every decomposition path.
- Native CoACD remains isolated on supported non-Windows systems; Windows native use is opt-in through `KA_COACD_NATIVE_WINDOWS=1`.
- Native worker assertions are redirected to stderr and OS error dialogs are suppressed as a second safety layer.
- Collider proxy cache moved to `ka_rigid_colliders_v6.kahc`; earlier compound results are rebuilt.
- Reduced the isolated worker timeout and guaranteed a single-hull fallback on decomposition errors.

## Version 0.6.2

- Convex-Hüllen erkannter KA-Fracture-Teile werden standardmäßig um 1 mm nach innen versetzt. Dadurch erhalten anfangs exakt aneinanderliegende Bruchstücke einen kleinen Solver-Abstand und bleiben nicht über lange seitliche Kontaktmanifolds aneinander hängen.
- Der neue Regler `Fracture Separation` steuert diesen Abstand. Normale, nicht als Fracture erkannte Convex-Objekte bleiben unverändert.
- CoACD läuft für uncached Compound-Convex-Collider in einem separaten Python-Prozess. Ein nativer CoACD-Absturz beendet daher nicht mehr Blender; das betroffene Objekt fällt kontrolliert auf einen einzelnen Convex-Hull zurück.
- Ohne ABI-v2-Jolt-Bridge wird ein Compound Convex nicht mehr in mehrere Bodies mit Fixed-Constraints zerlegt. Culverin erzeugt jetzt genau einen Compound-Body aus deterministischen orientierten Box-Teilformen.
- Der persistente Collider-Cache wurde auf `ka_rigid_colliders_v5.kahc` erhöht und speichert zusätzlich die Teilform-Boxen. Alte v4-Proxies werden automatisch neu erzeugt.
- Signatur-Schema, Add-on-, Manifest- und Runtime-Version wurden aktualisiert; bestehende Simulationscaches müssen neu gebacken werden.

## Version 0.6.1

- Behebt einen Bake-Abbruch im Preflight: `core.scene_io._evaluated_geometry_entry()` verwendete `_vector_list()`, obwohl die Hilfsfunktion beim 0.6.0-Umbau nicht in `scene_io.py` übernommen worden war.
- `_vector_list()` konvertiert `mathutils.Vector` und andere numerische Sequenzen nun zentral in JSON-sichere Float-Listen.
- Add-on-, Manifest- und Runtime-Version wurden auf 0.6.1 erhöht. Simulationscaches aus 0.6.0 werden wegen der Versionssignatur neu gebacken.

## Version 0.6.0

- Führt `SimulationScene v1` als solverneutralen, versionierten Datenvertrag ein. Der Jolt-Adapter rekonstruiert seinen bisherigen Solver-Payload nun aus diesem Schema.
- Szenen, Bodies, Collider und Compound-Child-Hulls erhalten persistente UUIDs. Umbenennungen ändern dadurch nicht mehr die Solver-Reihenfolge oder die interne Identität.
- Ergänzt einen optionalen nativen ABI-v2-Bridge für Jolt Physics 5.6.0. Ist eine kompilierte Bridge installiert, wird ein `Compound Convex` als ein einzelner Jolt-Body mit `StaticCompoundShape` erzeugt.
- Ohne Bridge bleibt Culverin 0.13.2 automatisch aktiv. Dessen bisheriger Fixed-Constraint-Cluster ist nur noch der Kompatibilitäts-Fallback.
- Native Body-Deskriptoren übertragen Masse, automatisch berechnete Trägheit, Reibung, Restitution, lineare und angulare Dämpfung sowie CCD direkt an Jolt.
- Der native Mesh-Pfad erzwingt statische Bodies und übernimmt Materialparameter; Zero-Time-Schritte werden nicht an `PhysicsSystem::Update` weitergereicht.
- Die Add-on-Einstellungen enthalten einen optionalen Bridge-Pfad; Windows-x64- und Linux-x64-Buildskripte sowie ein offline nutzbarer CMake-Pfad sind enthalten.
- Der Compound Runtime Guard berücksichtigt nun sowohl alte Primitive-Compounds als auch `Compound Convex`.
- Die Regression wurde auf 22 Tests erweitert. Alle 22 Tests bestehen im gebündelten Culverin-Fallback, einschließlich `SimulationScene`-Roundtrip und stabiler Body-/Collider-Identität.
- Signaturschema und Add-on-Runtime-Kennung wurden für den neuen Datenvertrag erhöht. Alte Simulationscaches müssen neu gebacken werden.

## Version 0.5.1

- Behebt den Blender-Registrierungsfehler von `KA_RIGID_BodySettings.collision_shape`.
- Die Kollisionsform verwendet wieder eine statische `EnumProperty`; Blender erlaubt bei dynamischen Enum-Callbacks keinen String-Standardwert.
- Explizite, stabile numerische Enum-IDs bewahren Collider-Einstellungen aus älteren `.blend`-Dateien.
- Ungültige Kombinationen wie dynamisches Triangle Mesh oder Plane werden weiterhin automatisch auf Convex Hull korrigiert.

## Version 0.5.0

- Neue Collider-Option `Compound Convex` für dynamische, kinematische und statische Bodies.
- Das ausgewertete Mesh oder ein separates Low-Poly-`Collision Proxy` wird mit gebündeltem CoACD 1.0.11 in mehrere echte konvexe Teil-Hulls zerlegt.
- Qualitätsstufen: `Fast` (bis 4 Teile), `Balanced` (bis 8) und `Accurate` (bis 16), zusätzlich vollständig einstellbares `Custom`.
- Die Zerlegung und alle Teil-Hulls werden im persistenten Collider-Cache gespeichert; nur der erste kalte Bake muss CoACD ausführen.
- Culverin 0.13.2 kann konvexe Teil-Hulls noch nicht als einen nativen Jolt-Compound anlegen. Deshalb erzeugt 0.5.0 pro logischem Body mehrere konvexe Jolt-Bodies und verbindet sie mit Fixed Constraints. Playback und Cache bleiben auf einen logischen Blender-Body reduziert.
- `Convex Hull` bleibt der schnelle Standard. `Compound Convex` ist für Körper gedacht, bei denen eine einzelne konvexe Hülle sichtbare Abstände oder überbrückte Einbuchtungen erzeugt.
- Die UI enthält Schalter, um alle ausgewählten KA-Bodies gemeinsam auf `Convex Hull` oder `Compound Convex` zu stellen.
- Der Collider-Cache wurde auf `ka_rigid_colliders_v4.kahc` erweitert und speichert jetzt Single-Hulls und CoACD-Zerlegungen.
- Die Regression umfasst 20 Tests; neu ist ein dynamischer Compound-Convex-Cluster aus zwei konvexen Teilkörpern.

## Neu in Version 0.4.8

- Convex-Hulls verwenden eine support-fehlergesteuerte Punktauswahl statt reinem Farthest-Point-Sampling.
- Die Formtoleranz kombiniert einen absoluten Wert mit einem relativen Anteil der Collider-Diagonale.
- Bei verfehltem Primärbudget wird kontrolliert bis zum `Rescue Vertex Limit` erhöht; erst danach erfolgt der vollständige Convex-Hull-Fallback.
- Jedes Rigid-Body-Objekt kann ein separates Low-Poly-`Collision Proxy`-Mesh verwenden.

# KA Rigid Dynamics 0.6.0

Eigenständige Rigid-Body-Pipeline für Blender ohne Blenders Rigid Body World. Version 0.6.0 ergänzt den solverneutralen `SimulationScene-v1`-Vertrag und den optionalen nativen Jolt-5.6-Bridge.

## Neu in Version 0.4.7

- Ein 64-Punkt-Proxy, der die Formtoleranz verfehlt, wird nicht mehr trotzdem verwendet.
- Der betroffene Body erhält automatisch den vollständigen Convex-Hull; bereits ausreichend genaue kleine Proxys bleiben unverändert.
- Diagnosen unterscheiden `precision_rescue` von einem normalen, toleranzgerechten Proxy.
- Alte 0.4.6-Simulationscaches werden ignoriert und müssen neu gebacken werden.
- Neue Regression **High-detail precision hull** prüft die native Simulation eines 384-Punkt-Hulls.

## Stabilität aus Version 0.4.6

- `KA_Physics_Ground` kann durch **Add Selected Bodies** nicht versehentlich zu Dynamic werden.
- Preflight, Payload und Jolt-Adapter halten ihn dauerhaft auf `STATIC + PLANE`.

## Neu in Version 0.4.5

- `Native Jolt` bleibt der Produktionsstandard für Island Sleeping.
- Transform-, Geschwindigkeits-, Aktivitäts- und Energiedaten werden pro Frame gemeinsam aus Culverins Shadow Buffers gelesen.
- Der nächste adaptive Substep-Wert wird aus dieser bereits vorhandenen Zustandsprobe bestimmt.
- Die Cache-Datei `ka_rigid_cache.karc` verwendet Schema 3 und erhält direkt voraufbereitete Float32-Transformblöcke.
- Der persistente Collider-Cache heißt `ka_rigid_hulls_v3.kahc`. Hull-Punkte werden als Float64 gespeichert, damit kalte und warme Bakes dieselbe Geometrie verwenden.
- Die automatische Worker-Heuristik verwendet 2 Threads bis 32 Bodies, 4 bis 750, 6 bis 3.000, 8 bis 10.000 und darüber höchstens 12, begrenzt durch die verfügbaren CPU-Threads.
- Bake-Diagnosen protokollieren `bulk_frame_sample_seconds`, `final_motion_energy_proxy` und die tatsächlich direkt erzeugte Zahl binärer Transformwerte.
- Die Qualitäts-Suite prüft zusätzlich den direkten Cachepfad und die konservative Thread-Heuristik.

## Stabilität aus Version 0.4.4

- Native Jolt island sleeping ist der Produktionsstandard.
- Hybrid/Custom-Deaktivierung wird gebündelt und erst nach Jolt-Bestätigung als schlafend gezählt.
- Früher Abbruch verwendet ausschließlich bestätigte aktive Indizes.
- Adaptive Substep-Minimum-/Maximumwerte entsprechen den tatsächlich ausgeführten Schritten.
- Extreme Solver-Massenverhältnisse können konditioniert werden, ohne die Quellmasse zu verändern.
- Detaillierte Kontakte bleiben standardmäßig deaktiviert und werden bei Bedarf einmal pro gerendertem Frame gelesen.

## Neu in Version 0.4.2

- Persistenter Hull-Cache über Blender-Neustarts hinweg.
- Native Culverin-Shadow-Buffer für Transformations- und Geschwindigkeitsdaten.
- Adaptive Substeps, frühes Bake-Ende und getrennte Ausführungsmodi.

## Neu in Version 0.4.1

- `Single Hull` ist wieder der Standard. `Auto Compound` und `Always Compound` sind ausdrücklich experimentell.
- Compound-Boxen werden nicht mehr nur über belegte Voxel bewertet. Zusätzlich werden geschätztes Außenvolumen, maximale Oberflächenabweichung und die Verbesserung gegenüber dem Single Hull geprüft.
- Abgelehnte Proxies protokollieren alle Fallback-Gründe sowie Messwerte für Innenabdeckung, Außenvolumen und Oberflächenabweichung.
- Die Side-Stick-Erkennung verwendet jetzt ausschließlich zusammenhängende Niedriggeschwindigkeitsphasen. Ein einzelner kurzzeitig niedriger Gleitwert erzeugt keinen Kandidaten mehr.
- Der optionale Runtime Guard erkennt verifizierte Compound-zu-Compound-Side-Sticks, stellt die betroffenen Bodies auf Single Hull zurück und führt den Bake einmal neu aus.
- Cache- und Signatur-Schema wurden für die neuen Collider-Regeln aktualisiert.

## Neu in Version 0.4.0

- `Dynamic Collider`: `Single Hull`, `Auto Compound` oder `Always Compound`.
- `Auto Compound` wird nur aktiv, wenn der einzelne Convex Hull die eingestellte Fehlergrenze überschreitet.
- Culverin 0.13.2 akzeptiert in Compound Bodies keine Convex-Hull-Teilformen zuverlässig. Deshalb verwendet 0.4.0 eine deterministische, innenliegende Voxel-Box-Zerlegung.
- Pro Körper sind Auflösung, maximale Teilzahl, Inset und Mindestabdeckung einstellbar.
- Zerlegungen mit weniger als zwei Teilen oder zu viel leerem Box-Volumen werden verworfen; der Körper fällt sicher auf den bisherigen Single Hull zurück.
- Compound-Proxies werden zusammen mit der ausgewerteten Mesh-Geometrie gecacht.
- Payload und Logs enthalten Part-Anzahl, Voxel-Abdeckung, Zerlegungszeit und Fallback-Grund.
- `Side-Stick Diagnostics` wertet bei aktivierten Detailkontakten Kontaktdauer, Normalenrichtung, Kontaktpunkt und Gleitgeschwindigkeit aus.
- Verdächtige lang anhaltende Seitenkontakte werden als `side_stick_candidates` im Bake-Ergebnis und Log gespeichert.

## Neu in Version 0.3.8

- `Create Ground` verwendet einen vorhandenen KA-Boden wieder.
- Preflight erkennt vollständig überlagerte statische Collider.
- Duplikate können ausgeschlossen, gelöscht oder nur gemeldet werden.

## Neu in Version 0.3.7

- `Balanced / Stable` reduziert dynamische Convex Hulls nicht mehr unter 64 Punkte.
- Statische Körper werden deterministisch vor kinematischen und dynamischen Körpern angelegt.
- Bestehende 0.3.6-Szenen werden auf die stabilen Collider-Standards migriert.
- `Fast` bleibt als ausdrücklich weniger stabile Performance-Option verfügbar.

## Neu in Version 0.3.6

- `Deterministic Mode` sortiert alle Bodies stabil und verwendet einen Jolt-Worker-Thread.
- Wiederholte Bakes derselben Scene-Signatur werden automatisch mit dem vorhandenen Cache verglichen.
- Der Cache speichert einen stabilen Ergebnis-Digest sowie maximale Transformabweichung und Vergleichspfad.
- Add-on-Version, Signatur-Schema, Cache-Version und Culverin-Version sind Bestandteil der Scene-Signatur.
- Beim Laden eines Caches mit anderer oder fehlender Runtime-Metadaten wird ein Rebake empfohlen.
- Convex Hulls werden nicht mehr ausschließlich über eine fixe Punktzahl begrenzt. Die adaptive Auswahl erhöht das Punktbudget, bis die gemessene richtungsabhängige Formabweichung unter dem Zielwert liegt.
- Collider-Presets: `Fast` (5 mm / 48 Punkte), `Balanced` (2 mm / 64 Punkte), `Accurate` (0,5 mm / 128 Punkte) und `Custom`.
- Pro Collider werden maximale und RMS-Formabweichung, Rohpunktzahl, gewählte Punktzahl und Zielerfüllung protokolliert.
- `Run Quality Tests` startet sechs isolierte Jolt-Tests, ohne die offene Blender-Szene zu verändern:
  - Fall und Settling
  - Restitutionssprung
  - Stapelstabilität
  - Reibungsvergleich
  - CCD gegen eine dünne Wand
  - identischer Doppel-Bake zur Determinismusprüfung
- Der Testbericht wird als `ka_rigid_regression.json` im Scene-Cache-Verzeichnis gespeichert und mit dem vorherigen Bericht verglichen.



## Neu in Version 0.3.5

- Evaluierte Mesh-Geometrie, Volumen, Bounds und Convex-Hull-Proxies werden im Arbeitsspeicher gecacht.
- Der Cache invalidiert automatisch bei Änderungen an Geometrie, Topologie, Modifier-Ergebnis oder Skalierung.
- Wiederholte Bakes mit unveränderten Fragmenten verwenden die vorhandenen Hull-Proxies.
- Vertex- und Triangle-Daten werden mit `foreach_get` gebündelt aus Blender gelesen.
- Preflight und Payload-Erstellung teilen sich dieselben Geometrieanalysen.
- Bake-Logs trennen Payload-, Backend- und Cache-Schreibzeit.
- Vollständige Pro-Körper-Payload-Logs sind separat und standardmäßig deaktiviert.
- Geschwindigkeitsstatistiken berücksichtigen nur aktive dynamische Körper.
- Der Collider-Cache kann im Panel manuell geleert werden.

## Neu in Version 0.3.4

- verpflichtender Bake-Preflight vor Aufbau der nativen Jolt-Welt
- automatische Korrektur von Mesh-Collidern auf dynamischen oder kinematischen Körpern zu Convex Hull
- zusätzlicher Operator `Fix Invalid Colliders` für eine manuelle Vorabkorrektur
- Mesh-Collider werden in der Objekt-UI nur noch für statische Körper akzeptiert
- native Jolt-Sleeping-Inseln sind der neue Standard; die bisherigen expliziten Schwellen bleiben als optionaler Custom-Modus verfügbar
- detaillierte Kontakt-Einzelereignisse sind standardmäßig deaktiviert und müssen ausdrücklich eingeschaltet werden
- stabilisierbare Mindestmasse und Mindestgröße für sehr kleine Fragmente
- optionale Modi: unverändert simulieren, stabilisieren oder aus der Solver-Payload ausschließen
- Warnung bei extremen Masseverhältnissen
- adaptive CCD-Auswahl anhand von Körperradius und initialer Geschwindigkeit
- deterministische Convex-Hull-Vereinfachung mit standardmäßig maximal 64 Support-Punkten
- Payload und Logs dokumentieren Rohmasse, effektive Masse, CCD-Entscheidung, Hull-Reduktion und übersprungene Körper

### Fehlerkorrektur für dynamische Mesh-Collider

Dynamische und kinematische Triangle-Mesh-Collider werden nicht mehr bis zum nativen Backend weitergereicht. Bei aktiviertem `Auto-Fix Invalid Colliders` werden sie vor dem Bake auf `Convex Hull` umgestellt. Ist Auto-Fix deaktiviert, wird der Bake mit einer verständlichen Preflight-Meldung blockiert.

## Korrektur in Version 0.3.3

- reparierte Cache-Wiedergabe nach Add-on-Updates
- veraltete Blender-Handler werden automatisch entfernt und neu registriert
- Cache Playback wird nach erfolgreichem Bake automatisch aktiviert
- der aktuelle Cache-Frame wird nach dem Bake direkt angewendet
- explizite Viewport-Redraws und Playback-Logs pro Frame

- gebündeltes natives Jolt-Runtime über Culverin 0.13.2
- lauffähig mit Blender/CPython 3.13 unter Windows x64 und Linux x64
- echte Convex-Hull-Kollisionskörper für dynamische Meshes
- statische Dreiecksmeshes
- von Jolt berechnete Massenträgheit und Rotationsdynamik
- Continuous Collision Detection pro Körper
- native Multi-Core-Simulation
- native Kontaktmanifolds und Broadphase
- Sleeping sowie einstellbare Ruhe-Schwellen
- Korrektur zwischen Jolt-Schwerpunkt und Blender-Objektursprung
- dichtebasierte Masse für KA-Fracture-Fragmente
- automatische Fracture-Erkennung auch beim normalen `Dynamic`-Button
- erweiterte Logs für Kontakte, Impulse, problematische Paare, Körpergeschwindigkeiten und Sleeping

## Installation

1. `KA-Rigid-Dynamics-v0.6.5-extension.zip` in Blender über `Edit > Preferences > Get Extensions > Install from Disk` installieren.
2. Add-on aktivieren.
3. Im 3D-Viewport das N-Panel `KA Physics` öffnen.
4. Als Backend `Jolt` auswählen.

Die klassische Add-on-ZIP ist nur für Installationen gedacht, die noch den älteren Add-on-Installer verwenden.

## Erster Test

1. `Create Ground Plane` drücken.
2. Ein Mesh über der Bodenfläche platzieren und auswählen.
3. `Dynamic` drücken.
4. `Collision Shape` auf `Convex Hull` setzen.
5. Optional `Mass Source > Density` verwenden.
6. Start- und Endframe festlegen.
7. `Bake KA Physics` drücken.
8. Timeline abspielen.

## KA Fracture

`Import KA Fracture Pieces` sucht nach diesen Tags:

- `ka_fracture_final_piece`
- `ka_fracture_break_piece`
- `ka_fracture_prepared_piece`

Zusätzlich werden Namen nach dem Muster `KA_Fracture_Piece_*` erkannt. Fracture-Teile erhalten automatisch:

- Dynamic
- Convex Hull
- dichtebasierte Masse
- die im Panel eingestellte Fracture Density
- CCD

Für aufgeraute oder stark subdividierte Fragmente kann unter `Selected Body > Collision Proxy` ein glattes Low-Poly-Mesh zugewiesen werden. Der Proxy darf eine eigene Position und Rotation besitzen; beim Payload-Aufbau wird er in den lokalen Kollisionsraum des Fragments transformiert.

## Collision Shapes

- `Sphere`: native Kugel
- `Box`: native orientierte Box
- `Plane`: unendliche statische Ebene in lokaler XY-Ausrichtung
- `Convex Hull`: native konvexe Hülle; dient als Ausgangsform für Single Hull und automatische Compound-Auswahl
- `Auto Compound`: Laufzeit-Proxy aus mehreren Jolt-Box-Subshapes; keine separat auswählbare Objektform
- `Mesh`: statisches Dreiecksmesh; nicht für dynamische Körper

## Solver-Einstellungen

- Jeder normale Jolt-Bake schreibt den direkten binären Float32-Transformcache; ein Profil muss nicht gewählt werden.
- `Detailed Contact Diagnostics`: liest und aggregiert Kontakt-Events, ohne Payload-Diagnosen oder Python-Frame-Dictionaries zu aktivieren.
- `Detailed Payload Diagnostics`: schreibt ausführliche Body-Payloads und erfasst Body-Geschwindigkeits-Peaks, ohne Kontakte auszuwerten.
- `Log Ausgaben`: schreibt allgemeine Bake-/Cache-Ereignisse und nur die ausdrücklich aktivierten Detailinformationen ins Log.
- `Substeps`: Anzahl nativer Jolt-Schritte pro Blender-Frame
- `Jolt Threads`: `0` wählt automatisch eine passende Worker-Anzahl
- `Penetration Slop`: zulässige Kontakttoleranz
- `Deterministic Mode`: erzwingt stabile Body-Reihenfolge und einen Jolt-Worker-Thread
- `Collider Quality`: legt absolute/relative Formtoleranz, Primärbudget und kontrolliertes Rescue-Budget fest
- `Sleeping`: erlaubt oder unterbindet das Ruhen dynamischer Körper
- `Sleeping Mode > Native Jolt`: verwendet ausschließlich Jolt-Island-Sleeping
- `Sleeping Mode > Hybrid Experimental`: optionaler, bestätigter Low-Motion-Settle-Pass für spezielle Fracture-Fälle
- `Sleeping Mode > Custom Thresholds`: verwendet die expliziten linearen/angularen Add-on-Schwellen
- `Sleep Time`: Mindestdauer unter den Schwellen, bevor Hybrid/Custom deaktiviert

Die internen Jolt-Iterationszahlen werden von Culverin 0.13.2 nicht separat bereitgestellt. Das Feld `Solver Iterations` gilt daher nur für das Reference-Backend.

## Cache

Bei gespeicherten Blend-Dateien liegt der Standardcache unter:

`//ka_rigid_cache/<Scene>/ka_rigid_cache.json.gz`

Bei ungespeicherten Dateien wird das temporäre Betriebssystemverzeichnis verwendet. Der Cache bleibt außerhalb der Blend-Datei.

## Log Ausgaben

Die Checkbox `Log Ausgaben` befindet sich ganz unten im Hauptpanel. Die Logdatei liegt standardmäßig unter:

`//ka_rigid_cache/<Scene>/ka_rigid_dynamics.log`

Pro Jolt-Bake werden unter anderem protokolliert:

- verwendete native Runtime und Thread-Anzahl
- Kollisionsform, Masse, Dichte, Schwerpunkt und Geometrieumfang jedes Körpers
- aktive und schlafende Körper
- maximale lineare und angulare Geschwindigkeit samt Körper und Frame
- Contact Added, Persisted und Removed nur bei aktiviertem `Detailed Contact Diagnostics`
- stärkster Kontaktimpuls samt Objektpaar und Frame nur in diesem Diagnosemodus
- stärkste Kontaktpartner pro Körper nur in diesem Diagnosemodus
- Cache-Pfad, Dateigröße und Laufzeit
- vollständige Fehler-Tracebacks

Culverin 0.13.2 stellt die numerische Penetrationstiefe eines Jolt-Kontakts nicht bereit. Deshalb wird im optionalen Kontakt-Diagnosemodus der stärkste Kontaktimpuls statt einer angeblich exakten Penetrationstiefe protokolliert. Die Einzelereignisdiagnostik ist wegen ihres Python-Aufwands standardmäßig aus.

## Reference-Backend

Das Reference-Backend bleibt als einfacher, Blender-unabhängiger Pipeline-Test erhalten. Es verwendet vereinfachte Kollisionskörper und ist nicht für finale Simulationen gedacht.

## Bekannte Einschränkungen

- Kinematic-Animationen werden noch nicht frameweise aus Blender gesampelt.
- Convex-Hull-Subshapes in Compound Bodies sind mit Culverin 0.13.2 nicht verfügbar; 0.4.0 verwendet deshalb Box-Subshapes.
- Gelenke, Constraints und Fracture-Bonds fehlen noch.
- Das PhysX-Backend ist weiterhin nur vorbereitet.
- Für statische Dreiecksmeshes stellt Culverin 0.13.2 keine individuellen Reibungs- und Restitutionsparameter bereit.
- macOS enthält derzeit kein gebündeltes Jolt-Binary; dort bleibt das Reference-Backend verfügbar.

## Drittanbieter

Das Add-on bündelt Culverin 0.13.2 und Jolt Physics. Die jeweiligen MIT-Lizenzen liegen in `THIRD_PARTY_LICENSES/`.


## Backend-Sicherheitskorrektur 0.3.1

- Alte Blend-Dateien, die noch `REFERENCE` gespeichert hatten, werden einmalig auf `JOLT` migriert, sofern die gebündelte Runtime verfügbar ist.
- Convex-Hull- und Mesh-Szenen dürfen nicht mehr still mit dem vereinfachten Reference-Solver gebacken werden.
- Beim Import von KA-Fracture-Teilen und beim Bake wird Jolt automatisch gewählt.
- Der Reference-Solver bleibt nur für einfache Sphere/Box/Plane-Pipeline-Tests verfügbar.


## 0.3.3 installation fix

Add-on registration no longer accesses `bpy.data.scenes` while Blender exposes `_RestrictData`. Scene migration and handler logging are deferred until Blender releases the data API.

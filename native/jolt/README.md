# Optionales direktes Jolt-Bridge-Scaffold

Version 0.4.9 verwendet dieses Scaffold noch nicht. Das aktive Backend lädt die gebündelte Culverin-0.13.2-Runtime für CPython 3.13 unter Windows und Linux.

Eine Aktualisierung auf Jolt Physics 5.6 kann nicht durch Python-Dateien ersetzt werden: Dafür müssen Culverin beziehungsweise eine direkte Bridge gegen die gewünschte Jolt-Version für beide Zielplattformen neu kompiliert und getestet werden. Das öffentliche Culverin-0.13.2-Interface stellt derzeit keine Velocity-/Position-Iterationszahlen, nativen Sleep-Schwellen oder vollständigen `PhysicsSettings` bereit.

Der Ordner bleibt als Grundlage für diese spätere C/C++-Integration erhalten.

# Legacy Controller Origin

This directory is a frozen runtime copy of `src/visual_navigation` from:

- workspace: `/home/hjh/cup_corporate/Public-Safety-Event-`
- Git commit: `71ddfde70ba7a449e6087fd524d71add4f674aab`
- imported: 2026-10-04

Only packaging changed: CMake builds these sources as
`waypoint_navigator_legacy`. The C++ files and headers remain byte-for-byte
identical to the source workspace so the comparison launch runs the old
in-place-turn controller itself, not a reimplementation.

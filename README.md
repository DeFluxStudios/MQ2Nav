MQ2Nav for MQ2 Legacy (discontinued)
======

Deprecation Notice
---
_Notice: this branch contains the original version of MQ2Nav for MacroQuest 2. This branch is no longer maintained.
For the latest version please use the master branch version which is designed and build for MacroQuest "Next"._

_MacroQuest "Next" is free and open source (unlike MacroQuest 2). Go [here](https://gitlab.com/macroquest/next/mqnext) for more information._


About
---

MQ2Nav is a NavMesh based navigation and pathfinding plugin for [MacroQuest2](http://www.macroquest2.com). Navigation is based on the RecastNavigation library. The plugin also includes an ingame overlay to display navigation meshes as well as a debugging UI provided by [imgui](https://github.com/ocornut/imgui/).

This project consists of two parts: The MQ2 Plugin (MQ2Nav.dll), and the Mesh Generator (MeshGenerator.exe)

The MeshGenerator tool is used for generating and editing the properties of navmeshes

Screenshots
-----------

* See the [wiki](https://github.com/brainiac/MQ2Nav/wiki/Screenshots) for some screenshots and features


Building
--------

##### Requirements

* Visual Studio 2022 with the Desktop development with C++ and CMake tools workloads/components
* MSVC v142 and v143 toolsets, plus a Windows 10 SDK

MQ2Nav is a unique plugin. You do not want to add it to your solution. Instead you will build it in a separate solution apart from MacroQuest2.sln.

1. Check out the sources into <your macroquest source folder>/MQ2Nav. MQ2Nav should be next to MQ2Main, etc.
2. Open MacroQuest2.sln and build
3. Open MQ2Nav\MQ2Nav.sln and select the configuration (most people will want Release)
4. **build MQ2Nav.dll:** select the Win32 architecture and build
5. **build MeshGenerator.exe:** select the x64 architecture and build

The first time you build, you'll download some NuGet packages. The bundled Protocol Buffers C++ runtime is pinned to upstream v21.12 (C++ 3.21.12); the `libprotobuf` project configures and builds it with CMake and produces a matching `protoc.exe` for the navmesh schema.

The outputs will go into your Release (or Debug, depending on your selected configuration) folder.

### Third Party Libraries

This plugin makes use of the following libraries:

* [EQEmu zone-utilities](https://github.com/EQEmu/zone-utilities)
* [imgui](https://github.com/ocornut/imgui)
* [google protocol buffers](https://github.com/google/protobuf)
* [glm](http://glm.g-truc.net)
* [rapidjson](http://rapidjson.org)
* [RecastNavigation](https://github.com/recastnavigation/recastnavigation)
* [SDL2](http://libsdl.org/)
* [zlib](http://zlib.net/)


Contributing
------------

Contributions are welcome! Feel free to open pull requests or report issues.

**TODO**

Usage
-----

See the [wiki](https://github.com/brainiac/MQ2Nav/wiki) for more info

### Headless mesh generation

MeshGenerator has headless commands for building, checking and comparing meshes. Each command prints one JSON object on stdout; progress and diagnostics go to stderr. Nothing is ever written over an existing file, and a build never installs a mesh into a live MacroQuest folder. Run `MeshGenerator.exe --help` for the full option list.

```powershell
.\MeshGenerator.exe build --zone lfaydark --eq-path "C:\EverQuest" --stage-root "C:\NavMesh-Staging"
.\MeshGenerator.exe validate --zone lfaydark --mesh "C:\NavMesh-Staging\MQ2Nav\lfaydark.navmesh"
```

| Command | What it does |
|---|---|
| `build` | Builds a zone into `<stage-root>\MQ2Nav\<zone>.navmesh` and validates it with MQ2Nav's own loader. |
| `validate` | Loads a mesh and reports tiles, polygons, walkable area, links and volumes. |
| `export` | Writes a mesh's off-mesh links, area volumes, area types and settings to one JSON file. |
| `diff` | Compares two meshes: lost, added and changed links, volumes and area types, settings, tile and polygon counts, and which tiles' geometry changed. |
| `path` | Plans a route the way the plugin does (same filter, search box, node pool growth and straight path flags). |
| `reach` | Floods the mesh from a point and lists the places that can't be reached, with a sample `/loc` for each. |
| `inspect` | Reports the polygon, area, flags and tile at a point, and the links and volumes near it. |
| `probe` | Checks whether points can be reached from the first one, or runs a route suite from a file. |
| `config` | Shows what the GUI's `MeshGenerator.ini` resolves to. |

#### Build options

- **Settings.** `--preset mq2nav|triune` or `--settings-from MESH`, then any single setting on top (`--tile-size`, `--cell-size`, `--cell-height`, `--agent-height`, `--agent-radius`, `--max-climb`, `--max-slope`, `--region-min-size`, `--region-merge-size`, `--edge-max-len`, `--edge-max-error`, `--verts-per-poly`, `--detail-sample-dist`, `--detail-sample-max-error`). Limits follow the GUI sliders. MQ2Nav's defaults apply when nothing is given.
- **Keeping fixes.** `--carry-from MESH` copies another mesh's links, volumes and area types into the build. `--fixes FILE` reads what `export` writes. `--connections FILE` and `--volumes FILE` take JSON arrays.
- **Region rebuild.** `--base MESH` loads an existing mesh and rebuilds only the tiles touched by `--box "Y1,X1:Y2,X2"` (a `/loc` box; `--tiles` is the older name) and by any new links or volumes. Settings come from the base. Grid cells with no tile in the base are refused, with a list, because building them would add mesh nobody reviewed.
- **Walkways.** With `--walkways`, volumes whose name starts with `walkway` keep their spans through the ledge filter and the agent-radius erosion, so narrow logs, planks and ledges stay walkable. The headroom filter still runs.
- **Drop links.** `--auto-drops FILE` writes one-way drop link candidates to a review file (open edge, no floor just past it, ground below within `--max-drop`, clear lines outward and down). They are never added; feed the reviewed file back with `--connections`.
- **Other.** `--threads N` caps the build threads. `--mesh-version 4|5` picks the file version; carried and region builds keep the input mesh's version. `--mesh-root` points to the MacroQuest resources root used to find `MQ2Nav\<zone>_doors.json` and defaults to `--eq-path`. `--no-max-extents` skips MQ2Nav's built-in zone bounds.
- **Live folder guard.** A stage root inside a MacroQuest install is refused unless `--allow-live` is given.

The build result also lists links whose ends are no longer on the mesh, and records each input file with its SHA-256.

#### Coordinates and files

Points on the command line are `/loc` order: `Y,X,Z`. JSON files use mesh axes, `[x,y,z]` with `y` up, exactly as `export` writes them. Output keys ending in `Loc`, and `loc`, `from`, `to` and `at`, are `/loc` order.

```json
{"name": "ledge", "start": [0, -63.8, 0], "end": [25, -63.4, 0], "bidirectional": false, "area": 1}
{"name": "walkway plank", "verts": [[-10, -80, -10], [10, -80, -10], [10, -80, 10]], "hmin": -80, "hmax": -40, "area": 1}
```

Area ids 62 and 63 are reserved. Volumes may use area 0 (not walkable).

A routes file for `probe --routes` is a JSON array:

```json
[
  {"name": "dock to mine", "points": "0,0,-63.8;0,25,-63.8"},
  {"name": "no shortcut over the wall", "points": [[0, 0, -63.8], [120, 40, -60]], "expect": "noreach"},
  {"name": "not a long detour", "points": "0,0,-63.8;20,0,-63.8", "maxLength": 60},
  {"name": "ends on the right floor", "points": "0,0,-63.8;0,25,-63.8", "endZ": -63.4, "endTolerance": 3}
]
```

Each route also reports where it really ends (`endLoc`) and the gap to the target. A route fails when that end is more than twice `endTolerance` (default 5) away across, or, when `endZ` is given, more than `endTolerance` away in height. Take `endZ` from where a character really stops in game: the search box can snap a target onto another floor.

#### Rebuilding without losing fixes

```powershell
.\MeshGenerator.exe export --zone everfrost --mesh "C:\MQ\resources\MQ2Nav\everfrost.navmesh" --out "C:\NavFixes\everfrost.json"
.\MeshGenerator.exe build --zone everfrost --eq-path "C:\EverQuest" --stage-root "C:\NavMesh-Staging" `
    --settings-from "C:\MQ\resources\MQ2Nav\everfrost.navmesh" --carry-from "C:\MQ\resources\MQ2Nav\everfrost.navmesh"
.\MeshGenerator.exe diff --zone everfrost --old "C:\MQ\resources\MQ2Nav\everfrost.navmesh" --new "C:\NavMesh-Staging\MQ2Nav\everfrost.navmesh"
.\MeshGenerator.exe probe --zone everfrost --mesh "C:\NavMesh-Staging\MQ2Nav\everfrost.navmesh" --routes "C:\NavFixes\everfrost_routes.json"
```

Copy the staged mesh into place yourself, after a backup, only when `diff` reports no lost fixes and every route passes. Add `--strict` to `diff` to also fail on changed settings or tile geometry. A mesh with hand-built tiles will show changed tiles after a full rebuild even when no fix was lost; change those meshes with `--base` region builds instead.

#### Exit codes

`0` success, `2` invalid arguments, `3` missing input, `4` build or write error, `5` mesh validation error, `6` a check failed (`diff` found lost fixes or, with `--strict`, changes; a `probe` route failed, `path` found no full route, `reach` listed unreachable places), `70` unexpected failure.

The program is a console application so shells capture its output. Started from Explorer, the GUI lets go of the console window Windows gives it.

Notes
-----

Loading into a zone with MQ2Nav loaded will generate a config file with a list of dynamic objects that can be consumed by MeshGenerator to add that extra geometry to the navmesh. These kinds of objects include some POK stones, for example. Doors are filtered out of the list, but there may be some false positives.


TODO List
---------

Some things to do. Eventually move these to issues...

- Split overlay code out into its own plugin
- Better path planning
- Switch to TileCache implementation (for dynamic obstacle support)
- Improved door opening behavior

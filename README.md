# Project_A

ESP32 robotics monorepo with multiple ESP-IDF apps plus simulator tooling.

Current ESP-IDF apps:
- `legacy/micromouse-robot-main`
- `development/localization`

Non-IDF tooling:
- `development/SA_Simulator`

## Repo Structure

```text
Project_A/
  idfapp                     # repo-root multi-app wrapper
  setup_env.sh               # developer environment helper
  development/
    SA_Simulator/            # Python simulator, not an ESP-IDF app
    components/              # shared components for development apps
    localization/            # ESP-IDF app root
  legacy/
    micromouse-robot-main/   # ESP-IDF app root
  tools/
    idfwrap.sh               # shared ESP-IDF wrapper
    esp-idf/                 # optional repo-local ESP-IDF checkout
```

## ESP-IDF Setup

This repo supports two clean setups:

1. Use an existing ESP-IDF installation by exporting `IDF_PATH`
2. Optional: keep a repo-local ESP-IDF checkout at `tools/esp-idf`

If you already have a working ESP-IDF installation, use that first:

```bash
cd /path/to/Project_A
source /path/to/esp-idf/export.sh
./idfapp development/localization build
```

Or, if you prefer to prepare the shell through the repo helper:

```bash
cd /path/to/Project_A
export IDF_PATH=/path/to/esp-idf
source ./setup_env.sh
```

Optional repo-local setup:

```bash
cd /path/to/Project_A
git clone --branch v5.1.6 --recursive https://github.com/espressif/esp-idf.git tools/esp-idf
source ./setup_env.sh --install
```

On later shells:

```bash
cd /path/to/Project_A
source ./setup_env.sh
```

What `setup_env.sh` does:
- prefers your existing `$IDF_PATH` if already exported
- otherwise uses `tools/esp-idf` if present
- sources `export.sh`
- with `--install`, runs `install.sh esp32`

If you only use `./idfapp`, you do not need to source `setup_env.sh` every time. `idfapp` and `tools/idfwrap.sh` source ESP-IDF for you.

## Updating An Existing Clone

If your partner already has a local clone of this repo, recloning is not required.

Update the existing clone with:

```bash
cd /path/to/Project_A
git fetch origin
git pull --rebase origin main
```

If there are local changes first:
- commit them before pulling, or
- stash them before pulling

Example:

```bash
git status
git stash push -u
git pull --rebase origin main
git stash pop
```

## Build From Repo Root

Build the legacy app:

```bash
cd /path/to/Project_A
./idfapp legacy/micromouse-robot-main build
```

Build the localization app:

```bash
cd /path/to/Project_A
./idfapp development/localization build
```

Direct `idf.py` still works if you want it:

```bash
cd /path/to/Project_A
source ./setup_env.sh
cd legacy/micromouse-robot-main
idf.py build
```

```bash
cd /path/to/Project_A
source ./setup_env.sh
cd development/localization
idf.py build
```

## Flash And Monitor

Localization app:

```bash
./idfapp development/localization -p /dev/ttyUSB0 flash
./idfapp development/localization -p /dev/ttyUSB0 monitor
./idfapp development/localization -p /dev/ttyUSB0 flash monitor
```

Legacy app:

```bash
./idfapp legacy/micromouse-robot-main -p /dev/ttyUSB0 flash
./idfapp legacy/micromouse-robot-main -p /dev/ttyUSB0 monitor
./idfapp legacy/micromouse-robot-main -p /dev/ttyUSB0 flash monitor
```

To build without hardware attached, just use `build`.

To inspect the generated flash arguments after a build:

```bash
sed -n '1,120p' development/localization/build/flash_project_args
sed -n '1,120p' legacy/micromouse-robot-main/build/flash_project_args
```

## Adding A Future Development App

For a future app under `development/`, follow the same pattern:

```text
development/
  measurement_prediction/
    CMakeLists.txt
    main/
      CMakeLists.txt
      main.cpp
      sdkconfig.defaults        # recommended
      partitions.csv            # optional
      idf_component.yml         # optional
```

Each app should usually have:
- top-level `CMakeLists.txt`
- `main/`
- `main/CMakeLists.txt`
- `sdkconfig.defaults`

Per-app optional files:
- `sdkconfig`
- `partitions.csv`
- `idf_component.yml`
- `managed_components/` (generated)
- `build/` (generated)

Share reusable code through `development/components/`, not by cross-including from one app’s `main/` into another’s.

Good candidates for `development/components/`:
- shared interfaces
- reusable drivers
- generic runtime helpers
- algorithm libraries used by multiple apps

Keep app-local in each app’s `main/`:
- `app_main()`
- app wiring
- app-specific maps, mocks, and behavior

## VS Code

The repo includes a portable task file at `.vscode/tasks.json`.

Available tasks:
- `ESP-IDF: Build (app path)`
- `ESP-IDF: Fullclean (app path)`
- `ESP-IDF: Flash (app path)`
- `ESP-IDF: Monitor (app path)`
- `ESP-IDF: Flash Monitor (app path)`

These tasks call `./idfapp`, so they work from the repo root without hardcoded user paths.

The ESP-IDF extension UI is optional. Terminal workflows are the source of truth for this repo.

Local-only IDE files such as `.vscode/settings.json` and `.vscode/c_cpp_properties.json` are intentionally ignored and should stay developer-specific.

## What To Commit

Commit:
- `idfapp`
- `setup_env.sh`
- root `README.md`
- app `CMakeLists.txt`
- app `sdkconfig.defaults`
- app `sdkconfig` when you want stable app config checked in
- app `partitions.csv` when app-specific
- app `idf_component.yml`
- app `dependencies.lock`
- shared components under `development/components/`
- `.vscode/tasks.json`

Ignore:
- `**/build/`
- `**/managed_components/`
- local `.vscode` files other than `tasks.json`
- Python virtual environments and caches

## Troubleshooting

Missing ESP-IDF:
- export `IDF_PATH`, or clone ESP-IDF into `tools/esp-idf`
- then run `source ./setup_env.sh`

`./idfapp` says the app path is invalid:
- use a path relative to repo root
- make sure the target directory contains `CMakeLists.txt`

No serial port found:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | tail -n 50
```

Want the repo-root workflow only:
- use `./idfapp <app-path> ...`
- no need to `cd` into each app

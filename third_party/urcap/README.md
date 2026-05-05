# third_party/urcap — Vendored External Control URCap

This directory vendors the **Universal Robots External Control URCap**
binary used by `docker/ursim/Dockerfile` to build the ENFIELD custom
URSim image (Lane 2, S25).

## File

| Field      | Value |
|------------|-------|
| Filename   | `externalcontrol-1.0.5.urcap` |
| Size       | 35287 bytes |
| SHA-256    | `21ee6c9b4d8a64cf47ce2446233627deec0d2999a0440f4261155d79223b4901` |
| Version    | 1.0.5 (released March 2021) |
| License    | Apache License, Version 2.0 — see [`LICENSE-Apache-2.0`](LICENSE-Apache-2.0) |
| Source     | <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap> |
| Copyright  | © 2019 FZI Forschungszentrum Informatik |

See [`NOTICE`](NOTICE) for full attribution and license-compliance notes.

## Why is this binary in the repo?

ENFIELD's open-science commitment (see `docs/open_science_release.md`)
requires fully deterministic image builds. Vendoring the URCap binary
fixes its SHA-256 across every reviewer's environment, so anyone
building `docker/ursim/Dockerfile` produces a bit-identical image
regardless of whether `/opt/ros/humble/share/ur_robot_driver/resources/`
is present on their machine.

The Apache 2.0 license explicitly permits this redistribution
(Sections 4(a) and 4(c)).

## Verify the vendored binary

If you want to verify the vendored binary matches an upstream build:

```bash
# Option 1: from a ROS humble installation with ur_robot_driver
sha256sum /opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap

# Option 2: from upstream GitHub release
curl -sSL \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.urcap \
  | sha256sum
```

Both should produce:
`21ee6c9b4d8a64cf47ce2446233627deec0d2999a0440f4261155d79223b4901`

## Build the URSim image

```bash
bash scripts/build_ursim_image.sh
```

The build script verifies the SHA-256 of this file before proceeding.
Output image tag: `enfield-ursim:5.12-urcap-1.0.5`.

## Why was this needed (technical rationale)

Universal Robots ships URSim with an OSGi bundle directory at
`/ursim/GUI/bundle/` and an entrypoint that copies `*.jar` files from
`/urcaps/` into that directory at startup. Felix (the OSGi runtime)
then auto-loads any jar in `bundle/`.

The `.urcap` file is structurally an OSGi bundle jar — but with a
`.urcap` extension. PolyScope's "install URCap" GUI button does the
rename internally; the headless Docker entrypoint does not. As a
result, S24 (2026-04-27) tested three volume-mount strategies and
all failed silently — `cp /urcaps/*.jar` produced zero matches.

The custom URSim Dockerfile bakes the URCap into `/ursim/GUI/bundle/`
**with the `.jar` extension**, satisfying Felix's scan path at
startup with no GUI interaction required. This is the single insight
that unblocks live URScript execution.

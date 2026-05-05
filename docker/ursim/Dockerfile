# syntax=docker/dockerfile:1.6
#
# ENFIELD custom URSim image — External Control URCap pre-baked.
# Lane 2 (S25): unblocks live URScript execution.
#
# Why this is one COPY instruction:
#   The vendored externalcontrol-1.0.5.urcap is an OSGi bundle jar with
#   a renamed extension. URSim's entrypoint runs:
#       cp -r /urcaps/*.jar /ursim/GUI/bundle/
#   and Felix scans /ursim/GUI/bundle/ for *.jar at startup. S24-04-27
#   tested three volume-mount strategies; all failed because none did
#   the .urcap -> .jar rename. PolyScope's "install URCap" GUI button
#   does this rename internally; the headless path does not. We do the
#   rename via the COPY destination filename, so Felix picks it up at
#   normal startup with no GUI interaction required.
#
# Build context is docker/ursim/. The URCap is staged from
# third_party/urcap/ by scripts/build_ursim_image.sh (and gitignored
# locally — see docker/ursim/.gitignore).
#
# Build:  bash scripts/build_ursim_image.sh
# Output: enfield-ursim:5.12-urcap-1.0.5

FROM universalrobots/ursim_e-series:5.12@sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51

COPY externalcontrol-1.0.5.urcap /ursim/GUI/bundle/externalcontrol-1.0.5.jar

LABEL org.opencontainers.image.title="enfield-ursim" \
      org.opencontainers.image.description="URSim e-Series 5.12 with External Control URCap baked in" \
      org.opencontainers.image.version="5.12-urcap-1.0.5" \
      org.opencontainers.image.licenses="Apache-2.0" \
      org.opencontainers.image.source="https://github.com/dryuemco/industrial_robot_security" \
      enfield.urcap.version="1.0.5" \
      enfield.urcap.sha256="21ee6c9b4d8a64cf47ce2446233627deec0d2999a0440f4261155d79223b4901" \
      enfield.ursim.base.digest="sha256:b7ad69f5bfa45ffab07788480ad43c753595ce35fcbfe4a3f420725f51764d51"

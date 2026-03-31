# Documentation Index

- [COMMANDS.md](COMMANDS.md): CLI command reference
- [DOCKER.md](DOCKER.md): Docker workflows and path mapping
- [IKALIBR_DOCKER.md](IKALIBR_DOCKER.md): iKalibr docker setup and troubleshooting
- [TEMPORAL_ALIGNMENT.md](TEMPORAL_ALIGNMENT.md): temporal-alignment status note
- [SCRIPTS.md](SCRIPTS.md): script/module overview
- [UTILS.md](UTILS.md): shared utilities overview

## iKalibr Config Note

`Docker/iKalibr/config/cam_calib.yaml` and
`Docker/iKalibr/config/tool/cam_calib.yaml` are currently duplicated on purpose
to preserve compatibility with two config-loading paths used by the iKalibr
tooling. Keep them in sync when updating calibration defaults.

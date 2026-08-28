# SafeSpread

This repository's `main` branch is the canonical SafeSpread application and
firmware source. The deployable iPhone app is in `SafeSpreadVIO/`; the rover
firmware is in `auto_vio/`.

The current setup workflow accepts entered rectangle dimensions only. The
retired **Walk corners** / Corner A / Corner B workflow is not part of the
current product and must not be restored from old branches, commits, build
artifacts, or historical documents.

Files under `docs/superpowers/` are historical planning records, not current
requirements. Before deploying, build from the latest `origin/main` and verify
that the setup screen has the fixed full-width Stop control at the top and no
Walk corners option.

# Agent rules

This is our fork of the Avian physics engine (`avian2d` and `avian3d`). It carries fixes for
engine defects we find, on its own `main`, on top of upstream releases. We post nothing
upstream ourselves (no issues, pull requests or comments on the upstream repository), but every
fix is kept ready to be offered upstream as it stands.
`AI_POLICY.md` is upstream's rule for contributions there; it does not govern this fork.

## Licence policy

- The engine is upstream's, dual-licensed MIT or Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`).
  Our changes are under the same dual licence.
- New dependencies must be under permissive licences compatible with that (MIT, Apache-2.0,
  BSD, ISC, Zlib and similar). No copyleft, and nothing without a licence.

## IP policy

- Engine, dependency and upstream names appear as upstream uses them.
- Projects that use this fork are not named in it: not in code, comments, docs or commit
  messages. Describe a defect by its engine-only reproduction.
- Refer to upstream issues and pull requests in plain text (`avianphysics/avian` 794), never as
  `#794`, a link or `owner/repo#n`, so nothing cross-references upstream.

## Patches only

- The fork carries patches, not upkeep. A change touches only what its fix needs: the fix, its
  tests, and what the fix forces to change (examples or expected values whose behaviour it
  changes). No migration guides, changelogs, READMEs, reformatting or other maintenance of
  upstream's repository.
- One fix, one commit, with a message in upstream's terms (no fork issue numbers), so it can be
  offered upstream as it stands. Follow-ups to a fix are folded into its commit before it is
  pushed.
- Each fix has an issue in this fork. Once the fix lands, the issue gets one comment naming its
  commit and its tests: the issue maps the defect to its commit. When the fix or its commit
  changes, edit that comment (and the issue text) instead of adding comments as a changelog.
- The fork's own files are `AGENTS.md` and `CLAUDE.md`, in a commit of their own.
- Upstream changes come in by merging upstream's `main` into ours, resolving conflicts in the
  merge.

## Defects and issues

- Each engine defect gets an issue in this fork stating the problem, the fix, and a
  reproducible sample: a minimal standalone program (`Cargo.toml` against the released crate,
  and `src/main.rs`) with its output before and after the fix, or, where a standalone program
  can't show it, why it is hard to reproduce and the evidence that it is still a defect.
- The fix comes with a test that fails without it.

## Commits

- Author and committer: `Liu <12598936+liubsp@users.noreply.github.com>`. No AI attribution.
- `main` moves forward only: push fast-forward, never force.

## Building and testing

- On Windows, link with `rust-lld`: copy `.cargo/config_ci.toml` to `.cargo/config.toml`
  (git-ignored; upstream's CI does the same). Run one cargo build at a time.
- The checks CI runs (`.github/workflows/rust.yaml`). CI denies all warnings
  (`RUSTFLAGS=--deny warnings`, `RUSTDOCFLAGS=--deny warnings`); locally, `-D warnings` after
  clippy's `--` and `RUSTDOCFLAGS` for `cargo doc` catch the same in our code without rebuilding
  every dependency:
  - `cargo fmt --all -- --check`
  - `cargo clippy --locked --all-targets --features avian2d/test,avian3d/test -- -D warnings`
  - `cargo check --locked --all-targets --no-default-features --features 2d,3d` (minimal features)
  - `cargo doc --locked --workspace --no-deps --document-private-items --keep-going` with
    `RUSTDOCFLAGS=--deny warnings`
  - The tests, per crate:
    - `cargo test -p avian3d --lib --no-default-features --features 3d,f64,parry-f64,enhanced-determinism,parallel,collider-from-mesh,serialize,debug-plugin,xpbd_joints,bevy_scene,bevy_picking,diagnostic_ui`
    - `cargo test -p avian2d --lib --no-default-features --features 2d,f64,parry-f64,enhanced-determinism,parallel,serialize,debug-plugin,xpbd_joints,bevy_scene,bevy_picking,diagnostic_ui`
  - CI also runs the examples' and the doc tests (`--examples`, `--doc`).
- `src/tests/determinism_2d.rs` holds a hash of a 2D scene's motion: it changes whenever 2D
  simulation behaviour changes, and is updated only for an intended change.

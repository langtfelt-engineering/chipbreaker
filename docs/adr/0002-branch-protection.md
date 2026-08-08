# ADR 0002 — Branch protection on `main`

- **Status:** accepted; deferred at first, **enabled** when the repository was
  made public
- **Date:** 2026-08-03, enabled 2026-08-08
- **Governs:** repository settings on `main`

## Decision

Branch protection on `main` **blocks force-pushes and branch deletion, for
everyone including administrators**, and does **not** require pull requests.
Direct pushes to `main` stay allowed.

It does **not** require status checks, and that is a change from what this ADR
originally anticipated. The two are incompatible: GitHub enforces required
checks on direct pushes as well as on merges, so a commit must have passing
checks *before* it can arrive — but the checks run *on* arrival. Requiring them
would have meant requiring pull requests too, which the section below rejects on
its own merits. The force-push and deletion guards were always the rows that
mattered; those are the ones taken.

## Why it was deferred at first: it was not available to buy with effort

Branch protection was asked for early. The GitHub API refuses:

```
GET /repos/spanwerk/chipbreaker/branches/main/protection
403: Upgrade to GitHub Pro or make this repository public to enable this feature.
```

`chipbreaker` was a **private repository in an organisation**, and for those,
branch protection — and rulesets, which are the newer equivalent — are a paid
feature. There is no configuration, no workflow, and no third-party action that
substitutes for it: the enforcement lives in GitHub's own push path, which is
exactly why it is worth having and exactly why it cannot be emulated.

Making the repository public removed the restriction, and the protection was
applied the same day.

This is recorded rather than silently skipped because a reader who finds
the requirement and no protection on the branch should be able to learn
that the gap is known, deliberate, and priced, rather than assume it was
forgotten.

## What guarded `main` while it was deferred

| risk | guarded? | by what |
|---|---|---|
| a commit that fails tests, clippy, or fmt | yes, after the fact | CI on every push to `main`, three platforms |
| output that differs across platforms | yes, after the fact | `cross-platform parity` job |
| output that differs on WASM | yes, after the fact | `wasm parity` job under `wasmtime` |
| a determinism rule broken (`f32`, FMA, `HashMap`, threads) | yes, after the fact | `determinism rules` job |
| a golden hash changed without noticing | yes | goldens are in the tree; a change is a reviewable diff |
| a dependency licence violation | yes, after the fact | `cargo deny` |
| **force-push rewriting history on `main`** | **no** | nothing |
| **merging while checks are still red** | **no** | nothing |

The two unguarded rows were the whole content of this ADR. The first is now
guarded by GitHub. The second is not, and is not going to be — see the decision
above for why requiring checks would have cost direct pushes.

"After the fact" is the important qualifier throughout the guarded rows. CI runs
*on* push, not *before* it, so a bad commit reaches `main` and is then reported.
For a solo project that is an acceptable trade — the feedback arrives in minutes
and the fix is another commit.

## Why the force-push row is the one that matters

This is not hypothetical. Early in the project the entire history was rewritten twice
with `git filter-branch`: once to correct the author identity on twenty-two
commits that GitHub could not attribute to an account, and once to strip a
trailer. The second rewrite also rewrote the local `origin/main` tracking ref,
which then lied about what was actually on the remote, and the subsequent push
had to be forced.

That episode ended well. It ended well because it was deliberate, done once, by
the only person working on the repository. Branch protection is what makes the
*next* one — done in a hurry, or by someone else, or by a tool — impossible
rather than merely unlikely. Nothing currently in the repository would notice a
force-push at all: the goldens would still match, CI would still pass, and the
history would simply be different.

## Why pull requests are not part of the eventual configuration

Recorded here so that it is a decision rather than an omission someone
"corrects" later.

Requiring a pull request would mean every unit needs a branch, a PR, and a merge,
and that `main` could no longer be pushed to directly. On a single-developer
repository that buys nothing: there is no second reviewer for the review
requirement to summon, and the checks that matter run identically on a push to
`main` as on a PR. It would convert a real guarantee — the checks pass — into a
ceremony around it.

The parts worth having are the force-push guard and the deletion guard, and both
are available without any PR requirement. If the project gains a second regular
contributor, this paragraph is the one to revisit; the reasoning above stops
holding the moment there is somebody to review.

## What this does and does not do about pull requests

The repository is public, so anybody may open a pull request. **Nobody outside
the organisation can merge one**, and that is GitHub's permission model rather
than anything configured here: merging needs write access, and write access is
held by the maintainers alone.

`CONTRIBUTING.md` says code contributions are not accepted, and
`.github/pull_request_template.md` says the same thing to somebody who has
already started, before they spend an evening on a patch that cannot be taken.
Neither is a security control; both are courtesy.

Forking is deliberately left **enabled**. Disabling it would prevent pull
requests from existing at all, but the GPL grants the right to copy and modify
regardless of whether GitHub's fork button works, so turning it off would buy
nothing real and would read as hostility to people exercising a right the
licence gives them.

## When to revisit

- A second regular contributor joins, which reopens the pull-request question
  above: the reasoning against requiring reviews stops holding the moment there
  is somebody to review.
- A pre-merge check becomes genuinely wanted, at which point the honest route is
  a merge queue rather than required checks on direct pushes.

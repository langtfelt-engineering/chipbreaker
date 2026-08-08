# ADR 0002 — Branch protection on `main`: deferred, and what guards it meanwhile

- **Status:** Accepted. Deferred, blocked on the repository's GitHub plan.
- **Date:** 2026-08-03
- **Governs:** repository settings; to be acted on whenever the GitHub plan or
  the repository's visibility changes

## Decision

Branch protection on `main` is **not enabled**, and will not be until either the
`spanwerk` organisation moves to a paid GitHub plan or the repository is made
public.

When it is enabled, it will require the CI checks and block force-pushes and
branch deletion, and it will **not** require pull requests. Direct pushes to
`main` stay allowed.

## Why it is deferred: it is not available to buy with effort

Branch protection was asked for early. The GitHub API refuses:

```
GET /repos/spanwerk/chipbreaker/branches/main/protection
403: Upgrade to GitHub Pro or make this repository public to enable this feature.
```

`chipbreaker` is a **private repository in an organisation**, and for those,
branch protection — and rulesets, which are the newer equivalent — are a paid
feature. There is no configuration, no workflow, and no third-party action that
substitutes for it: the enforcement lives in GitHub's own push path, which is
exactly why it is worth having and exactly why it cannot be emulated.

This is recorded rather than silently skipped because a reader who finds
the requirement and no protection on the branch should be able to learn
that the gap is known, deliberate, and priced, rather than assume it was
forgotten.

## What actually guards `main` today

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

The two unguarded rows are the whole content of this ADR.

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

## When to revisit

Any of:

- The organisation moves to a paid GitHub plan for any other reason.
- The repository is made public, which the GPL-3.0-or-later half of the licence
  makes likely eventually.
- A second regular contributor joins, which also reopens the pull-request
  question above.

<!--
  Please read this before spending any more time on the patch.
-->

**Chipbreaker does not accept code contributions at present, and this pull
request cannot be merged.**

The reason is licensing, not a judgement about the change. Chipbreaker is
dual-licensed — GPL-3.0-or-later plus a commercial licence — and the commercial
licence requires clean copyright title to the whole work. That means every
contribution has to arrive under a signed Contributor Licence Agreement, and the
CLA process does not exist yet. A single commit without one would taint the
commercial offering permanently, so the honest answer is no rather than
accepting a patch that could not be used.

We are sorry about the friction, and sorrier if you found this after writing the
code rather than before.

**What is genuinely wanted, and carries no paperwork at all:**

- **An issue describing the bug**, ideally with an input file that reproduces
  it — a `.stl`, `.nc`, `.tdx` or tool library. A reproducer for a wrong answer
  is worth more to this project than a patch.
- **Anything with a security dimension** goes to
  [SECURITY.md](../SECURITY.md) instead, privately. That explicitly includes an
  input that makes the engine report a gouged part as clean: for a verification
  tool, a silently wrong answer is worse than a crash.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the full reasoning.

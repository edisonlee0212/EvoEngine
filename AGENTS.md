# Guidelines

## Before Writing Code

**NEVER GUESS - ALWAYS VERIFY:**

- Check codebase for existing solutions
- Read actual source code/docs to verify APIs, signatures, parameters
- Search for built-in features or libraries that already solve this
- Prefer language/framework built-ins over custom implementations
- When uncertain about behavior, write and run a small test script to verify instead of reasoning about it

**When uncertain:** Ask before implementing if requirements are ambiguous, multiple approaches exist, or trade-offs are significant.

When working on a new branch, create a local `./tasks` directory with these files:

- `todo.md`
- `in-progress.md`
- `done.md`

Use these files to keep a brief, current record of planned work, active work, and completed work for the branch. The `./tasks` directory is intentionally git-ignored and should remain local to the working branch/worktree.

Record the current branch at the top of `tasks/todo.md` as `# Branch: <branch-name>`. Before reusing an existing `./tasks` directory, compare that branch name with `git branch --show-current`. If `tasks/todo.md` names a different branch, remove the local `./tasks` directory and recreate fresh `todo.md`, `in-progress.md`, and `done.md` files for the current branch. If no branch is recorded yet and the tasks clearly belong to the current branch, add the branch line instead of deleting them.

For each non-trivial task, establish brief milestones before implementation and record them in `tasks/todo.md`. Keep milestones outcome-oriented and update `tasks/in-progress.md` and `tasks/done.md` as work moves forward. For very small tasks, one milestone is enough; avoid over-planning when the change is obvious.

## Implementation

**Write minimal, simple code:**

- Minimize lines and complexity. Remove unnecessary variables and combine operations.
- Bias toward REMOVING code, not adding
- No new abstractions for one-time operations
- Minimize comments (only for non-obvious logic)
- Write testable code (prefer pure functions, dependency injection over globals)

**Iterate toward perfection:**

- Refactor and simplify code before finalizing. Aim for improvements to conciseness, readability, maintainability, and best practices with each pass.

**Avoid scope creep:**

- Minimal, targeted changes only
- No error handling for impossible scenarios

## After Implementation

- Grep for usages of any removed or renamed symbols to clean up dead references
- Add unit tests for new logic when a test framework is available
- Build the relevant executable target before manual testing app/editor behavior, and report the exact executable path used for the manual test.
- Install all executable applications after finishing a task, unless the user explicitly says not to, and report the exact install command and result.

## GPT-Codex-5.3-Spark Subagent Workflow

Use `GPT-Codex-5.3-Spark` subagents to preserve the main `GPT-5.5` lane for implementation, architecture, debugging decisions, code review conclusions, commits, and PR work.

Spark subagents should run commands, inspect files, collect logs, narrow failures, and return concise findings. They must not modify source code or repository files, stage changes, commit, push, create pull requests, or make architecture/product decisions. If a Spark subagent finds a code issue, it should report the failure and suspected cause; only the current/main agent should make code changes, then ask the Spark subagent to rerun the relevant verification.

Suitable Spark subagent tasks:

- Test runner: run focused/full tests, retry flaky commands once, and report pass/fail with key logs.
- Build/install runner: run package builds, app builds, install scripts, and verify expected binaries or manifests exist.
- Failure log triage: parse MSBuild, CTest, runtime, or installer logs and identify the first real error, target, file, and likely cause.
- Codebase reconnaissance: search APIs, call sites, ownership boundaries, and related tests; return a compact file/function map.
- Diff review: inspect staged/unstaged changes for regressions, accidental files, missing tests, dead references, and scope creep.
- UI smoke/screenshot collector: launch apps, click basic paths, capture screenshots, and report crashes, blank windows, or obvious layout issues.
- Package/dependency inventory: inspect `.evepackage` manifests, package DLLs, dependency chains, and loaded/available package states.
- Docs consistency check: compare changed behavior against README, scripts, and AGENTS guidance; report stale or missing docs.
- PR readiness audit: check branch, status, staged files, test evidence, dirty submodules, and PR body accuracy without changing git state.

This file provides repository-wide guidance for coding agents working in EvoEngine.

## Commit Workflow

When the user asks you to make a commit:

- Make sure the current changes pass all relevant tests before committing.
- If a full test run is not practical or cannot be completed, run the most relevant subset and clearly report what was and was not verified.
- Always update the README or other documentation when the change makes documentation inaccurate, incomplete, or missing.
- Do not include signs of AI/tool usage in commit summaries, commit messages, pull request titles, or pull request descriptions.
- Make sure perform code format check right before commit.

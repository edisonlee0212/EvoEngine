# AGENTS.md

This file provides repository-wide guidance for coding agents working in EvoEngine.

## Branch Task Tracking

When working on a new branch, create a local `./tasks` directory with these files:

- `todo.md`
- `in-progress.md`
- `done.md`

Use these files to keep a brief, current record of planned work, active work, and completed work for the branch. The `./tasks` directory is intentionally git-ignored and should remain local to the working branch/worktree.

## Commit Workflow

When the user asks you to make a commit:

- Make sure the current changes pass all relevant tests before committing.
- If a full test run is not practical or cannot be completed, run the most relevant subset and clearly report what was and was not verified.
- Always update the README or other documentation when the change makes documentation inaccurate, incomplete, or missing.
- Do not include signs of AI/tool usage in commit summaries, commit messages, pull request titles, or pull request descriptions.

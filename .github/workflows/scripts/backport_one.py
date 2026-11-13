#!/usr/bin/env python3
# cspell: ignore worktrees
"""
Check that a pull request cherry-picks onto one base branch.

The runner cherry-picks only to see whether the merge commit applies. It does not
create the remote branch and it does not push. GitHub refuses GITHUB_TOKEN when
that new branch points at some target commits, and this repository does not ask
Actions to open the backport pull request.

When the cherry-pick is clean, the script comments on the original pull request
with Ubuntu commands. Those commands clone into a temporary directory, cherry-pick,
push the backport branch, and open the pull request with gh.

Usage: backport_one.py <base branch> <head branch>
"""

import os
import re
import shlex
import subprocess
import sys

REPO = os.environ["REPO"]
PR_NUMBER = os.environ["PR_NUMBER"]
PR_TITLE = os.environ["PR_TITLE"]
MERGE_SHA = os.environ["MERGE_SHA"]
PR_AUTHOR = os.environ["PR_AUTHOR"]
COMMENT_PATH = os.path.join(os.environ.get("RUNNER_TEMP", "."), "comment.md")


def run(*args: str) -> str:
    return subprocess.run(args, check=True, text=True, capture_output=True).stdout


def git(*args: str) -> str:
    return run("git", *args)


def gh(*args: str) -> str:
    return run("gh", *args)


def log(message: str) -> None:
    """Print a step to the workflow log before a later error can hide it."""
    print(message, file=sys.stderr, flush=True)


def mention() -> str:
    """Ping the author of the original pull request, who has to finish the backport.

    A bot cannot act on the ping and renders as literal text, so it is left out.
    """
    if not PR_AUTHOR or PR_AUTHOR.endswith("[bot]"):
        return ""
    return f"@{PR_AUTHOR} "


def command_error(error: subprocess.CalledProcessError) -> str:
    """Return the captured output from a failed git or gh command."""
    parts = [part.strip() for part in (error.stdout, error.stderr) if part and part.strip()]
    return "\n".join(parts)


def cherry_pick_options() -> list[str]:
    """Use -m 1 only when the merged commit itself has more than one parent."""
    listing = git("rev-list", "--parents", "-n", "1", MERGE_SHA).split()
    if len(listing) > 2:
        return ["-x", "-s", "-m", "1", MERGE_SHA]
    return ["-x", "-s", MERGE_SHA]


def original_author() -> str:
    """Return the author of the merged commit as "Name <email>", or "" if it is unknown."""
    try:
        return git("log", "-1", "--format=%an <%ae>", MERGE_SHA).strip()
    except subprocess.CalledProcessError:
        return ""


def reset_author_command() -> str:
    """Make the person who runs the commands the author of the backport commit.

    A cherry-pick keeps the author of the merged commit, whose email (set by GitHub on
    squash merge) does not always match any Signed-off-by line, so DCO would fail. The
    original author is kept as a Co-authored-by trailer.
    """
    command = "git commit --amend --no-edit --reset-author"
    author = original_author()
    if author:
        command += " --trailer " + shlex.quote(f"Co-authored-by: {author}")
    return command


def backport_title() -> str:
    return f"{PR_TITLE} (backport #{PR_NUMBER})"


def backport_body() -> str:
    return "\n".join(
        [
            f"Backport {MERGE_SHA} from #{PR_NUMBER}.",
            "",
            f"(cherry picked from commit {MERGE_SHA})",
        ]
    )


def local_commands(base: str, head: str) -> str:
    """Ubuntu commands that clone, cherry-pick, push, and open the pull request.

    The clone goes into a temporary directory, so the commands do not use an
    existing checkout or a global git identity.
    """
    options = cherry_pick_options()
    cherry_pick = " ".join(["git", "cherry-pick", *options])
    lines = [
        "set -euo pipefail",
        'WORKDIR="$(mktemp -d)"',
        'echo "Working directory: ${WORKDIR}"',
        "gh repo clone "
        + shlex.quote(REPO)
        + ' "$WORKDIR" -- --branch '
        + shlex.quote(base)
        + " --single-branch",
        'cd "$WORKDIR"',
        "git fetch origin " + shlex.quote(MERGE_SHA),
        'GH_LOGIN="$(gh api user --jq .login)"',
        'GH_ID="$(gh api user --jq .id)"',
        "GH_NAME=\"$(gh api user --jq '.name // .login')\"",
        'git config user.name "$GH_NAME"',
        'git config user.email "${GH_ID}+${GH_LOGIN}@users.noreply.github.com"',
        "git switch --create " + shlex.quote(head),
        cherry_pick,
        reset_author_command(),
        "git push --set-upstream origin HEAD",
        "gh pr create --repo "
        + shlex.quote(REPO)
        + " --base "
        + shlex.quote(base)
        + " --head "
        + shlex.quote(head)
        + " --title "
        + shlex.quote(backport_title())
        + " --body "
        + shlex.quote(backport_body()),
        'rm -rf "$WORKDIR"',
    ]
    return "\n".join(lines)


def clean_comment(base: str, head: str) -> str:
    return "\n".join(
        [
            f"{mention()}Cherry-pick of `{MERGE_SHA}` onto `{base}` applies cleanly.",
            "",
            "Actions checked the cherry-pick and did not create the remote branch. "
            "Run these commands on Ubuntu. They clone into a temporary directory, "
            f"push `{head}`, and open the backport pull request:",
            "",
            "```bash",
            local_commands(base, head),
            "```",
        ]
    )


def failure_comment(base: str, head: str, error: str) -> str:
    """Manual recovery steps used when the runner cherry-pick does not apply.

    This is the previous failure comment. A conflict has to be resolved in a
    checkout the author already has, so the clean temporary-clone script is not
    a substitute.
    """
    worktree = f".worktrees/backport-{base}"
    return "\n".join(
        [
            f"{mention()}The backport to `{base}` failed:",
            "```",
            error,
            "```",
            "To backport manually, run these commands in your terminal:",
            "```bash",
            "# Fetch latest updates from GitHub",
            "git fetch",
            "# Create a new working tree",
            f"git worktree add {worktree} {base}",
            "# Navigate to the new working tree",
            f"cd {worktree}",
            "# Create a new branch",
            f"git switch --create {head}",
            "# Cherry-pick the merged commit of this pull request and resolve the conflicts",
            f"git cherry-pick -x -s --mainline 1 {MERGE_SHA}",
            "# After `git cherry-pick --continue`, make yourself the author so that DCO passes",
            reset_author_command(),
            "# Push it to GitHub",
            f"git push --set-upstream origin {head}",
            "# Go back to the original working tree",
            "cd ../..",
            "# Delete the working tree",
            f"git worktree remove {worktree}",
            "```",
            f"Then, create a pull request where the `base` branch is `{base}` "
            f"and the `compare`/`head` branch is `{head}`.",
        ]
    )


def contained_comment(base: str) -> str:
    return "\n".join(
        [
            f"{mention()}`{MERGE_SHA}` is already contained in `{base}`.",
            "",
            "There is nothing to cherry-pick.",
        ]
    )


def post_comment(body: str) -> None:
    path = COMMENT_PATH + ".ready.md"
    with open(path, "w", encoding="utf-8") as file:
        file.write(body)
    gh("pr", "comment", PR_NUMBER, "--repo", REPO, "--body-file", path)
    log(f"Commented on #{PR_NUMBER}")


def is_empty_cherry_pick(error: subprocess.CalledProcessError) -> bool:
    text = command_error(error).lower()
    return "now empty" in text or "nothing to commit" in text


def backport(base: str, head: str) -> None:
    """Cherry-pick on the runner. Comment with local commands when it is clean."""
    log(f"Fetching {base}")
    try:
        git("fetch", "--force", "origin", base)
        base_oid = git("rev-parse", "FETCH_HEAD").strip()
        git("fetch", "--force", "origin", MERGE_SHA)
    except subprocess.CalledProcessError as error:
        detail = f"git fetch failed.\n{command_error(error)}"
        _write_comment(failure_comment(base, head, detail))
        raise subprocess.CalledProcessError(
            1, ["git", "fetch"], output="", stderr=detail
        ) from error

    log(f"{base} is {base_oid}. Cherry-picking {MERGE_SHA}")
    git("switch", "--force-create", head, base_oid)

    try:
        git("cherry-pick", *cherry_pick_options())
    except subprocess.CalledProcessError as error:
        subprocess.run(["git", "cherry-pick", "--abort"], check=False)
        if is_empty_cherry_pick(error):
            log(f"{MERGE_SHA} is already contained in {base}")
            post_comment(contained_comment(base))
            return
        detail = command_error(error) or "git cherry-pick failed"
        log(f"Cherry-pick conflicts:\n{detail}")
        raise _failed(base, head, detail) from error

    log(f"Cherry-pick of {MERGE_SHA} onto {base} applies cleanly")
    post_comment(clean_comment(base, head))
    print(f"::notice::Commented on #{PR_NUMBER} with local backport commands for {head}")


class _StepFailure(subprocess.CalledProcessError):
    """Cherry-pick or fetch failure whose comment is already stored."""


def _write_comment(body: str) -> None:
    with open(COMMENT_PATH, "w", encoding="utf-8") as file:
        file.write(body)


def _failed(base: str, head: str, detail: str) -> _StepFailure:
    _write_comment(failure_comment(base, head, detail))
    return _StepFailure(1, ["backport_one.py", base, head], output="", stderr=detail)


def validate_ref(kind: str, value: str) -> str:
    """Reject a branch name that could be read as an option or a path traversal.

    The base branch comes from a pull request label, and anyone with triage
    permission can create a label. The name reaches `git fetch`, `gh repo clone`
    and `gh pr create`, so a leading dash or an unexpected character is refused
    here rather than passed on.
    """
    if not re.fullmatch(r"[A-Za-z0-9._/-]+", value):
        raise ValueError(f"{kind} branch {value!r} contains characters that are not allowed")
    if value.startswith("-") or "/-" in value:
        raise ValueError(f"{kind} branch {value!r} must not start a path segment with '-'")
    if value.startswith("/") or value.endswith("/") or "//" in value or ".." in value:
        raise ValueError(f"{kind} branch {value!r} is not a well-formed branch name")
    return value


def main() -> int:
    try:
        base = validate_ref("base", sys.argv[1])
        head = validate_ref("head", sys.argv[2])
    except ValueError as error:
        print(f"::error::{error}", file=sys.stderr)
        return 1
    try:
        backport(base, head)
    except subprocess.CalledProcessError as error:
        message = command_error(error) or "backport failed"
        print(message, file=sys.stderr)
        if not os.path.exists(COMMENT_PATH):
            with open(COMMENT_PATH, "w", encoding="utf-8") as file:
                file.write(failure_comment(base, head, message))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

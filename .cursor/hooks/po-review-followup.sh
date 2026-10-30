#!/usr/bin/env bash
# After dev agent completes on a ticket branch, auto-submit a PO feature QA prompt.
# User retains final approval — this only starts the review turn.
#
# Ticket branches follow .cursor/rules/ticket-driven-development.mdc:
#   <n>-<slug>                  (canonical, from `gh issue develop <n>`)
#   <prefix>/<n>-<slug>[-suffix] (e.g. cloud agents: cursor/6-po-hook-7f95)
#
# Only fires when the branch's change-set actually differs from the last
# time this hook fired for this branch. Without this, every agent completion on
# a ticket branch re-triggers a review even when the turn was pure Q&A or a
# repeat of the same review with nothing new to look at.

set -euo pipefail

input=$(cat || true)
# Malformed or empty stdin must degrade to "do nothing", not a jq stack trace.
status=$(printf '%s' "$input" | jq -r '.status // empty' 2>/dev/null || true)
loop_count=$(printf '%s' "$input" | jq -r '.loop_count // 0' 2>/dev/null || true)

# Only chain once per agent completion; user decides after PO report.
if [[ "$status" != "completed" || "$loop_count" != "0" ]]; then
  echo '{}'
  exit 0
fi

branch=""
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  # symbolic-ref also resolves an unborn branch (no commits yet); rev-parse does not.
  branch=$(git symbolic-ref --short -q HEAD 2>/dev/null || git rev-parse --abbrev-ref HEAD 2>/dev/null || true)
fi

# Only auto-review on ticket branches: the issue number is the first token
# after the (optional) last "<prefix>/". Captured once here, reused below.
ticket_re='^([A-Za-z0-9._-]+/)?([0-9]+)-'
if [[ ! "$branch" =~ $ticket_re ]]; then
  echo '{}'
  exit 0
fi
issue_num="${BASH_REMATCH[2]}"

# Fingerprint the branch's change-set: the commit HEAD points at (so work that
# was committed since the last review is detected — agents commit every turn,
# leaving a clean tree), every path git considers changed (tracked
# modifications, deletions, renames, and untracked new files), and the current
# content of each of those paths so edits to an already-dirty file are also
# detected across turns.
repo_root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
# Per-VM and ephemeral on cloud agents (fresh /tmp every run), so the first
# completion on a branch in a new VM always fires. Acceptable: the user still
# has final say on the review.
state_dir="/tmp/cursor-po-review-state"
mkdir -p "$state_dir"
state_key=$(printf '%s' "${repo_root}:${branch}" | sha256sum | cut -d' ' -f1)
state_file="${state_dir}/${state_key}.sha"

# NUL-delimited porcelain so paths with spaces/quotes/newlines survive. For
# rename and copy entries (`R`/`C` in either column) the *original* path is a
# separate NUL-terminated field that follows the entry and must be skipped.
hash_changed_files() {
  local entry code path _orig
  while IFS= read -r -d '' entry; do
    code=${entry:0:2}
    path=${entry:3}
    if [[ $code == *[RC]* ]]; then
      IFS= read -r -d '' _orig || true
    fi
    if [[ -f $path && ! -L $path ]]; then
      sha256sum -- "$path"
    fi
  done < <(git status --porcelain=v1 -z --untracked-files=all -- .)
}

current_fingerprint=$(
  {
    git rev-parse --verify HEAD 2>/dev/null || echo "unborn"
    git status --porcelain=v1 -z --untracked-files=all -- .
    hash_changed_files
  } | sha256sum | cut -d' ' -f1
)

previous_fingerprint=""
if [[ -f "$state_file" ]]; then
  previous_fingerprint=$(cat "$state_file")
fi

if [[ "$current_fingerprint" == "$previous_fingerprint" ]]; then
  echo '{}'
  exit 0
fi

lint_gate="${repo_root}/scripts/lint-gate.sh"
if [[ -x "$lint_gate" ]]; then
  lint_log=$(mktemp)
  if ! "$lint_gate" >"$lint_log" 2>&1; then
    tail -n 60 "$lint_log" >&2
    rm -f "$lint_log"
    jq -n '
      {
        followup_message: (
          "**Lint gate failed** (project hook — fix before PO review or merge)\n\n"
          + "CI Build must pass before this branch is ready for review. From the repo root run:\n\n"
          + "```bash\nscripts/lint-gate.sh\n```\n\n"
          + "Fix every build error and static-check finding, commit, push, "
          + "and confirm the GitHub **Build** workflow is green before marking the PR ready."
        )
      }
    '
    exit 0
  fi
  rm -f "$lint_log"
fi

echo "$current_fingerprint" > "$state_file"

issue_hint="GitHub issue #${issue_num} (branch: ${branch})."

jq -n --arg hint "$issue_hint" '
  {
    followup_message: (
      "**Auto PO review** (triggered by project hook — user has final say)\n\n"
      + "Act as Product Owner. Follow @.cursor/rules/po-feature-qa.mdc and @.cursor/rules/product-owner-workflow.mdc.\n\n"
      + "Review the implementation work completed in this conversation. "
      + $hint
      + "\n\nLoad the ticket acceptance criteria if possible, then report each criterion as met / not met / untestable. "
      + "End with a clear PO recommendation checkbox block. Do not write code."
    )
  }
'

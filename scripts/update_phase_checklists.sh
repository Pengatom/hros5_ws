#!/usr/bin/env bash
set -e

# Usage:
#   ./scripts/update_phase_checklists.sh
# or:
#   PHASES="P0 P1" ./scripts/update_phase_checklists.sh

REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)

# Which phases to process (default all)
PHASES="${PHASES:-P0 P1 P2 P3 P4 P5 P6}"

for PHASE in $PHASES; do
  echo "=== Updating checklist for phase $PHASE ==="

  # 1) Find the parent issue for this phase
  PARENT_NUM=$(gh issue list \
    --search "[${PHASE}] HR-OS5 Phase" \
    --state all \
    --json number,title \
    --jq '.[0].number' 2>/dev/null || true)

  if [ -z "$PARENT_NUM" ]; then
    echo "  !! Could not find parent issue for phase $PHASE (title like \"[${PHASE}] HR-OS5 Phase ...\")"
    continue
  fi

  echo "  Parent issue: #$PARENT_NUM"

  # 2) Get all issues with this phase label, except the parent
  TASK_LINES=$(gh issue list \
    --label "$PHASE" \
    --state open \
    --json number,title \
    --jq ".[] | select(.number != $PARENT_NUM) | \"- [ ] #\" + (.number|tostring) + \" \" + .title" \
    2>/dev/null || true)

  if [ -z "$TASK_LINES" ]; then
    echo "  (No open child issues with label $PHASE)"
    TASK_LINES="(no open tasks yet)"
  fi

  # 3) Get current body of the parent
  BODY=$(gh issue view "$PARENT_NUM" --json body --jq .body)

  # 4) Strip existing '## Tasks' section (if any)
  TMP_BODY_RAW=$(mktemp)
  TMP_BODY_HEAD=$(mktemp)
  TMP_BODY_NEW=$(mktemp)

  printf "%s\n" "$BODY" > "$TMP_BODY_RAW"

  # Keep everything up to (but not including) a line that starts with '## Tasks'
  awk '
    /^## Tasks[[:space:]]*$/ {flag=1}
    !flag {print}
  ' "$TMP_BODY_RAW" > "$TMP_BODY_HEAD"

  # 5) Compose new body: old head + fresh Tasks section
  {
    cat "$TMP_BODY_HEAD"
    printf "\n## Tasks\n\n"
    printf "%s\n" "$TASK_LINES"
    printf "\n"
  } > "$TMP_BODY_NEW"

  # 6) Update parent issue body
  gh issue edit "$PARENT_NUM" --body-file "$TMP_BODY_NEW"

  echo "  Updated #$PARENT_NUM with checklist for phase $PHASE."

  rm -f "$TMP_BODY_RAW" "$TMP_BODY_HEAD" "$TMP_BODY_NEW"
done

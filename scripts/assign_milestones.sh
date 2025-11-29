#!/usr/bin/env bash
set -e

# Map P0..P6 -> Milestone title
milestone_for_phase() {
  case "$1" in
    P0) echo "P0 – Foundation" ;;
    P1) echo "P1 – Teleop" ;;
    P2) echo "P2 – Digital twin" ;;
    P3) echo "P3 – IK" ;;
    P4) echo "P4 – Balance" ;;
    P5) echo "P5 – Walking" ;;
    P6) echo "P6 – Autonomous demo" ;;
    *)  echo "" ;;
  esac
}

# Get repo name
REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)

# Loop through all phases automatically
for PHASE in P0 P1 P2 P3 P4 P5 P6; do
  MILESTONE_TITLE="$(milestone_for_phase "$PHASE")"
  echo "Assigning milestone '$MILESTONE_TITLE' to issues labeled '$PHASE'..."

  # Find milestone number
  MILESTONE_NUM=$(gh api \
    "/repos/$REPO/milestones" \
    --jq ".[] | select(.title==\"$MILESTONE_TITLE\") | .number")

  if [ -z "$MILESTONE_NUM" ]; then
    echo "ERROR: Milestone '$MILESTONE_TITLE' not found!"
    exit 1
  fi

  # Get issues with this phase label
  gh issue list \
    --label "$PHASE" \
    --state open \
    --json number \
    --jq '.[].number' \
  | while read -r ISSUE_NUM; do
      echo "  -> Setting issue #$ISSUE_NUM to milestone $MILESTONE_NUM"
      gh api \
        -X PATCH \
        -H "Accept: application/vnd.github+json" \
        "/repos/$REPO/issues/$ISSUE_NUM" \
        -f milestone="$MILESTONE_NUM" >/dev/null
    done

done

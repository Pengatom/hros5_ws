#!/usr/bin/env bash
set -e

REPO=$(gh repo view --json nameWithOwner -q .nameWithOwner)

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

for PHASE in P0 P1 P2 P3 P4 P5 P6; do
  echo "=== Fixing parent for phase $PHASE ==="

  # parent is the one with label phase-Px
  PARENT_NUM=$(gh issue list \
    --label "phase-$PHASE" \
    --state all \
    --json number,title \
    --jq '.[0].number' 2>/dev/null || true)

  if [ -z "$PARENT_NUM" ]; then
    echo "  !! No parent issue found with label phase-$PHASE"
    continue
  fi

  echo "  Parent issue: #$PARENT_NUM"

  # find milestone number
  M_TITLE=$(milestone_for_phase "$PHASE")
  M_NUM=$(gh api "/repos/$REPO/milestones" \
    --jq ".[] | select(.title==\"$M_TITLE\") | .number")

  if [ -z "$M_NUM" ]; then
    echo "  !! Milestone '$M_TITLE' not found, skipping"
    continue
  fi
  echo "  Milestone: $M_TITLE (#$M_NUM)"

  # 1) add Px label to parent
  echo "  Adding label '$PHASE' to parent..."
  gh issue edit "$PARENT_NUM" --add-label "$PHASE"

  # 2) set milestone on parent
  echo "  Setting milestone on parent..."
  gh api -X PATCH -H "Accept: application/vnd.github+json" \
    "/repos/$REPO/issues/$PARENT_NUM" \
    -f milestone="$M_NUM" >/dev/null

  echo "  Done for $PHASE."
done

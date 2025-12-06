#!/usr/bin/env bash
set -e

# Usage:
#   ./scripts/sync_all_issues.sh            # run full sync, master issue defaults to #1
#   ./scripts/sync_all_issues.sh 5          # update master issue #5 instead
#
# Runs the helper scripts that keep GitHub issues and milestones in sync.
# Order matters: parents first, then milestones, then child checklists, then master issue.

MASTER_ISSUE_NUM="${1:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Syncing GitHub issues and milestones ==="

echo "--- [1/4] Fixing phase parent labels/milestones..."
"$SCRIPT_DIR/fix_phase_parents.sh"

echo "--- [2/4] Assigning milestones to phase issues..."
"$SCRIPT_DIR/assign_milestones.sh"

echo "--- [3/4] Updating phase parent checklists..."
"$SCRIPT_DIR/update_phase_checklists.sh"

echo "--- [4/4] Refreshing master roadmap issue #$MASTER_ISSUE_NUM..."
"$SCRIPT_DIR/update_master_issue.sh" "$MASTER_ISSUE_NUM"

echo "=== Done. GitHub tracking should now be in sync. ==="

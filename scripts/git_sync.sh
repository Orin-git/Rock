#!/usr/bin/env bash
# ROS2 工作区自动提交并推送到远程。
# - 宿主机：直接执行 git add / commit / push
# - Docker 内：写入请求文件，由宿主机 cron 约 1 分钟内执行（避免 root 污染 .git）
set -euo pipefail

REPO_HOST="${HOME}/ros2_ws"
REPO_DOCKER="/ros2_ws"
REASON="${1:-manual}"
LOCK_NAME=".git_sync.lock"
REQUEST_NAME=".git_sync_request"

is_docker() { [ -f /.dockerenv ]; }

repo_dir() {
  if is_docker && [ -d "$REPO_DOCKER/.git" ]; then
    echo "$REPO_DOCKER"
  elif [ -d "$REPO_HOST/.git" ]; then
    echo "$REPO_HOST"
  elif [ -d "$REPO_DOCKER/.git" ]; then
    echo "$REPO_DOCKER"
  else
    echo "找不到 ros2_ws git 仓库" >&2
    exit 1
  fi
}

REPO="$(repo_dir)"
LOG_DIR="$REPO/logs"
REQUEST_FILE="$REPO/$REQUEST_NAME"
LOCK_FILE="$REPO/$LOCK_NAME"
mkdir -p "$LOG_DIR"

log() {
  local line="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
  echo "$line" | tee -a "$LOG_DIR/git_sync.log"
}

# Docker 内只发请求，真正同步在宿主机执行
if is_docker; then
  date '+%Y-%m-%d %H:%M:%S' > "$REQUEST_FILE"
  echo "reason=${REASON}" >> "$REQUEST_FILE"
  echo "已请求 Git 同步（reason=${REASON}）。"
  echo "宿主机 cron 约 1 分钟内会提交并 push；日志：/ros2_ws/logs/git_sync.log"
  echo "也可以在宿主机直接运行：~/ros2_ws/scripts/git_sync.sh"
  exit 0
fi

if ! command -v git >/dev/null 2>&1; then
  log "错误: 宿主机未安装 git"
  exit 1
fi

# 简单锁，避免并发
if [ -f "$LOCK_FILE" ]; then
  lock_pid="$(cut -d' ' -f1 "$LOCK_FILE" 2>/dev/null || true)"
  if [ -n "${lock_pid:-}" ] && kill -0 "$lock_pid" 2>/dev/null; then
    log "跳过: 已有同步在运行 (pid=$lock_pid)"
    exit 0
  fi
  rm -f "$LOCK_FILE"
fi
echo "$$ $(date '+%Y-%m-%d %H:%M:%S') $REASON" > "$LOCK_FILE"
trap 'rm -f "$LOCK_FILE"' EXIT

cd "$REPO"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  log "错误: $REPO 不是 git 仓库"
  exit 1
fi

# 确保在 main 且有 remote
if ! git remote get-url origin >/dev/null 2>&1; then
  log "错误: 未配置 origin 远程"
  exit 1
fi

# 拉取远程变更，减少非快进失败（允许失败继续尝试提交）
git fetch origin 2>>"$LOG_DIR/git_sync.log" || true
current_branch="$(git rev-parse --abbrev-ref HEAD)"
if git rev-parse --verify "origin/${current_branch}" >/dev/null 2>&1; then
  git pull --rebase --autostash origin "$current_branch" 2>>"$LOG_DIR/git_sync.log" || {
    log "警告: pull --rebase 失败，继续尝试本地提交"
  }
fi

# 暂存工作区变更（不含 logs、构建产物等由 .gitignore 控制）
git add -A

if git diff --cached --quiet; then
  log "无变更，跳过提交 (${REASON})"
  # 仍可尝试 push（以防本地有未推送 commit）
else
  msg="auto-sync: $(date '+%Y-%m-%d %H:%M:%S') [${REASON}]"
  if [ "$REASON" = "daily" ]; then
    msg="daily backup: $(date '+%Y-%m-%d') 17:00"
  fi
  git commit -m "$msg" >>"$LOG_DIR/git_sync.log" 2>&1
  log "已提交: $msg"
fi

# 是否有未推送 commit
if git rev-parse --verify "origin/${current_branch}" >/dev/null 2>&1; then
  ahead="$(git rev-list --count "origin/${current_branch}..HEAD" 2>/dev/null || echo 0)"
else
  ahead=1
fi

if [ "${ahead:-0}" -eq 0 ]; then
  log "无待推送提交 (${REASON})"
  exit 0
fi

if git push -u origin "$current_branch" >>"$LOG_DIR/git_sync.log" 2>&1; then
  log "已推送到 origin/${current_branch} (${REASON})"
else
  log "错误: git push 失败，详见 logs/git_sync.log"
  exit 1
fi

#!/usr/bin/env bash
# ROS2 工作区自动提交并推送到远程。
# - 宿主机：直接执行 git add / commit / push（逐步打印进度）
# - Docker 内：请求宿主机执行，并等待完成、实时显示进度
set -euo pipefail

REPO_HOST="${HOME}/ros2_ws"
REPO_DOCKER="/ros2_ws"
REASON="${1:-manual}"
LOCK_NAME=".git_sync.lock"
REQUEST_NAME=".git_sync_request"
STATUS_NAME=".git_sync_status"
PROGRESS_NAME=".git_sync_progress"
MAX_WAIT_SEC="${GIT_SYNC_WAIT_SEC:-120}"

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
STATUS_FILE="$REPO/$STATUS_NAME"
PROGRESS_FILE="$REPO/$PROGRESS_NAME"
mkdir -p "$LOG_DIR"

log() {
  local line="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
  echo "$line" | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"
}

step() {
  # 进度步骤：终端 + 日志 + 进度文件，供 Docker 侧 tail
  local line=">>> $*"
  echo "$line"
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_DIR/git_sync.log" >>"$PROGRESS_FILE"
}

write_status() {
  local code="$1"
  local msg="$2"
  local req_id="${3:-}"
  {
    echo "status=${code}"
    echo "time=$(date '+%Y-%m-%d %H:%M:%S')"
    echo "reason=${REASON}"
    echo "request_id=${req_id}"
    echo "message=${msg}"
  } >"$STATUS_FILE"
}

# ---------- Docker：发请求并等待结果 ----------
if is_docker; then
  req_id="req-$(date +%s)-$$"
  : >"$PROGRESS_FILE"
  rm -f "$STATUS_FILE"
  {
    date '+%Y-%m-%d %H:%M:%S'
    echo "reason=${REASON}"
    echo "request_id=${req_id}"
  } >"$REQUEST_FILE"

  echo "========================================"
  echo " Git 同步已请求（由宿主机执行）"
  echo " reason : ${REASON}"
  echo " id     : ${req_id}"
  echo " 最长等待: ${MAX_WAIT_SEC}s"
  echo "========================================"
  echo "等待宿主机开始处理..."
  echo "（若超过约 1 分钟无动静，请在宿主机检查: systemctl --user status ros2-git-sync-watch.timer）"
  echo

  elapsed=0
  seen_progress=0
  start_size=0
  [ -f "$PROGRESS_FILE" ] && start_size=$(wc -c <"$PROGRESS_FILE" | tr -d ' ')

  while [ "$elapsed" -lt "$MAX_WAIT_SEC" ]; do
    # 打印新增进度
    if [ -f "$PROGRESS_FILE" ]; then
      cur_size=$(wc -c <"$PROGRESS_FILE" | tr -d ' ')
      if [ "$cur_size" -gt "$start_size" ]; then
        tail -c +"$((start_size + 1))" "$PROGRESS_FILE" 2>/dev/null || true
        start_size=$cur_size
        seen_progress=1
      fi
    fi

    if [ -f "$STATUS_FILE" ]; then
      # 再刷一次进度文件结尾
      if [ -f "$PROGRESS_FILE" ]; then
        cur_size=$(wc -c <"$PROGRESS_FILE" | tr -d ' ')
        if [ "$cur_size" -gt "$start_size" ]; then
          tail -c +"$((start_size + 1))" "$PROGRESS_FILE" 2>/dev/null || true
        fi
      fi
      status_val="$(grep '^status=' "$STATUS_FILE" | tail -1 | cut -d= -f2-)"
      msg_val="$(grep '^message=' "$STATUS_FILE" | tail -1 | cut -d= -f2-)"
      rid_val="$(grep '^request_id=' "$STATUS_FILE" | tail -1 | cut -d= -f2-)"
      # 接受匹配 request_id 或未设置（兼容）
      if [ -z "${rid_val:-}" ] || [ "$rid_val" = "$req_id" ]; then
        echo
        echo "========================================"
        if [ "$status_val" = "ok" ]; then
          echo " 完成: ${msg_val}"
          echo "========================================"
          exit 0
        else
          echo " 失败: ${msg_val}"
          echo " 日志: /ros2_ws/logs/git_sync.log"
          echo "========================================"
          exit 1
        fi
      fi
    fi

    # 心跳：未看到进度时每 10 秒提示一次
    if [ "$seen_progress" -eq 0 ] && [ $((elapsed % 10)) -eq 0 ] && [ "$elapsed" -gt 0 ]; then
      echo "... 仍在等待宿主机处理 (${elapsed}s/${MAX_WAIT_SEC}s)"
    fi

    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo
  echo "========================================"
  echo " 超时: ${MAX_WAIT_SEC}s 内未完成"
  echo " 可查看: /ros2_ws/logs/git_sync.log"
  echo " 宿主机手动: ~/ros2_ws/scripts/git_sync.sh"
  echo "========================================"
  exit 1
fi

# ---------- 宿主机：真正执行 ----------
if ! command -v git >/dev/null 2>&1; then
  log "错误: 宿主机未安装 git"
  write_status "fail" "宿主机未安装 git"
  exit 1
fi

# 从请求文件读取 request_id（若有）
REQ_ID=""
if [ -f "$REQUEST_FILE" ] && grep -q '^request_id=' "$REQUEST_FILE" 2>/dev/null; then
  REQ_ID="$(grep '^request_id=' "$REQUEST_FILE" | tail -1 | cut -d= -f2-)"
fi
# 兼容由 watch 脚本传入的第二参数
if [ -n "${2:-}" ]; then
  REQ_ID="$2"
fi

# 简单锁
if [ -f "$LOCK_FILE" ]; then
  lock_pid="$(cut -d' ' -f1 "$LOCK_FILE" 2>/dev/null || true)"
  if [ -n "${lock_pid:-}" ] && kill -0 "$lock_pid" 2>/dev/null; then
    log "跳过: 已有同步在运行 (pid=$lock_pid)"
    write_status "fail" "已有同步在运行 pid=$lock_pid" "$REQ_ID"
    exit 0
  fi
  rm -f "$LOCK_FILE"
fi
echo "$$ $(date '+%Y-%m-%d %H:%M:%S') $REASON" >"$LOCK_FILE"
: >"$PROGRESS_FILE"
trap 'rm -f "$LOCK_FILE"' EXIT

cd "$REPO"
step "[1/6] 工作区: $REPO  (reason=${REASON})"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  log "错误: $REPO 不是 git 仓库"
  write_status "fail" "不是 git 仓库" "$REQ_ID"
  exit 1
fi

if ! git remote get-url origin >/dev/null 2>&1; then
  log "错误: 未配置 origin 远程"
  write_status "fail" "未配置 origin" "$REQ_ID"
  exit 1
fi

step "[2/6] 拉取远程 (fetch + pull --rebase)..."
if git fetch origin 2>&1 | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"; then
  :
else
  step "    fetch 失败，继续本地提交"
fi

current_branch="$(git rev-parse --abbrev-ref HEAD)"
step "    当前分支: ${current_branch}"

if git rev-parse --verify "origin/${current_branch}" >/dev/null 2>&1; then
  if git pull --rebase --autostash origin "$current_branch" 2>&1 | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"; then
    step "    pull 完成"
  else
    step "    警告: pull --rebase 失败，继续尝试本地提交"
  fi
else
  step "    远程尚无此分支，跳过 pull"
fi

step "[3/6] 暂存变更 (git add -A)..."
git add -A
# 显示将要提交的内容摘要
if git diff --cached --quiet; then
  step "    无新文件变更"
else
  step "    变更文件:"
  git diff --cached --name-status 2>/dev/null | while read -r line; do
    echo "      $line" | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"
  done
fi

step "[4/6] 提交..."
if git diff --cached --quiet; then
  step "    无变更，跳过 commit"
else
  msg="auto-sync: $(date '+%Y-%m-%d %H:%M:%S') [${REASON}]"
  if [ "$REASON" = "daily" ]; then
    msg="daily backup: $(date '+%Y-%m-%d') 17:00 Asia/Shanghai"
  fi
  git commit -m "$msg" 2>&1 | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"
  step "    已提交: $msg"
fi

step "[5/6] 检查待推送提交..."
if git rev-parse --verify "origin/${current_branch}" >/dev/null 2>&1; then
  ahead="$(git rev-list --count "origin/${current_branch}..HEAD" 2>/dev/null || echo 0)"
else
  ahead=1
fi
step "    领先远程 ${ahead} 个提交"

if [ "${ahead:-0}" -eq 0 ]; then
  step "[6/6] 无需 push（已是最新）"
  write_status "ok" "无变更，已与远程一致" "$REQ_ID"
  log "无待推送提交 (${REASON})"
  exit 0
fi

step "[6/6] 推送到 origin/${current_branch}..."
if git push -u origin "$current_branch" 2>&1 | tee -a "$LOG_DIR/git_sync.log" "$PROGRESS_FILE"; then
  step "    push 成功"
  write_status "ok" "已推送到 origin/${current_branch}" "$REQ_ID"
  log "已推送到 origin/${current_branch} (${REASON})"
  exit 0
else
  step "    push 失败"
  write_status "fail" "git push 失败，见 logs/git_sync.log" "$REQ_ID"
  log "错误: git push 失败，详见 logs/git_sync.log"
  exit 1
fi

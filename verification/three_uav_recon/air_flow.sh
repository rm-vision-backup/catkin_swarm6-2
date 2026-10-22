#!/usr/bin/env bash
# air_flow.sh —— 实飞分阶段测试：空中 Stage 1（链路/odom）→ Stage 2（跟随环）→ 全流程 + 机载 rosbag
#
# 为什么：9/22、9/23 两轮 coverage 失败的直接判据是"跟随者拿不到领机 odom"；10-09 地面彩排
# 把 prepare→coverage 跑通了，但**空中链路条件（姿态/多径/距离/电机干扰）没覆盖**。本脚本把
# no_prop_link_test.md 的 Stage 1/2 从"地面/无桨"搬到"空中/实飞"，并把 rosbag 与健康 JSONL
# 两层证据接进流程：出现 9/22 型错误时，第一步就跑 `odom` 回答"odom 通不通、断在哪一段"。
#
# 前置（与对应 README §零/§二/§三 相同；**不带任何贴地覆盖参数、不带 noprop**）：
#   - 按 three/five_uav_recon README §1.2 摆好（现场踩过：摆错原点角机 → prepare 多飞几米）；
#   - 机载栈用实飞命令起（README §二 去掉 noprop）；GCS 起栈（README §三，含 ROS_HOSTNAME 覆盖）；
#   - 全体遥控 arm + OFFBOARD（自动升至 5 m 悬停，遥控器在手）；
#   - 板载 rosbag 脚本为新版：bag 子命令会先核对（旧版提示 rsync，见 three README §九）。
#
# 用法（在 GCS 上跑；子命令含义见 three README §九）：
#   bash verification/three_uav_recon/air_flow.sh odom            # 任何时刻：odom 通畅性快查（分层）
#   bash verification/three_uav_recon/air_flow.sh bag [秒=600]    # 全体同时录 rosbag（跟机录 leader，领机录其余所有机）
#   bash verification/three_uav_recon/air_flow.sh stage1 [秒=60]  # 空中链路观测窗 + 判定
#   bash verification/three_uav_recon/air_flow.sh stage2          # 只发 coverage（跟随环）
#   bash verification/three_uav_recon/air_flow.sh full            # prepare + coverage 全流程
#
# 机数参数化（默认 = 三机 8/10/13，向后兼容；五机见 five_uav_recon/README §五）：
#   AIR_FLOW_UAVS="6 8 10 12 13"   机号列表（含领机）
#   AIR_FLOW_LEADER=8              领机机号（从跟随者循环里排除；默认 8）
#   AIR_FLOW_DIR=five_uav_recon    driver 所在目录（默认 three_uav_recon）
#
# 判据（stage1，按消费方分档）：
#   - 非领机源（1 m 避碰门禁，pose_timeout=1 s）：gt1000=0、stalls=0、age ≤1 s；
#   - 领机源（跟随环 LEADER_LOST=3 s）：gt3000=0、age ≤3 s；1–3 s 空档记 ⚠，不判失败；
#   - 所有源：rx/msgs 持续增长、无缺源；link/连接类事件仍判失败。
#   可用 AIR_FLOW_LEADER_TOL_S（默认 3.0）/ AIR_FLOW_PEER_TOL_S（默认 1.0）覆盖。
#   每台 local odom 的 rx/tx 同步增长。任一不满足 = 记时刻、留文件。
# 退出码：0 = 该子命令判定通过；1 = 不通过（odom 子命令恒 0，只报告）。
# =============================================================================
set -u

WS=$(cd "$(dirname "$0")/../.." && pwd)
UAVS=${AIR_FLOW_UAVS:-"8 10 13"}
LEADER=${AIR_FLOW_LEADER:-8}
FLOW_DIR=${AIR_FLOW_DIR:-three_uav_recon}
LEADER_TOL_S=${AIR_FLOW_LEADER_TOL_S:-3.0}
PEER_TOL_S=${AIR_FLOW_PEER_TOL_S:-1.0}
UAVS_CSV=$(echo $UAVS | tr ' ' ',')
NUAV=$(echo $UAVS | wc -w)
FOLLOWERS=""
for _n in $UAVS; do [ "$_n" = "$LEADER" ] || FOLLOWERS="$FOLLOWERS $_n"; done
CONFIG="$WS/src/tcp_to_ros/tools/configs/onboard.env"
[ -f "$CONFIG" ] && . "$CONFIG"

CMD="${1:-odom}"; shift 2>/dev/null || true

# 本机 GCS 自带 master；~/.bashrc 的 ROS_HOSTNAME=yjq 会解析到不存在的 192.168.1.100（roslaunch/
# rostopic 自检卡死，2026-10-09 实测）。统一覆盖，不给操作员留坑。
export ROS_MASTER_URI="${GCS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${GCS_ROBOT_IP:-192.168.5.241}"
unset ROS_HOSTNAME
set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck source=/dev/null
source "$WS/devel/setup.bash" 2>/dev/null
set -u

ip_of() { echo "${BOARD_IP_PREFIX:-192.168.5.}$((10 * $1 + 1))"; }

peers_for() {  # $1=当前机号；输出要记录其转发话题的对端（跟机=领机；领机=其余所有机）
  local me="$1" out=""
  if [ "$me" = "$LEADER" ]; then
    for p in $UAVS; do [ "$p" = "$me" ] || out+="${out:+,}$p"; done
  else
    out="$LEADER"
  fi
  printf '%s' "$out"
}

read_health() {  # $1=ip $2=mode(quick|window) $3=窗起点 $4=领机机号 $5=在线邻机数 $6=领机容忍s $7=其他源容忍s
  # shellcheck disable=SC2029
  timeout 40 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -T -o ConnectTimeout=8 \
    -o StrictHostKeyChecking=no "${BOARD_SSH_USER:-nvidia}@$1" \
    "python3 - '$2' '${3:-0}' '${4:-}' '${5:-}' '${6:-$LEADER_TOL_S}' '${7:-$PEER_TOL_S}'" <<'PY'
import json, os, socket, sys, time

mode = sys.argv[1] if len(sys.argv) > 1 else "quick"
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
leader_raw = sys.argv[3].strip() if len(sys.argv) > 3 else ""
expect_peers = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4].strip() else None
leader_tol = float(sys.argv[5]) if len(sys.argv) > 5 and sys.argv[5].strip() else 3.0
peer_tol = float(sys.argv[6]) if len(sys.argv) > 6 and sys.argv[6].strip() else 1.0
leader_src = ((leader_raw if leader_raw.upper().startswith("UAV")
               else "UAV" + leader_raw) if leader_raw else "")
uav = socket.gethostname()
f = os.path.expanduser("~/catkin_swarm6-2/.ros_home/log/health/%s_bridge_health.jsonl" % uav)
if not os.path.exists(f):
    print("  [%s] 无健康文件（栈没起 / 代码没同步？）" % uav)
    sys.exit(1)
rows, events = [], []
for line in open(f):
    try:
        d = json.loads(line)
    except ValueError:
        continue
    (rows if d.get("event") == "sample" else events).append(d)
if not rows:
    print("  [%s] 文件里没有 sample 行" % uav)
    sys.exit(1)


def odom_of(items):
    for item in items or []:
        if item.get("topic") == "swarm_pose":
            return item
    return None


def fmt_age_ms(value):
    return "n/a" if value is None else "%.0fms" % value


def brief(item, side):
    if item is None:
        return "无该话题条目"
    gt1000 = (item.get("gt") or {}).get("gt1000", 0)
    if side == "local":
        # local 行字段是 age_ms（只有 remote 行有 age_max_ms）；drop 是限频丢弃，正常非零
        return "rx=%d tx=%d drop=%d age=%s gt1000=%d stalls=%d" % (
            item.get("rx", 0), item.get("tx", 0), item.get("drop", 0),
            fmt_age_ms(item.get("age_ms")), gt1000, item.get("stalls", 0))
    stale = item.get("stale_srcs") or []
    return "rx=%d srcs=%s age_max=%s%s gt1000=%d stalls=%d" % (
        item.get("rx", 0),
        item.get("srcs") if item.get("srcs") is not None else "?",
        fmt_age_ms(item.get("age_max_ms")),
        ("  stale_srcs=%s" % [s.get("src") for s in stale]) if stale else "",
        gt1000, item.get("stalls", 0))


last = rows[-1]
if mode == "quick":
    age_file = time.time() - os.path.getmtime(f)
    print("  [%s] 健康文件 %.1fs 前更新%s" % (uav, age_file, "" if age_file < 6 else "  ✗ 桥可能在静默"))
    print("     本机发出 swarm_pose: " + brief(odom_of(last.get("local")), "local"))
    print("     收到邻机 swarm_pose: " + brief(odom_of(last.get("remote")), "remote"))
    for ev in events[-3:]:
        print("     近期事件: %s %s %s" % (ev.get("event"), ev.get("link_event") or "",
                                        ev.get("src") or ev.get("detail") or ""))
    sys.exit(0)

# window 模式：对 t0 之后的样本做计数差分 + 事件扫描
win = [d for d in rows if (d.get("t") or 0.0) >= t0]
if len(win) < 2:
    print("  [%s] 窗内样本不足（%d）" % (uav, len(win)))
    sys.exit(1)
a, b = win[0], win[-1]
dur = (b.get("t") or 0.0) - (a.get("t") or 0.0)
ok = True


def diff_local():
    global ok
    x, y = odom_of(a.get("local")), odom_of(b.get("local"))
    if x is None or y is None:
        # 整窗没有该侧 odom 数据行 = 该窗内 odom 从没出现在账本里，必须判 ✗。
        print("  [%s] 本机发出: 无条目 ✗（窗内无 local odom 数据行）" % uav)
        ok = False
        return
    drx = y.get("rx", 0) - x.get("rx", 0)
    dtx = y.get("tx", 0) - x.get("tx", 0)
    dg1 = (y.get("gt") or {}).get("gt1000", 0) - (x.get("gt") or {}).get("gt1000", 0)
    dst = y.get("stalls", 0) - x.get("stalls", 0)
    gap = y.get("gap_max_ms") or 0  # 累计值（自进程启动）；窗口判据看 gt1000 增量
    # 发送侧：rx（mavros 到达）与 tx（限频后实发）都要增长；drop 为限频丢弃，不计入判据
    good = drx > 0 and dtx > 0 and dg1 == 0 and dst == 0
    print("  [%s] 本机发出: rx +%d（%.1f/s）tx +%d（%.1f/s）gt1000 +%d stalls +%d 累计gap_max %.0fms %s"
          % (uav, drx, drx / dur if dur > 0 else -1,
             dtx, dtx / dur if dur > 0 else -1, dg1, dst, gap, "✓" if good else "✗"))
    if not good:
        ok = False


def src_map(sample):
    return {s.get("src"): s for s in (sample.get("sources") or []) if s.get("src")}


def gt_delta(x, y, key):
    return ((y.get("gt") or {}).get(key, 0) - (x.get("gt") or {}).get(key, 0))


def remote_check():
    global ok
    ra, rb = odom_of(a.get("remote")), odom_of(b.get("remote"))
    if ra is None or rb is None:
        print("  [%s] 收到邻机 swarm_pose: 无条目 ✗" % uav)
        ok = False
        return
    drx = rb.get("rx", 0) - ra.get("rx", 0)
    if drx <= 0:
        print("  [%s] 收到邻机: rx 未增长（+%d）✗" % (uav, drx))
        ok = False
    if expect_peers is not None and (rb.get("srcs") or 0) != expect_peers:
        print("  [%s] 收到邻机: srcs=%s，期望 %d ✗"
              % (uav, rb.get("srcs"), expect_peers))
        ok = False
    sa, sb = src_map(a), src_map(b)
    if not sb:
        print("  [%s] 收到邻机: 无 sources 明细 ✗" % uav)
        ok = False
        return
    bad, warned = [], []
    for src in sorted(set(sa) | set(sb)):
        x, y = sa.get(src), sb.get(src)
        if x is None or y is None:
            print("  [%s] 源 %-6s: 窗口%s无记录 ✗"
                  % (uav, src, "末" if y is None else "首"))
            ok = False
            continue
        dmsgs = y.get("msgs", 0) - x.get("msgs", 0)
        dg1 = gt_delta(x, y, "gt1000")
        dg3 = gt_delta(x, y, "gt3000")
        dst = y.get("stalls", 0) - x.get("stalls", 0)
        age = y.get("age_ms")
        is_leader = (src == leader_src)
        tol = leader_tol if is_leader else peer_tol
        if dmsgs <= 0:
            good = False
        elif is_leader:
            # 跟随环 3 s 才判 LEADER_LOST；1–3 s 空档只记警告。
            good = (dg3 == 0 and age is not None and age <= tol * 1000)
        else:
            # 1 m 距离门禁用 pose_timeout_s=1 s，非领机源仍按 1 s。
            good = (dg1 == 0 and dst == 0 and age is not None
                    and age <= tol * 1000)
        if not good:
            bad.append((src, is_leader, dmsgs, dg1, dg3, dst, age))
            print("  [%s] 源 %-6s%s: msgs +%d gt1000 +%d gt3000 +%d stalls +%d age %s ✗"
                  % (uav, src, "(领机)" if is_leader else "",
                     dmsgs, dg1, dg3, dst, fmt_age_ms(age)))
            ok = False
        elif is_leader and (dg1 > 0 or dst > 0):
            warned.append(src)
            print("  [%s] 源 %-6s(领机): msgs +%d gt1000 +%d gt3000 +%d stalls +%d age %s ⚠"
                  % (uav, src, dmsgs, dg1, dg3, dst, fmt_age_ms(age)))
    print("  [%s] 源连续性: %d 源检查，%d 失败，%d 领机警告"
          % (uav, len(set(sa) | set(sb)), len(bad), len(warned)))
    # aggregate remote 行的 stale_srcs/age_max 带源名，用于抓“整包在走、单话题 stale”。
    for item in (rb.get("stale_srcs") or []):
        src = item.get("src") or ""
        age = item.get("age_ms") or 0.0
        tol = leader_tol if src == leader_src else peer_tol
        if age > tol * 1000:
            print("  [%s] swarm_pose 源 %-6s stale %.0fms > %.1fs ✗"
                  % (uav, src, age, tol))
            ok = False
        else:
            print("  [%s] swarm_pose 源 %-6s stale %.0fms（≤%.1fs，警告）⚠"
                  % (uav, src, age, tol))
    am, asrc = rb.get("age_max_ms"), rb.get("age_max_src") or ""
    if am is not None and asrc:
        tol = leader_tol if asrc == leader_src else peer_tol
        if am > tol * 1000:
            print("  [%s] swarm_pose age_max %.0fms（%s）> %.1fs ✗"
                  % (uav, am, asrc, tol))
            ok = False
        elif asrc == leader_src and am > peer_tol * 1000:
            print("  [%s] swarm_pose age_max %.0fms（领机 %s），按 %.1fs 容忍 ⚠"
                  % (uav, am, asrc, leader_tol))
        else:
            print("  [%s] swarm_pose age_max %.0fms（%s）≤%.1fs ✓"
                  % (uav, am, asrc, tol))


print("[%s] 窗口 %.0fs（领机源 %.1fs / 其他源 %.1fs）"
      % (uav, dur, leader_tol, peer_tol))
diff_local()
remote_check()
evs = [e for e in events if (e.get("t") or 0.0) >= t0
       and e.get("event") in ("stall_start", "stall_end", "link", "connect_retried",
                              "connect_delayed", "connected", "disconnected", "handshake_succeeded")]
fail_evs, warn_evs = [], []
for e in evs:
    src = e.get("src") or ""
    if src and src == leader_src and e.get("event") in ("stall_start", "stall_end"):
        # 领机源：只有 ≥3 s 的 stall 才算失败，1–3 s 记录为警告。
        if e.get("event") == "stall_start" and (e.get("silent_s") or 0) >= leader_tol:
            fail_evs.append(e)
        else:
            warn_evs.append(e)
    else:
        fail_evs.append(e)
if fail_evs:
    ok = False
    for e in fail_evs[-6:]:
        print("  [%s] ✗ 窗口内事件 %s %s %s" % (uav, e.get("event"), e.get("link_event") or "",
                                              e.get("src") or ""))
if warn_evs:
    for e in warn_evs[-6:]:
        print("  [%s] ⚠ 领机链路事件（≤%.1fs 容忍）%s %s %s"
              % (uav, leader_tol, e.get("event"), e.get("link_event") or "",
                 e.get("src") or ""))
if not evs:
    print("  [%s] 窗口内无 stall/link 事件 ✓" % uav)
sys.exit(0 if ok else 1)
PY
}

followers_events() {  # $1=起始 epoch：打印各跟随者窗口内的跟随环事件
  for n in $FOLLOWERS; do
    # shellcheck disable=SC2029
    timeout 30 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -T -o ConnectTimeout=8 \
      -o StrictHostKeyChecking=no "${BOARD_SSH_USER:-nvidia}@$(ip_of "$n")" "python3 - $1" <<'PY'
import json, os, socket, sys, time
t0 = float(sys.argv[1])
uav = socket.gethostname()
f = os.path.expanduser("~/catkin_swarm6-2/.ros_home/log/health/%s_follower_events.jsonl" % uav)
if not os.path.exists(f):
    print("  [%s] 无跟随环事件文件（跟随环没跑过）" % uav)
    sys.exit(0)
rows = [json.loads(l) for l in open(f) if l.strip()]
recent = [r for r in rows if isinstance(r.get("t"), (int, float)) and r["t"] >= t0]
if not recent:
    print("  [%s] 本轮无跟随环事件" % uav)
for r in recent:
    print("  [%s] %s %s" % (uav, time.strftime("%H:%M:%S", time.localtime(r["t"])),
                            json.dumps({k: v for k, v in r.items()
                                        if k in ("event", "wait_s", "reason", "message", "ran_s",
                                                 "leader_age_s", "offset")}, ensure_ascii=False)))
PY
  done
}

driver_verdict() {  # $1=jsonl $2=阶段名（打印 phase_result 与逐机状态）
  python3 - "$1" <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
for d in rows:
    if d.get("event") == "phase_result":
        print("  [%s] %s（%s）" % (d.get("outcome"), d.get("phase"), ", ".join(
            "%s=%s" % kv for kv in (d.get("statuses") or {}).items())))
    elif d.get("event") == "command_feedback" and d.get("status") in ("FAILURE",):
        print("  %s FAILURE %s %s" % (d.get("robot_id"), d.get("error_code") or "",
                                      (d.get("error_message") or "")[:80]))
PY
}

check_gates() {  # stage2/full 前置：各机 armed+OFFBOARD、标定 ready、TCP 在听
  rc=0
  ready=$(timeout 10 rostopic echo -n1 /group_a/calibration_ready 2>/dev/null | head -1)
  [ "$ready" = "data: True" ] && echo "  calibration_ready=True ✓" || { echo "  calibration_ready=${ready:-读不到} ✗"; rc=1; }
  for n in $UAVS; do
    st=$(timeout 8 rostopic echo -n1 "/UAV$n/mavros/state" 2>/dev/null | grep -E "armed|mode" | tr -d ' \n')
    case "$st" in *"armed:True"*"OFFBOARD"*) echo "  UAV$n armed+OFFBOARD ✓" ;; *) echo "  UAV$n $st ✗（未 arm/未 OFFBOARD）"; rc=1 ;; esac
  done
  nc -z "${GCS_ROBOT_IP:-192.168.5.241}" 39001 >/dev/null 2>&1 && echo "  GCS TCP 39001 在听 ✓" || { echo "  GCS TCP 39001 不通 ✗（先起对应的 gcs_a_real_*_uav.launch）"; rc=1; }
  return $rc
}

case "$CMD" in
  odom)
    echo "== odom 通畅性快查（$(date +%H:%M:%S)；$NUAV 台并行）=="
    for n in $UAVS; do read_health "$(ip_of "$n")" quick 0 & done
    wait
    cat <<EOM
  判读：领机（UAV$LEADER）看"本机发出 odom"（rx/tx 在涨，drop 是限频丢弃属正常）；跟随者看
  "收到邻机 odom"（age_max 应 ~50 ms 内、stale_srcs 空）。stale_srcs 里出现 UAV$LEADER = 跟随环
  输入断了（9/22 型故障），立刻跑 \`bag\`（若还没录）并在两分钟后复跑本命令看是否恢复。
EOM
    ;;
  bag)
    DUR="${1:-600}"
    if ! echo " $UAVS " | grep -q " $LEADER "; then
      echo "AIR_FLOW_LEADER=$LEADER 不在 AIR_FLOW_UAVS 中" >&2; exit 2
    fi
    echo "== 同录 rosbag（${DUR}s，$NUAV 台；跟机录 leader，领机录其余所有机）=="
    rc=0
    for n in $UAVS; do
      ip=$(ip_of "$n")
      peers=$(peers_for "$n"); [ -n "$peers" ] || peers="none"
      # 板上脚本版本核对：新版含"链路哨兵"注释；旧版先 rsync（README §九）
      if ! timeout 20 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -n -o ConnectTimeout=8 -o StrictHostKeyChecking=no \
             "${BOARD_SSH_USER:-nvidia}@$ip" 'grep -q "链路哨兵" ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/record_rosbag.sh'; then
        echo "  UAV$n: 板上 record_rosbag.sh 是旧版 ✗ —— 先 rsync（见 README §九 命令）"; rc=1; continue
      fi
      timeout 20 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -n -o ConnectTimeout=8 -o StrictHostKeyChecking=no \
        "${BOARD_SSH_USER:-nvidia}@$ip" \
        "setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/record_rosbag.sh -u $peers -n air -d $DUR </dev/null >~/rosbag_record.log 2>&1 &"
      echo "  UAV$n: 已启动（-u $peers，>~/rosbag_record.log）"
    done
    [ "$rc" -eq 0 ] && echo "  录完/停止后：Ctrl-C 由 -d 自动收；取包 scp 见 README §九（log_fetch.sh 不采 bag）"
    exit "$rc"
    ;;
  stage1)
    SEC="${1:-60}"
    EXPECT_PEERS=$((NUAV - 1))
    echo "== Stage 1：链路观测（${SEC}s 窗；领机源 ${LEADER_TOL_S}s / 其他源 ${PEER_TOL_S}s）=="
    echo "-- 窗内快查（起点基线）--"
    for n in $UAVS; do read_health "$(ip_of "$n")" quick 0 & done
    wait
    T0=$(date +%s)
    echo "-- 观测中 ${SEC}s（$(date +%H:%M:%S) 起）--"
    sleep "$SEC"
    echo "-- 窗后判定（起点 $(date -d @$T0 +%H:%M:%S)）--"
    # 并行读取：窗口对所有机一致，避免逐机串行把后段窗口越拉越长。
    TMPD=$(mktemp -d)
    for n in $UAVS; do
      ( out=$(read_health "$(ip_of "$n")" window "$T0" "$LEADER" "$EXPECT_PEERS" \
                "$LEADER_TOL_S" "$PEER_TOL_S" 2>&1); rc=$?
        printf '%s\n' "$out" > "$TMPD/$n.out"
        echo "$rc" > "$TMPD/$n.rc" ) &
    done
    wait
    rc=0
    for n in $UAVS; do
      cat "$TMPD/$n.out"
      [ "$(cat "$TMPD/$n.rc")" -eq 0 ] || rc=1
    done
    rm -rf "$TMPD"
    echo "  证据：各机 <ws>/.ros_home/log/health/UAVn_bridge_health.jsonl（⚠=领机源 1–3 s 容忍）"
    [ "$rc" -eq 0 ] && echo "== Stage 1 通过：窗内无超出容忍的链路缺口 ==" || echo "== Stage 1 不通过：按上面 ✗ 项定层（发送/桥/链路/接收）=="
    exit "$rc"
    ;;
  stage2|full)
    if [ "$CMD" = stage2 ]; then PHASES="coverage"; else PHASES="prepare coverage"; fi
    echo "== 前置门禁 =="
    check_gates || { echo "== 门禁未过，先修（不要下发）=="; exit 1; }
    LOG="$WS/verification/$FLOW_DIR/logs/air_${CMD}_$(date +%Y%m%d_%H%M%S).jsonl"
    T0=$(date +%s)
    echo "== 下发：--phases $PHASES（$(date +%H:%M:%S)；证据 $LOG）=="
    python3 "$WS/verification/$FLOW_DIR/driver.py" \
      --tcp-host "${GCS_ROBOT_IP:-192.168.5.241}" --tcp-port 39001 \
      --phases $PHASES --log "$LOG"
    drv_rc=$?
    echo "== driver 判定（退出码 $drv_rc）=="
    driver_verdict "$LOG"
    sleep 3   # 等跟随环事件落盘
    echo "== 跟随环事件（各跟随者，本轮窗口起 $(date -d @$T0 +%H:%M:%S)）=="
    followers_events "$T0"
    cat <<'EOM'
  判读：跟随者应出现 follower_start → leader_odom_first_frame（wait_s ≲ 0.1 s）且**无** LEADER_LOST；
  出现 LEADER_LOST / "leader odom timed out" = 9/22 型故障复现 → 立刻跑 `air_flow.sh odom` 定层，
  并在收工后按 rosbag 各机对齐（发送侧/接收侧/哨兵话题）。
EOM
    [ "$drv_rc" -eq 0 ] && echo "== $CMD 通过 ==" || echo "== $CMD 未全通过（阶段终态不作为唯一判据，先看跟随环事件与 odom）=="
    exit "$drv_rc"
    ;;
  *)
    sed -n '2,30p' "$0"; exit 2
    ;;
esac

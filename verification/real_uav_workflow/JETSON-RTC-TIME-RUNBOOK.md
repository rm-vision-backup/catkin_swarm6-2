# Jetson 机载电脑「时间每次上电重置」排查与修复手册

> 适用范围：**达妙 DAMIAO Orin NX V2 载板 + NVIDIA Orin NX/Nano 模组**（L4T R35.6.1 / JetPack 6.x / Ubuntu 20.04）
> 已实战验证：**UAV12、UAV15** 两架，2026-10-18
> 本文档目标：新飞机出现同样症状时，**照着第 4、5、6 节做一遍即可**，不需要重新诊断。

---

## 0. 一句话结论

**时间重置不是电池坏了，是「错误时间被反复写回 RTC」的自我维持闭环。**
修复的关键动作只有一个：**把第一个正确时间写进 RTC**。电池绝大多数情况是好的。

---

## 1. 症状

| 观察项 | 异常表现 |
|---|---|
| `date` | 显示 `1970-01-01 08:0x`，且数值约等于「1970 + uptime」 |
| `timedatectl` | `System clock synchronized: no` / `NTP service: n/a` |
| 每次断电重启 | 时间又回到 1970，永不累积 |
| `last` 登录记录 | 每次重启都是 `Jan 1 08:00` —— **历史上从没有过一次正确时间** |
| `/var/log/syslog` | 常年不轮转、体积异常大（如 113 MB），因为 logrotate/anacron 靠时间戳判断 |

---

## 2. 硬件事实（排障必须知道）

### 2.1 这块板上有两个 RTC

| 设备 | 驱动 | 物理位置 | 是否带电池 |
|---|---|---|---|
| `rtc0` | `nvvrs-pseq-rtc` | I2C `4-003c`，VRS 电源时序器 | ✅ **有备电（CR1220）** |
| `rtc1` | `tegra_rtc` | `c2a0000.rtc`，SoC 内部 | ❌ 掉电即归零 |

```bash
cat /sys/class/rtc/rtc0/name    # nvvrs-pseq-rtc
cat /sys/class/rtc/rtc1/name    # tegra_rtc
```

### 2.2 默认 `/dev/rtc` 指向的是**没电池的那个**

Ubuntu 的 `/lib/udev/rules.d/50-udev-default.rules` 规则：

```
SUBSYSTEM=="rtc", ATTR{hctosys}=="1", SYMLINK+="rtc"
SUBSYSTEM=="rtc", KERNEL=="rtc0",     SYMLINK+="rtc", OPTIONS+="link_priority=-100"
```

内核编译时 `CONFIG_RTC_HCTOSYS` 指定的是 **tegra_rtc**，所以 `hctosys=1` 的是 rtc1，`/dev/rtc` 就指向了 rtc1；而 rtc0 被显式压成 `link_priority=-100` 垫底。

**后果**：直接敲 `hwclock -r` 读的是没电池的 rtc1，会误导判断。**读备份 RTC 必须显式 `-f /dev/rtc0`。**

### 2.3 开机关机的时间流（NVIDIA 自带脚本，不要改）

```
上电
 ├─ [4s ] 内核从 rtc1(无电池) 取时间 → 1970        ← 灾难起点
 ├─ [13s] nvrtc-sync-boot.service 启动
 └─ [15s] /etc/systemd/nvrtc-sync-boot.sh:
            hwclock --hctosys -f /dev/rtc0   ← 从备份RTC恢复系统时间
            hwclock --systohc -f /dev/rtc1   ← 再写回 tegra

关机
 └─ nvrtc-sync-shutdown.service 的 ExecStop:
            hwclock --systohc -f /dev/rtc0   ← 把系统时间写回备份RTC
```

验证启动确实执行了：

```bash
sudo journalctl -b -o short-monotonic | grep -E "setting system clock|nvrtc-sync-boot"
```

正常输出形如：

```
[    4.07] tegra_rtc c2a0000.rtc: setting system clock to 1970-01-01T00:00:23 UTC (23)
[   15.38] nvrtc-sync-boot.service: Succeeded.
```

> ⚠️ **开机后 0~15 秒内时间仍是 1970 是正常的**，不要在这个窗口里下结论。

### 2.4 其他环境事实

- `hwclock.service` 被 mask（NVIDIA 用 `nvrtc-sync-*` 替代）
- `systemd-timesyncd` 出厂是 **masked**
- `ntp` 出厂是 **enabled**，但配了一个**无效的**内网源 `server 192.168.5.51 prefer`
- `nvvrs` RTC 的有效范围是 **2000-01-01 ~ 2099**，写入 1970 会返回 `EINVAL`（这是驱动 `range_min` 限制，不是坏了）
- 设备树里的 `nvidia,p3768-0000+p3767-0000-super` 是**参考 DT**，第三方载板（达妙）也用它，**不能据此判断载板型号**
- SSH：`nvidia@192.168.55.1`（USB device mode 固定地址），默认密码 `nvidia`

---

## 3. 根因：自我维持闭环

```
出厂/刷机后 rtc0 从未被初始化（值 = 0）
        │
        ▼
开机 → rtc1 也是 0 → 系统时间 = 1970
        │
        ▼
关机 → nvrtc-sync-shutdown 把 1970 写回 rtc0     ★ 关键这一环
        │
        ▼
下次开机从 rtc0 读回 1970 …… 永远出不来
```

**电池在这里是从犯不是主犯**：它忠实地保存着一个错误的值。所以换不换电池结果都一样 —— 只要没人把第一个正确时间写进去，环就一直转。

### 为什么「电池有没有坏」无法从软件判断

因为关机脚本每次都会用（错误的）系统时间去覆盖 rtc0。无论电池好坏，rtc0 里都只会存着 1970。**唯一判定方法是：把时间设对 → 断电 → 看是否保持**（见第 6 节）。

---

## 4. 快速定位（5 分钟，4 条命令）

```bash
# 1) 当前时间与 uptime —— 若 date ≈ 1970 + uptime，基本确诊
date; uptime

# 2) 两个 RTC 的读数（注意显式 -f）
sudo hwclock -r -f /dev/rtc0      # 带电池的
sudo hwclock -r -f /dev/rtc1      # 无电池的

# 3) /dev/rtc 现在指向谁
readlink /dev/rtc

# 4) 开机时时间是从哪恢复的
sudo journalctl -b -o short-monotonic | grep -E "setting system clock|nvrtc-sync-boot"
```

### 判读表

| 现象 | 结论 | 下一步 |
|---|---|---|
| `date` = 1970 且 ≈ uptime | 确诊，RTC 里是 0 | 走第 5 节 |
| `date` 正确，`rtc0` 也正确 | 已修好 | 走第 6 节做断电验证 |
| `rtc0` 读得到、写入返回 `EINVAL`（写 1970） | 正常，驱动限制范围 2000+ | 别写 1970 |
| 写入任意值都失败 / `rtc0` 不存在 | RTC 硬件或 I2C 异常 | 查 `dmesg \| grep nvvrs` |
| 断电后 `date` 回到 1970 且 `rtc0` 读到 0 | **电池真坏了** | 换 CR1220（第 8 节）|

---

## 5. 标准修复流程（7 步）

> 全部命令按顺序执行。每步都有验证，不要跳步。

### 步骤 1：确认能连上

```bash
ssh nvidia@192.168.55.1
```

### 步骤 2：确认 RTC 芯片本身是好的（排除硬件）

```bash
# 写一个显眼的值，读回，看是否正常走秒
sudo hwclock --set --date "2024-06-18 12:00:00" -f /dev/rtc0
sudo hwclock -r -f /dev/rtc0          # 应读到 2024-06-18 12:00:00
sleep 3
sudo hwclock -r -f /dev/rtc0          # 应变成 12:00:03 左右
```

读回一致且在走秒 → 芯片没问题，问题在「没人设过时间」。

### 步骤 3：设置正确时间

**有外网时（首选，最准）：**

```bash
# 临时借公网 NTP 校一次（Ubuntu 20.04 自带 sntp）
sudo sntp -sS pool.ntp.org
date
```

**无外网时：** 从一台**已校准**的机器取 epoch（注意：Windows 宿主机可能不准，本手册实测某台 Windows 比真实时间快约 2 秒）

```powershell
# 在 Windows 上取 epoch
[int][DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
```

```bash
# 在飞机上
sudo date -u -s @<上面得到的epoch>
date
```

### 步骤 4：让 `/dev/rtc` 指向带电池的 rtc0

```bash
sudo tee /etc/udev/rules.d/99-rtc-prefer-rtc0.rules >/dev/null <<'EOF'
# Make the battery-backed RTC (nvvrs-pseq-rtc = rtc0) the default /dev/rtc.
# Ubuntu's 50-udev-default.rules points /dev/rtc at whichever RTC did hctosys
# (tegra_rtc = rtc1, no battery backup) and gives rtc0 link_priority=-100.
SUBSYSTEM=="rtc", KERNEL=="rtc0", SYMLINK+="rtc", OPTIONS+="link_priority=100"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=rtc
sleep 2
readlink /dev/rtc        # 必须是 rtc0
```

> 这一步只影响 `hwclock` 等工具的默认目标，**不影响**时间能否保持（`nvrtc-sync` 脚本用的是显式 `-f` 路径）。属于整理性改动。

### 步骤 5：处理 ntp（历史遗留，必删）

出厂配置里有 `server 192.168.5.51 prefer`，那个源回的是**废包**（`LI=3 / stratum=0 / refID=INIT / Originate=0`），ntpd 永远 `reach=0`，只会刷 DNS 失败日志。

**推荐：连包一起卸载**（需要外网，因为 apt 要下载 `systemd-timesyncd` 补 `time-daemon` 依赖）

```bash
# 先修可能损坏的 apt 索引（常见故障：Unable to parse ... i18n_Translation-en）
sudo rm -f /var/lib/apt/lists/*Translation-en
sudo rm -rf /var/lib/apt/lists/partial
sudo apt-get update

# 卸掉 mask 残留，再 purge
sudo rm -f /etc/systemd/system/ntp.service
sudo apt-get -y purge ntp

# 清残留
sudo rm -rf /etc/ntp.conf /var/lib/ntp /etc/init.d/ntp \
            /etc/dhcp/dhclient-exit-hooks.d/ntp /run/ntp.conf.dhcp \
            /etc/ntp.conf.bak-*
sudo systemctl daemon-reload && sudo systemctl reset-failed
```

**如果暂时没外网**，用等效的最小处理：

```bash
sudo cp -a /etc/ntp.conf /etc/ntp.conf.bak-$(date +%Y%m%d-%H%M%S)
sudo sed -i '/^[[:space:]]*server[[:space:]]\+192\.168\.5\.51\b/d' /etc/ntp.conf
sudo systemctl stop ntp && sudo systemctl disable ntp && sudo systemctl mask ntp
```

### 步骤 6：启用 systemd-timesyncd 做长期校准（有外网时）

```bash
sudo systemctl unmask systemd-timesyncd
sudo systemctl enable --now systemd-timesyncd
sleep 6
timedatectl | grep -E "synchronized|NTP service"
systemctl status systemd-timesyncd --no-pager | grep Status
```

正常应看到：

```
System clock synchronized: yes
     NTP service: active
   Status: "Initial synchronization to time server x.x.x.x:123 (0.pool.ntp.org)."
```

> 无外网的飞机跳过这步。此时时间只靠晶振，典型漂移 **±20 ppm ≈ 每天 1.7 秒**，一个月约 1 分钟。长期精度需求请上 GNSS/PTP（载板有 SYNC IO / PPS 引脚）。

### 步骤 7：把正确时间回写两个 RTC

```bash
sudo hwclock --systohc -f /dev/rtc0
sudo hwclock --systohc -f /dev/rtc1

date
sudo hwclock -r -f /dev/rtc0     # 应与 date 一致
sudo hwclock -r -f /dev/rtc1
readlink /dev/rtc                # rtc0
```

---

## 6. 必须做的验证：断电测试

**软件层面看不出电池好坏，只有断电才能验证。**

```bash
# 记录断前基准
date "+%Y-%m-%d %H:%M:%S   epoch=%s"
sudo hwclock -r -f /dev/rtc0
```

然后**物理断电 2 分钟以上**，再上电，重新 SSH 进来：

```bash
date; uptime
sudo hwclock -r -f /dev/rtc0
sudo journalctl -b -o short-monotonic | grep -E "setting system clock|nvrtc-sync-boot"
```

### 判读

| 结果 | 结论 |
|---|---|
| 时间 ≈ 断电时刻 + 断电时长（误差 < 2 秒） | ✅ **修复成功，电池也是好的** |
| 时间回到 1970，且 `rtc0` 读到 0 | ❌ **电池真的坏了**，换 CR1220 |
| 时间正确但差了几十秒 | 电池电压偏低或接触不良，重新插一下电池 |

### 实战参考数据（2026-10-18）

| 飞机 | 真实经过 | rtc0 增加 | 丢失 | 结论 |
|---|---|---|---|---|
| UAV15 第 1 次 | 97 s | 90 s | ~7 s | 电池刚重插，首次热循环一次性损失 |
| UAV15 第 2 次 | 482 s | 483 s | **0.8 s** | 噪声级，正常 |
| UAV12 | 224 s | 225 s | **0.7 s** | 噪声级，正常 |

> UAV15 第 1 次的 7 秒曾被误判为「电池失效」，实际是电池重新插装后接触未稳定。**单次异常不要急着下结论，再做一次。**

---

## 7. 一键脚本（整段复制到飞机上执行）

```bash
#!/bin/bash
# Jetson 时间修复一键脚本
# 用法: sudo bash fix-rtc-time.sh            # 自动尝试用公网 NTP 校时
#       sudo bash fix-rtc-time.sh 1792310000 # 无外网时手动给 epoch
set -e

EPOCH="$1"

echo "=== 1. 现状 ==="
date; uptime
readlink /dev/rtc
hwclock -r -f /dev/rtc0 2>/dev/null

echo "=== 2. 校时 ==="
if sntp -sS -t 8 pool.ntp.org >/dev/null 2>&1; then
    echo "   已用 pool.ntp.org 校准"
elif [ -n "$EPOCH" ]; then
    date -u -s "@$EPOCH"
    echo "   已用指定 epoch 校准"
else
    echo "   !! 无法校时（无外网且未给 epoch），时间可能不准，继续配置"
fi
date

echo "=== 3. /dev/rtc -> rtc0 ==="
cat > /etc/udev/rules.d/99-rtc-prefer-rtc0.rules <<'EOF'
SUBSYSTEM=="rtc", KERNEL=="rtc0", SYMLINK+="rtc", OPTIONS+="link_priority=100"
EOF
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=rtc
sleep 2
readlink /dev/rtc

echo "=== 4. 清理 ntp ==="
rm -f /etc/systemd/system/ntp.service
if command -v apt-get >/dev/null && apt-get -y purge ntp >/dev/null 2>&1; then
    echo "   ntp 已卸载"
else
    echo "   purge 失败（可能无外网），改用停用"
    sed -i '/^[[:space:]]*server[[:space:]]\+192\.168\.5\.51\b/d' /etc/ntp.conf 2>/dev/null || true
    systemctl stop ntp 2>/dev/null || true
    systemctl disable ntp 2>/dev/null || true
    systemctl mask ntp 2>/dev/null || true
fi
rm -rf /etc/ntp.conf /var/lib/ntp /etc/init.d/ntp \
       /etc/dhcp/dhclient-exit-hooks.d/ntp /run/ntp.conf.dhcp
systemctl daemon-reload; systemctl reset-failed

echo "=== 5. 长期校准（有外网才有用） ==="
systemctl unmask systemd-timesyncd 2>/dev/null || true
systemctl enable --now systemd-timesyncd 2>/dev/null || true
sleep 6
timedatectl | grep -E "synchronized|NTP service" || true

echo "=== 6. 回写 RTC ==="
hwclock --systohc -f /dev/rtc0
hwclock --systohc -f /dev/rtc1

echo "=== 7. 结果 ==="
date
hwclock -r -f /dev/rtc0
hwclock -r -f /dev/rtc1
readlink /dev/rtc
echo
echo ">>> 现在请【物理断电 2 分钟以上】再上电，然后重新执行本命令的 1) 段验证。"
```

---

## 8. 电池：什么时候才需要换

| 情况 | 处理 |
|---|---|
| 断电测试后时间保持（误差 < 2 秒）| **不换**，电池是好的 |
| 断电后回到 1970，且 `rtc0` 读到 0 | 换电池 |
| 时间在走但明显偏慢/断续 | 检查电池接触，重新插一次 |

**电池规格**：`CR1220`（不可充锂锰，3.0 V，容量约 35–40 mAh）

**寿命估算**：RTC 待机电流典型 1–2 µA → **约 2–4 年**（高温环境明显缩短）

**更换注意**：
- 丝印写 `CR1220` 就是**非充电型**，不要换成可充的 ML2032（除非确认板子有充电回路）
- 换完电池后 RTC 会归零，**必须重新走一遍第 5 节的设时间流程**
- 判断电池是否该换：万用表量空载电压，**低于 2.5 V 就该换**（新电池约 3.0–3.2 V）

---

## 9. 常见误判（都是实际踩过的）

| 坑 | 说明 |
|---|---|
| ❌ 用「开机后时间是 1970」判断电池坏 | 只要系统时间本身是错的，关机脚本就会把 1970 固化进 RTC。**不能作为判据** |
| ❌ 用 `hwclock -r` 判断备份 RTC | 默认读的是**没电池的** rtc1，必须 `-f /dev/rtc0` |
| ❌ 在开机 15 秒内下结论 | `nvrtc-sync-boot` 要到 ~15 s 才从 rtc0 恢复时间 |
| ❌ 按设备树 `p3768` 判断载板型号 | 那是参考 DT，达妙等第三方载板也用它 |
| ❌ 相信 `dmesg` 里的 `nvvrs_pseq: CAUTION: interrupt status reg:0x10 set to 0x8` | 那是 **RTC 闹钟中断**（REG_INT_SRC1 bit3），**不是电池告警** |
| ❌ 把 1970 写进 rtc0 | 驱动 `range_min = 2000`，会返回 `EINVAL`，别误判成硬件坏 |
| ❌ 以为宿主机时间是准的 | 实测某台 Windows 比真实 NTP 时间**快约 2 秒**，有外网优先用 NTP |
| ❌ 直接 `apt-get purge ntp` 报错就以为坏了 | 常见两个前置故障：① apt 索引损坏（删 `*Translation-en` 再 update）② 需要外网下载 `systemd-timesyncd` 补 `time-daemon` 依赖 |

---

## 10. 每架飞机的处理记录（复制一份填）

```markdown
### UAV__
- 日期：
- 处理前：date =              rtc0 =              /dev/rtc ->
- RTC 芯片读写测试：通过 / 失败
- 校时来源：NTP pool.ntp.org / 宿主机 epoch / 其他
- 处理动作：☐设置时间 ☐udev规则 ☐删ntp配置 ☐purge ntp ☐启用timesyncd
- 断电测试：断前 =           断后 =           丢失 =      秒
- 结论：☐电池正常 ☐电池更换
- 备注：
```

### 已完成记录

| 飞机 | 载板 | 电池 | 根因 | 处理 | 断电测试 | 状态 |
|---|---|---|---|---|---|---|
| UAV15 | 达妙 Orin NX V2 | CR1220（好，曾重插）| 自我维持闭环 | 设时间 + udev + purge ntp + timesyncd | 482 s 丢 0.8 s | ✅ |
| UAV12 | 达妙 Orin NX V2 | CR1220（好）| 自我维持闭环 | 同上 | 224 s 丢 0.7 s | ✅ |

---

## 11. 修复后仍然存在的限制

1. **开机 0~15 秒仍是 1970** —— 内核先读无电池的 tegra RTC，`nvrtc-sync-boot` 到 ~15 s 才校正。若这段时间的日志时间戳很重要，可另加一个更早期的 systemd unit（把窗口压到 2~3 s）。
2. **无外网的飞机没有任何时间源** —— 只靠晶振，漂移 ±20 ppm（约每天 1.7 秒）。要精度请上 GNSS/PTP。
3. **`/var/log/syslog` 可能长期未轮转** —— 坏时钟会连带搞乱 anacron/logrotate 的调度判断。时间修好后通常自行恢复，若仍不轮转需单独排查。

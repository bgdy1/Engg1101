# ENGG1101 机械臂项目：Agent 开发指南

本文是当前工作区的项目背景与开发入口，适用于其下所有目录。当前含五个原有操作脚本与新增的 06 手动摆位录制入口，也供后续进一步融合时参考。

## 1. 使用本文的方式

- 开始任务时先读本文；不需要为了重新了解背景而遍历全部源码或 SDK。
- 按第 10 节定位本次任务涉及的模块，只阅读准备修改的函数、直接依赖和相关测试。修改前仍应核实这些局部实现，本文不能代替代码事实。
- 下文“当前行为”是源码事实；“融合建议”是后续设计参考，不代表已经实现。06 已按用户要求实现，其他进一步融合的入口与功能仍需按任务确定。
- 修改行为、配置、动作格式、入口或测试命令后，同步更新本文对应内容，避免背景过期。不要记录未经验证的硬件参数或测试成功结论。
- 用户主要使用中文沟通；保留现有英文标识符和文件名。未经任务要求，不批量改写动作数据、标定参数或厂商 SDK。

## 2. 项目定位、环境与边界

- 工作区目录为 `D:\Projects\ENGG1101`；当前源码位于其子目录 `STS3215-5DOF-Robot-Arm-main/`。下文路径除特别说明均相对于该源码目录。
- 原始上游：https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm 。当前项目仓库为 https://github.com/bgdy1/Engg1101 ，Git 根目录是 `D:\Projects\ENGG1101`，远端 `origin` 指向 `https://github.com/bgdy1/Engg1101.git`，分支为 `main`，基于远端原有历史建立。不要在源码子目录另建仓库。
- 功能：读取五舵机位置、原始刻度控制、角度控制、手动拖动示教录制/按时间回放、人工输入关节路点/逐点回放、单键记录实物五轴角度路点。
- 硬件链路：树莓派 Python → USB 串口 → Waveshare ESP32 控制板（需配置转发模式）→ STS3215 舵机总线；舵机由外部电源供电。
- 仅包含上位机 Python 代码和随附舵机 SDK，不包含 ESP32 固件、CAD/STL、URDF、连杆尺寸、机械装配或负载验证。
- “五自由度”在本代码中是底座、肩、肘、腕、夹爪五个控制通道；夹爪是开合通道，不能据此声称能独立控制末端六维位姿。
- 本仓库没有正/逆运动学、XYZ 控制、碰撞检测、视觉识别或末端直线轨迹规划。README 指向另一个 `robot-arm-simulation` 仓库；其标定不会自动与本仓库共享。
- 设计运行环境是树莓派/Linux 交互终端。当前工作区宿主是 Windows PowerShell，原生 Windows 存在下述终端兼容问题。

## 3. 完整目录与职责

```text
ENGG1101/
├── .git/                            本地 Git 元数据，不上传
├── .gitignore                       忽略虚拟环境、缓存、临时文件及本地环境配置
├── .gitattributes                   Python/Markdown/JSON 等文本以 LF 规范化
├── README.md                        项目入口、克隆说明及版本同步流程
├── hello.txt                        保留远端仓库原有文件
├── AGENTS.md                         本文，覆盖整个工作区
└── STS3215-5DOF-Robot-Arm-main/
    ├── .gitignore                    忽略 .venv、Python 缓存、config/robot.json
    ├── README.md                     树莓派安装、连接、六个练习及关联仿真说明
    ├── requirements.txt              pyserial>=3.5,<4
    ├── arm_settings.py               串口、关节映射、标定、限位、运动参数
    ├── arm_helpers.py                通信与转换公共函数
    ├── terminal_keys.py              POSIX 终端即时键盘读取 RawTerminal
    ├── scripts/
    │   ├── 01_read_positions.py       释放扭矩，单次读取五关节
    │   ├── 02_move_servo.py           输入五个原始目标，单次同步运动
    │   ├── 03_move_joint_angle.py     输入五个角度，转换后单次同步运动
    │   ├── 04_timestamp_motion.py     拖动示教录制与按时间回放
    │   ├── 05_waypoint_sequence.py    人工录入路点与等待完成的逐点回放
    │   └── 06_waypoint_recording.py   释放扭矩，q 记录五轴角度，e 保存供 05 播放
    ├── motions/
    │   ├── .gitkeep                   保留动作目录的占位文件
    │   ├── 1.json                    10 点角度路点，内部名称 1
    │   ├── 2.json                    9 点角度路点，内部名称 2
    │   ├── 3.json                    11 点角度路点，内部名称 3
    │   ├── 4.json                    4 点角度路点，内部名称 attack01
    │   └── 5.json                    30 点角度路点，内部名称 endgame，拼接 1/2/3
    ├── scservo_sdk/
    │   ├── __init__.py               导出 SDK 类、常量和函数
    │   ├── port_handler.py           pyserial 串口封装、收发与包超时
    │   ├── protocol_packet_handler.py 协议封包、校验、寄存器读写、广播
    │   ├── scservo_def.py            指令、广播 ID、通信结果常量
    │   ├── sms_sts.py                本项目使用的 SMS/STS 寄存器与位置接口
    │   ├── scscl.py                  另一舵机系列接口，当前业务未使用
    │   ├── group_sync_write.py       多舵机同步写入参数与发送
    │   └── group_sync_read.py        多舵机同步读取支持，当前 read_all 未使用
    └── tests/
        ├── test_arm_helpers.py       位置显示、输入解析、同步下发、扭矩接口
        ├── test_pose_sequence.py     路点解析、存储兼容、菜单保存、回放参数、类型筛选
        └── test_waypoint_recording.py 06 按键录制、05 加载兼容、异常保存与清理
```

`config/robot.json` 只是忽略规则，当前目录没有该文件，现有程序也不以它作为配置源。业务配置源是 `arm_settings.py`。

## 4. 公共数据、配置与底层行为

### 4.1 固定顺序及当前默认值

所有五值输入按 `settings.JOINT_NAMES` 排列，不依赖用户随意选择字典顺序：

| 名称 | ID | ZERO_TICKS | DIRECTIONS | ANGLE_LIMITS（度） |
| --- | --- | --- | --- | --- |
| base_yaw | 1 | 2048 | +1 | [-90, 90] |
| shoulder | 2 | 2048 | +1 | [-90, 90] |
| elbow | 3 | 2048 | +1 | [-135, 135] |
| wrist | 4 | 2048 | +1 | [-135, 135] |
| gripper | 5 | 2048 | -1 | [-90, 90] |

- `SERIAL_PORT="/dev/ttyUSB0"`，`BAUD_RATE=115200`，`TICKS_PER_REVOLUTION=4096`。
- 单次移动：`MOVE_SPEED=3000`、`MOVE_ACCELERATION=254`。
- 路点回放：`POSE_SEQUENCE_MOVE_SPEED=3000`、`POSE_SEQUENCE_MOVE_ACCELERATION=254`。
- 时间回放：`TIMESTAMP_MOVE_SPEED=800`、`TIMESTAMP_MOVE_ACCELERATION=50`。
- 录制采样目标间隔：`RECORD_INTERVAL_SECONDS=0.1`；脚本 04 自身定义保存间隔 2 秒。
- 上述值是当前配置，不代表实物已完成标定。速度、加速度直接传入 SDK，勿把数值标为度/秒等未经确认的单位。

### 4.2 位置表示与换算

- 公共原始位置：`dict[str, int]`，正常业务包含五个关节名，值为 0～4095。
- 角度：度数，通常为 `dict[str, float]`；原始输入也可能是按关节顺序排列的字符串列表。
- 换算：`raw = round(ZERO_TICKS[name] + DIRECTIONS[name] * degrees * 4096 / 360)`。
- 反向：`degrees = (raw - ZERO_TICKS[name]) * 360 / (DIRECTIONS[name] * 4096)`。
- `angle_to_position` 检查关节名、角度限位与换算结果范围；`position_to_angle` 检查关节名与原始范围，不检查计算角度是否符合关节限位。
- 例：默认配置下角度 `[0, 10, -10, 20, 90]` 转为原始位置 `[2048, 2162, 1934, 2276, 1024]`。这是数值示例，不是推荐实物姿态。
- 原始模式不会自动应用 `ANGLE_LIMITS`；合法编码范围不等于机械机构可用范围。夹爪角度不等于开口毫米数，也不能仅凭正负确定开合方向。

### 4.3 arm_helpers.py 接口速查

| 函数 | 返回/副作用 |
| --- | --- |
| `connect()` | 打开配置串口，设置波特率，返回 `(port, sms_sts_packet_handler)`；可能抛异常 |
| `check_result(handler, result, error, action)` | 通信失败或舵机错误时抛 `RuntimeError` |
| `disable_all_torque(handler)` | 按配置 ID 逐个写扭矩 0；汇总已返回的错误并抛异常 |
| `enable_all_torque(handler)` | 按配置 ID 逐个写扭矩 1；汇总已返回的错误并抛异常 |
| `read_servo(handler, servo_id)` | 读取一个舵机原始位置，检查通信结果 |
| `read_all(handler)` | 顺序读取五个舵机，返回关节名到原始位置的字典，并非同时采样 |
| `move_all_sync(handler, positions, speed=None, acceleration=None)` | 为所有配置关节调用下一个函数；不使能扭矩、不等待完成 |
| `move_positions_sync(handler, positions, joint_names, speed=None, acceleration=None)` | 指定关节组同步下发；缺省使用 MOVE 参数；`finally` 清空同步参数 |
| `wait_for_servos_or_estop(handler, servo_ids, terminal)` | 全部报告不在运动时返回 `True`；空格释放扭矩并返回 `False`；通信错误抛异常 |
| `angle_to_position(name, angle_degrees)` | 角度转换为原始位置 |
| `position_to_angle(name, position)` | 原始位置转换为角度 |
| `positions_to_angles(positions)` | 批量转换五关节 |

等待函数每轮先 `terminal.read_key(0.02)`，再顺序 `ReadMoving`，仍有舵机运动时再 `sleep(0.02)`；实际周期包含通信耗时，不是固定 20ms。没有整段运动超时，也没有实际位置误差判断。

### 4.4 SDK 和终端边界

- `sms_sts.SyncWritePosEx(id, position, speed, acc)` 只向组中添加参数；`groupSyncWrite.txPacket()` 才实际发送。
- 同步写入从寄存器 41 开始，连续 7 字节：加速度、目标位置低/高字节、时间 0/0、速度低/高字节。不包含地址 40 的扭矩使能。
- 其他常用地址：当前位置 56～57、运动状态 66。广播 ID 为 `0xFE`，同步写指令为 `0x83`，协议使用 `FF FF` 包头和校验和。
- 同步广播发送成功不等于所有舵机接收成功；同步下发也不保证同时到达，没有多关节统一到达时间规划。
- `PortHandler.openPort()` 先以 SDK 缺省 1Mbps 打开，再由 `connect()` 设置为配置的 115200；这不等于修改舵机自身的波特率寄存器。控制板转发与实际总线参数需正确配置。
- `RawTerminal` 是上下文管理器：检查 stdin 是终端、保存设置、`tty.setraw()`，退出时恢复；`read_key(timeout)` 通过 `select` 和 `os.read` 返回一个字符或 `None`。
- `terminal_keys.py` 无条件导入 `termios`/`tty`；Windows 原生 Python 无法直接导入。04 的录制还使用 `select` 监听 stdin，也要单独考虑跨平台适配。
- raw 模式下 Ctrl+C 可能只是字符 `\x03`；当前业务只显式处理空格，不应把 `KeyboardInterrupt` 捕获当作可靠的播放停止机制。

## 5. 六个脚本详解：当前行为

六个脚本都是独立 CLI，使用 `sys.path.insert` 导入源码根目录模块，均以 `if __name__ == "__main__"` 启动。数字前缀文件名不适合普通 `import` 语法；现有测试用 `runpy.run_path(..., run_name=非__main__)` 加载函数，不执行主菜单。

### 5.1 01_read_positions.py：单次读取

- **输入**：无数值输入，依赖配置串口、ID、标定。
- **函数**：`format_positions(raw_positions)` 返回 `(name, servo_id, raw, angle)` 元组列表；`main()` 管理连接和输出。
- **流程**：`connect → disable_all_torque → read_all → format_positions → 打印五关节 → closePort`。
- **输出**：每个舵机的 ID、名称、原始刻度、相对标定零位的角度；不写文件、不连续监视。
- **状态**：读取之前主动释放扭矩，读取结束不恢复；机械臂可能受重力移动。读取位置本身并不要求关闭扭矩，这是当前脚本的业务选择。
- **异常**：捕获 `Exception`，输出排障提示并以 `SystemExit(1)` 退出；已连接则在 `finally` 关闭串口。
- **融合复用点**：读取服务与显示格式可分离；整合时不能因用户选择“查看位置”就无意关闭正在保持姿态的舵机。

### 5.2 02_move_servo.py：原始位置单次移动

- **输入**：一行五个整数，空格分隔；`parse_positions(raw_values)` 返回五关节原始位置字典。
- **校验**：数量必须等于关节数；逐项 `int` 解析，范围 0～4095；不检查机械角度限位。
- **流程**：显示关节顺序 → 输入/解析 → 打印目标 → Enter 确认 → 连接 → `move_all_sync` → 在 `RawTerminal` 中等待完成/空格停止 → 关闭串口。
- **参数**：使用公共函数默认的 `MOVE_SPEED/MOVE_ACCELERATION`。
- **状态缺口**：当前没有调用 `enable_all_torque`。若先执行 01、04 录制或空格停止，扭矩仍关闭时下发位置不会自动恢复使能。
- **异常/退出**：`finally` 关闭串口；解析/通信异常不在主流程内统一恢复。正常结束不释放扭矩；先发运动再进入即时键盘模式。
- **融合复用点**：原始输入解析与单次运动执行分离；执行层应与 03 共用，明确接管扭矩状态。

### 5.3 03_move_joint_angle.py：角度单次移动

- **输入**：一行五个角度，可有小数；`parse_angles(raw_values)` 返回已换算的五关节原始位置字典。
- **函数**：`format_joint_limits()` 生成每关节 ID/名称/允许角度范围的显示文本。
- **校验**：数量 → `float` → `math.isfinite` → `angle_to_position` 的关节限位和原始范围。
- **流程**：显示限位 → 输入/转换 → 打印原始目标 → Enter 确认 → 连接 → 同步下发 → 等待完成/空格停止 → 关闭串口。
- **参数/状态**：与 02 相同，使用 MOVE 参数，没有主动使能扭矩，正常结束不释放扭矩。
- **融合复用点**：02 与 03 的区别主要在输入单位、校验及显示；转换后都是相同原始目标，可共用执行路径。转换不包含逆运动学。

### 5.4 04_timestamp_motion.py：拖动示教与按时间回放

**菜单**：`1` 录制、`2` 播放、`q` 退出，执行后返回菜单。

**录制 `record_motion()`：**

1. 提示支撑机械臂，Enter 后连接并关闭全部扭矩；读取一次初始位置，附时间和角度，供用户确认。
2. `confirm(prompt)` 的实际规则是仅输入 `q` 取消，其他输入均继续。输入名称和描述，名称正则与 05 相同。
3. 同名文件通常拒绝复用；仅当已有文件名称匹配、`steps` 为列表且为空时允许使用。初始位置会在新录制中重建，不是续录非空动作。
4. 再次确认后创建动作对象，先保存一次；以 `time.monotonic()` 计算采样经过时间，循环 `read_all → 转角度 → append step`。
5. 每步保存墙钟 ISO 时间、经过秒数、五关节原始位置与角度、名义采样间隔；`positions` 和 `positions_raw` 保存相同内容用于兼容。
6. 每约 2 秒重写整个 JSON，采样循环用 `sleep(max(0, 目标间隔-本轮耗时))`，并非硬实时 10Hz；五个关节依次读取。
7. 输入 `q` 加 Enter 或收到 `KeyboardInterrupt` 后停止；采样段 `finally` 再保存，外层 `finally` 关闭串口。录制结束/取消不会恢复扭矩。

**录制辅助函数：** `timestamp_now()` 输出带本地时区、毫秒的 ISO 时间；`save_motion()` 先写 `.json.tmp` 再 `os.replace`；`stop_requested()` 用 `select` 非阻塞检测一行 `q`。

**选择/加载：**

- `find_motion_files()` 按文件名排序，排除 `type=pose_sequence` 和 `type=cartesian_path`；无法解析的文件仍列为候选，不能把它当成严格格式验证器。没有实现 cartesian_path 播放功能。
- `choose_motion(files)` 支持菜单序号或文件 stem；数字输入先按序号尝试。`q` 取消，其他未匹配值抛 `ValueError`。
- `load_motion(path)` 要求 JSON `name == path.stem`，且 `steps` 是非空列表；可选的初始位置排在首个播放目标，`elapsed_seconds=None`。
- 每步优先取 `positions_raw`，缺失时回退 `positions`；`validate_positions` 要求键集合恰好为全部五个关节，值转 `int` 后检查 0～4095，不重新检查角度限位。
- 步时间转 `float` 后只检查不小于 0；采样间隔默认 0.1，亦只检查不小于 0。当前未完整拒绝 NaN/Infinity，也未要求时间严格递增；数值型小数原始位置会被 `int` 截断，勿误写为严格整数 JSON 校验。
- 返回 `(motion_metadata, poses, interval)`；每个 `pose` 为 `{"positions": raw_dict, "elapsed_seconds": float_or_None}`。

**播放 `play_motion()`：**

1. 选择并完整加载动作，显示说明并等 Enter；连接后主动 `enable_all_torque`，进入 `RawTerminal`。
2. 第一个目标立即发送。之后若当前/上一个目标都有经过时间，等待 `max(0, 当前elapsed-上一elapsed)`；否则等待文件采样间隔。用 monotonic deadline 分段等待并检查空格。
3. 按 `TIMESTAMP_MOVE_SPEED/TIMESTAMP_MOVE_ACCELERATION` 下发各样本。不会为每个样本调用 `ReadMoving`，也不等待初始姿态先到位。
4. 回放使用原始位置，不使用保存的角度或每步 `duration_seconds` 控制运动。相对等待包含额外发送/输出耗时，可能累计时间漂移；没有插值/轨迹拟合。
5. 空格关闭扭矩并结束播放；正常结束和异常后的关闭串口不等于关闭扭矩。最后样本下发后不会等待最后运动完成，打印 finished 不代表实物已到位。

**融合复用点**：保留“按时间调度”的独立策略，不能直接替换成 05 的逐点等待而声称保持示教节奏。录制、文件解析、调度、交互和串口生命周期目前耦合，需要逐步拆分。

### 5.5 05_waypoint_sequence.py：人工录入与逐点回放

**菜单**：`1` 人工输入并保存序列、`2` 选择已保存序列、`q` 退出。创建本身不连接硬件、不读取位置，不是拖动示教。

**解析函数：**

- `parse_point(raw_values, input_type)` 对一行文本 `split()` 后调用 `parse_point_values(values, input_type)`。
- `parse_point_values` 要求五值，按当前 JOINT_NAMES 配对；`raw` 用 `int` 检查 0～4095；`angles` 用 `float`、有限性检查和 `angle_to_position`；返回原始位置字典。
- raw 模式正常命令行字符串小数会失败；人工修改 JSON 为数值型小数时可能被 `int` 截断。输入类型只允许 raw/angles。

**创建 `create_sequence()`：**

1. 名称匹配 `^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$`；同名文件存在就拒绝覆盖；输入描述和模式（`1=raw`、`2=angles`）。
2. 显示关节顺序，角度模式额外显示限位；每行输入一个点，解析失败可重新输入该点，不终止整个录入。
3. 解析后的 raw 只用于校验和显示；实际追加 `{"input_values": raw_values.split()}`，保留原始字符串，不保存这次换算结果。
4. `play`/`p` 结束并请求保存后播放；`q` 结束并保存后返回菜单。命令不区分大小写；少于两点则不保存。
5. 构造元数据和 points，`save_sequence` 先写临时文件再替换，每个点单独一行；没有逐点自动保存、撤销或编辑功能。
6. 需要播放时返回文件路径，否则返回 `None`，由 main 决定是否进入播放。名称/模式错误会抛异常，没有像逐点解析那样重试。

**发现/加载：**

- `find_sequences()` 扫描并按文件名排序，仅列出字典且 `type=pose_sequence` 的 JSON；读取/语法错误跳过，但不会在列表阶段完整验证 points。
- `load_sequence(path)` 验证根类型、type、至少两点、input_type，每点为对象；全部解析完后返回 `(sequence_metadata, checked_raw_points)`。加载完成前不连接硬件。
- 点的 `input_values` 非 None 时按当前标定解析；否则 `_load_legacy_positions` 读取旧 `positions_raw` 字典，要求完整五关节并检查原始范围。
- 不要求内部 name 与文件 stem 一致；4.json/attack01 和 5.json/endgame 都合法。菜单显示文件 stem、描述、点数，选择方式仅菜单序号或 q。
- `joint_order`、`schema_version` 是保存的元数据，当前没有据此重排、校验配置或做版本分派。改变当前关节顺序/ID 会影响执行，不能假定文件内元数据提供保护。

**播放 `play_sequence(path)`：**

1. 全量加载校验 → 显示信息并 Enter 确认 → 连接 → 主动使能全部扭矩 → 进入 RawTerminal。
2. 每点前检测空格；调用 `move_all_sync`，明确传入 POSE_SEQUENCE 参数；随后 `wait_for_servos_or_estop` 等全部 Moving 为 0。
3. 等待返回 True 才发送下一点；False 立即结束本次播放。空格是释放扭矩并终止，不是暂停/继续。
4. 从当前实际姿态直接移动到文件第一点；不会检查已处于初始姿态，不做自动回零。每点都发送五个关节；只改夹爪时应保留其余四个目标。
5. 没有每点时长/停留、轨迹融合、插值、末端直线控制；重复同一点不能表示可靠的定时停留。
6. 正常完成后恢复终端、关闭串口，但不关闭扭矩；捕获 KeyboardInterrupt 仅打印提示，其他通信/文件错误可能向外传播。进入 raw 模式后不能依赖 Ctrl+C。

**融合复用点**：保留“等待完成再推进”的策略；与 04 共用位置下发、文件目录和停止处理，但不要混淆调度机制或文件验证规则。

### 5.6 06_waypoint_recording.py：单键记录实物角度路点

- **流程**：提示支撑机械臂并 Enter → `connect` → `disable_all_torque` → 输入未使用的文件名 → `RawTerminal` 单键录制 → 保存 → 恢复终端并关闭串口。全程不使能扭矩、不发送运动目标、不自动播放；退出保持释放状态。
- **名称**：沿用 05 的正则和 `motions/` 目录；非法名称/重名会重试，不增加描述输入，description 为空。目录基于脚本位置解析，不依赖启动工作目录。
- **q**：调用 `read_all → positions_to_angles`，再逐轴用 `angle_to_position` 校验当前限位；全部成功才追加一个点。超限/数值范围错误提示关节并拒绝该点，可调整后重试；通信异常终止本次录制。五轴顺序读取，并非同时采样。
- **e**：至少两个点才保存退出；不足两个点提示并继续。q/e 均无需 Enter，其他普通键忽略。
- **文件**：05 的 schema_version=1、type=pose_sequence、input_type=angles，包含 name、空 description、joint_order、created_at 和 points。每点仅存 `input_values`，按 JOINT_NAMES 排列为角度数字字符串，保留转换精度，屏幕仅显示两位小数。05 无需修改即可发现并加载，回放按当时配置重新换算。
- **保存/异常**：同目录唯一临时文件完成写入后 `os.replace`，保存前再次拒绝已有目标，失败清理临时文件并报错。Ctrl+C（含 raw 模式 `\x03`）或录制异常时尝试保存此前完整的至少两个点；不足两个点不写文件。通信异常在保存后仍报告失败；保存失败不打印成功。finally 关闭已取得的串口，RawTerminal 上下文恢复终端。
- **接口**：`choose_path`、`read_angle_point`、`save_sequence`、`record_points`、`record_sequence`、`main`；普通公共模块提供通信/转换，不动态加载 01/05。仅导入不连接硬件。运行：`python scripts/06_waypoint_recording.py`，要求树莓派/Linux 交互终端。

## 6. 动作文件契约与兼容性

### 6.1 路点文件（当前 motions/ 的五份文件均为此类型）

下例展示新格式的完整字段结构；仅用于理解数据，不代表实物可执行姿态：

```json
{
  "schema_version": 1,
  "type": "pose_sequence",
  "name": "demo",
  "description": "format example",
  "input_type": "angles",
  "joint_order": [
    {"servo_id": 1, "joint": "base_yaw"},
    {"servo_id": 2, "joint": "shoulder"},
    {"servo_id": 3, "joint": "elbow"},
    {"servo_id": 4, "joint": "wrist"},
    {"servo_id": 5, "joint": "gripper"}
  ],
  "created_at": "2026-11-16T18:00:00+08:00",
  "points": [
    {"input_values": ["0", "10", "-10", "20", "90"]},
    {"input_values": ["0", "10", "-10", "20", "30"]}
  ]
}
```

旧式点为 `{"positions_raw": {五个关节名及其原始位置}}`，仍可加载；没有 input_values 时才走此分支。角度新格式会随当前标定重新换算，raw/旧格式则保留原始目标。

### 6.2 时间录制文件（当前没有随附实例）

04 新录制不写 pose_sequence 那样的 type/schema_version；以 name、steps 等字段识别和校验：

| 字段路径 | 当前录制内容/加载用途 |
| --- | --- |
| `name` / `description` | 名称、描述；加载要求 name 等于文件 stem |
| `sampling_interval_seconds` | 名义采样间隔，缺省加载为 0.1 |
| `initial_position.timestamp` | 初始墙钟时间 |
| `initial_position.positions_raw` | 初始五关节原始位置；加载为首个目标 |
| `initial_position.angles_degrees` | 初始角度，回放不使用 |
| `steps[].timestamp` | 每步墙钟时间，调度不使用 |
| `steps[].elapsed_seconds` | 从采样开始的相对秒数，用相邻差值调度 |
| `steps[].positions_raw` | 五关节原始目标，加载优先采用 |
| `steps[].positions` | 与 positions_raw 内容相同的兼容字段 |
| `steps[].angles_degrees` | 用当时标定计算的角度，仅记录 |
| `steps[].duration_seconds` | 名义采样间隔，当前播放器不使用 |

两个格式共享 motions 目录但不能交给同一加载器盲读。时间加载器要求文件名/内部名匹配，路点加载器不要求；不要在融合时无意把一种约束施加到另一种数据上。

## 7. 五脚本融合时的约束与建议

06 已实现 01 的实物读数与 05 的角度路点格式组合；本节其余完整融合建议仍属后续设计参考，不自动授权新增产品功能。

| 能力 | 输入/数据 | 进入时的当前扭矩行为 | 完成条件 |
| --- | --- | --- | --- |
| 01 读取 | 配置 | 关闭 | 一次读取结束 |
| 02 原始移动 | 五个 raw | 不主动使能 | 全部 Moving=0 |
| 03 角度移动 | 五角度转 raw | 不主动使能 | 全部 Moving=0 |
| 04 录制 | 手动拖动 | 关闭 | q+Enter/中断，保存 |
| 04 时间播放 | 原始样本+时间 | 使能 | 最后目标已发送 |
| 05 创建 | 人工输入 | 无硬件连接 | 至少两点后保存 |
| 05 路点播放 | 角度或 raw 路点 | 使能 | 最后点全部 Moving=0 |
| 06 摆位录制 | q 读取实物五轴角度 | 关闭 | e 且至少两点后保存 |

融合建议（需按用户后续需求落地）：

- 将可复用函数放在普通命名模块中，新脚本作为交互入口；不要通过执行旧脚本主程序、子进程拼接或依赖数字文件名动态加载来共享业务。
- 分离四类职责：配置/串口会话、位置转换与校验、文件加载保存、交互与动作调度。02/03 共用原始目标执行，04/05 保留不同调度策略。
- 明确串口由哪层拥有、何时打开关闭，避免菜单功能之间重复占用；不要假定同一端口可与浏览器控制器并发使用。
- 明确扭矩状态转换，尤其是“读取/录制后再移动”和“停止后再次移动”；修复 02/03 的使能缺口时不要把释放扭矩作为所有操作的统一清理动作。
- 统一停止结果与错误传递，但区分完成、取消、通信失败、停止失败。释放扭矩可能导致下落，正常结束保持姿态与释放状态不是同一语义。
- 保留已有两种 JSON、旧路点兼容、名称差异及角度重新标定语义。需要迁移时提供明确兼容路径，不批量重写现有 motions。
- 文件输入校验和执行状态检查值得加强：严格整数/有限时间、元数据配置一致性、运动超时、到位误差、初始姿态与结束状态。它们是待改进项，当前没有实现。
- 没有硬件时先验证纯解析、调度和模拟通信；不要为验证菜单或文件格式而运行真实运动。新增脚本不应仅导入就打开串口或发送指令。

## 8. 已知问题与容易误解的行为

- 01 读取主动释放扭矩；04 录制取消/结束也不恢复。02/03 又不使能，因此跨功能切换会暴露状态缺口。
- `closePort()` 只关闭通信，不是停止/断电/扭矩关闭；多数 finally 只关闭串口。扭矩部分使能失败也没有统一回滚。
- 软件空格停止依赖串口与轮询；不是硬件急停，也不保持当前位置。SDK 收发异常可能提前终止逐个关闭流程。
- `ReadMoving=0` 只是状态判断，缺少目标误差校验；等待没有总超时。
- 04 不等待初始姿态或最后姿态完成，并有相对调度漂移；“Playback finished”不等于机械臂已停止。其空格分支有时仍打印 finished。
- 04 的“无录制文件”提示误写为用 script 05 录制；实际应使用 04，不能据此推断 05 有时间示教功能。
- 05 的 q 在录入阶段是保存退出，主菜单 q 才是退出程序；少于两点不保存。名称/文件错误与点输入错误的恢复方式不同。
- 两种播放器都无自动避障、自动回零、夹持力控制或硬件校准确认；不要把软件限位当作完整机械安全验证。
- 当前 Windows 不能直接使用 POSIX 键盘模块；改成 COM 端口仍不够。WSL 的 USB 接入也不能默认已配置。

## 9. 开发验证与当前验证记录

所有命令在源码目录执行；以下安装/测试说明不是已在当前环境完成安装的声明：

```sh
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

- 当前现有测试共 35 个（原有 18 个 + 06 新增 17 个），使用 unittest、Fake handler、Mock 和 runpy；不驱动实物。测试依赖源码目录作为工作目录，改入口时注意该假设。
- `test_arm_helpers.py`：01 的显示结果、02 数量/整数/范围、03 换算/限位/有限角度、同步目标与参数清理、扭矩开关写入。
- `test_pose_sequence.py`：路点参数选择、raw/angle 解析、最简新格式、旧 raw 格式、内部名称与文件名不同、play 命令保存后返回路径、时间列表排除路点。
- `test_waypoint_recording.py`：q/e 与最小点数、角度精度/夹爪反向及 05 加载、名称重试/拒绝覆盖、超限、读取中断不追加半点、两种 Ctrl+C、异常保存、扭矩关闭失败、保存失败、终端恢复/串口关闭及无导入硬件副作用。
- 当前测试没有全面覆盖 04 的采样/时间调度、异常清理、真实协议应答、运动到位或物理安全；通过这些测试也不能声称完成实物验证。
- **2026-11-16 已观察**：当前 Windows Python 3.13.9 缺少 serial；unittest 在两个测试模块导入阶段报 `ModuleNotFoundError: serial`，不能报告 18 项测试已通过。
- **同次静态检查**：18 个 Python 文件均可 AST 解析；5 份动作 JSON 均可读取，所有现有角度符合当时配置限位；Windows 的 termios 模块不存在。
- **2026-11-16 新增 06 后验证**：Windows Python 3.13.9 仍缺少 serial/termios，且 WSL 未安装。测试进程内为缺失的 serial、termios、tty 注入替身后，以 `unittest.defaultTestLoader.discover('tests')` 运行全部 35 项，均通过；serial.Serial 替身禁止任何实物访问。此结果验证业务逻辑与模拟通信/按键，不代表原生 Windows 可运行或真实 POSIX 终端、串口已验证。另对 20 个 Python 文件执行 AST 解析、5 份现有动作执行 JSON 解析，均成功；未修改现有动作。
- 未连接硬件、未验证实物零位/方向/电源/承载/急停。后续应重新核实环境状态，不把上述记录当成永久现状。
- 若实现融合，优先覆盖有意义的场景：读取/录制后移动需正确使能、停止不继续推进、两种调度不混淆、原有文件兼容、无导入硬件副作用、异常恢复终端并释放串口。用可控时钟/模拟串口，不靠真实 sleep 或实物来测试。

## 10. 按任务最小阅读范围

Git 同步约定：在工作区根目录管理整个项目；使用 `origin/main`，普通推送，不强制重写历史。本地已设置 `pull.ff=only`，其他设备可显式使用 `git pull --ff-only origin main`。修改前确认工作区状态并拉取；修改后审阅、commit、push；其他设备再拉取。用 `git fetch origin` 后的 `git rev-parse HEAD` / `git rev-parse origin/main` 与干净工作区确认版本一致。缓存/虚拟环境/临时文件不上传，新的 motions JSON 需提交才能同步。具体命令见根目录 README。

| 后续任务 | 优先检查 |
| --- | --- |
| 理解整体项目 | 本文即可建立背景；README 仅用于安装/操作细节 |
| 修改 ID/零位/方向/速度 | arm_settings.py 与受影响的转换/执行函数 |
| 合并读取与单次运动 | 01/02/03 的解析和 main、arm_helpers 的扭矩/运动/读取 |
| 修改拖动录制 | 04 的 record_motion、stop_requested、save_motion |
| 修改时间回放 | 04 的 load_motion、validate_positions、play_motion |
| 修改路点输入或文件格式 | 05 的 create/parse/save/load、test_pose_sequence |
| 修改路点播放 | 05 的 play_sequence、公共 wait/move、相关测试 |
| 修改单键摆位录制 | 06 的 read_angle_point/record_points/record_sequence/save_sequence、test_waypoint_recording，兼容性对照 05 的 load_sequence |
| 支持 Windows 或重做键盘输入 | terminal_keys.py、04 的 stdin select、各 raw 上下文调用点 |
| 排查真实串口/协议问题 | arm_helpers.connect/check_result，再按需进入 port_handler/protocol_packet_handler/sms_sts |
| 新统一入口 | 先按第 7 节明确交互与状态流，再只读需要提取的业务函数；无需重读整个 SDK |

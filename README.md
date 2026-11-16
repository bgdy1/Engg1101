# Engg1101
I want to use github in my engineering project

ENGG1101 五轴机械臂项目。代码源于
[Kelvinchan324/STS3215-5DOF-Robot-Arm](https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm)，
包含六个 Python 操作脚本，以及手动摆位、按 q 记录五轴角度的新录制入口。

## 项目结构与使用

- `STS3215-5DOF-Robot-Arm-main/`：Python 源码、SDK、动作文件及测试。
- [操作说明](STS3215-5DOF-Robot-Arm-main/README.md)：树莓派安装、接线和六个脚本的操作方式。
- [AGENTS.md](AGENTS.md)：开发背景、文件契约和验证记录。
- `hello.txt`：保留仓库原有文件。

树莓派/Linux 首次安装：

```sh
git clone https://github.com/bgdy1/Engg1101.git
cd Engg1101/STS3215-5DOF-Robot-Arm-main
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/06_waypoint_recording.py
```

运行 06 前支撑机械臂；Enter 后释放扭矩，输入名称，按 **q** 记录一个五轴角度点，
至少两个点后按 **e** 保存。随后运行 05，选择 **2** 播放保存的动作。

## 保持版本一致

Git 仓库根目录包含本文件、AGENTS.md 和源码子目录；无需在源码目录另建仓库。
Windows 当前工作区对应 `D:\Projects\ENGG1101`。
所有设备都从 [bgdy1/Engg1101](https://github.com/bgdy1/Engg1101) 克隆，使用 `main` 分支。

每次修改之前，先确认工作区干净，再更新：

```sh
git status --short --branch
git pull --ff-only origin main
```

完成修改后，在仓库根目录检查差异并提交、上传：

```sh
git diff
git add .
git diff --cached --stat
git commit -m "Describe the change"
git push origin main
```

然后在其他设备执行 `git pull --ff-only origin main`。若拉取因本地修改或分支分歧失败，
先保留并处理修改，不使用强制推送或丢弃文件来同步。

检查本机与远端提交一致：

```sh
git fetch origin
git status --short --branch
git rev-parse HEAD
git rev-parse origin/main
```

工作区无修改且两个提交 ID 相同，表示已与刚获取的远端版本对齐。
Git 不会自动上传编辑内容；修改和新录制的动作需要 commit、push 后才会共享。
虚拟环境、Python 缓存、临时文件、`.env` 和本地 `config/robot.json` 不纳入版本控制。
`arm_settings.py` 当前受版本控制，修改标定或串口设置时也要检查差异。

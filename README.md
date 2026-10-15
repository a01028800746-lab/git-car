# NEVC-SimOne 参赛代码

本仓库保存团队当前的 Python 算法、自定义运行脚本和协作约定，不包含 NEVC-SimOne 软件本体或官方 SDK。

## 目录与入口

文件保留原来的安装目录层级，位于 `SimOneAPI/lib/Win64/`：

- `AutoDrive.py`：自动驾驶测试场景控制程序。
- `ParkingAPA.py`：自动泊车场景控制程序，同时包含部分公共辅助函数。
- `TestDrive.py`：当前 ADAS/LKA 控制程序，以实际源码为准。
- `StartNEVCALL.bat` / `KillNEVCALL.bat`：AutoDrive 启动与停止脚本。
- 其他 `.bat`、`run_algorithm.py`：已有的各入口启动与停止脚本。
- `SimOneEvaluationAPI.py`、`sitecustomize.py`、`environment_check.py`：自定义评价适配和环境辅助文件。
- `README_PYTHON_SETUP.md`：已有的 Python/PyCharm 配置说明。

修改范围与分工见 `AGENTS.md`。

## 环境与运行注意事项

1. 需通过官方渠道另行安装匹配版本的 NEVC-SimOne 和 SimOne SDK；本仓库不能单独替代软件安装。
2. 当前机器使用 Windows、Python 3.11（`C:\Python311\python.exe`），代码位于 `D:\Sim-One`。部分已有脚本使用绝对路径，其他机器使用前须核对路径及匹配的本地 API 适配。
3. SDK 提供的 Python API、DLL、地图、场景、平台配置和授权材料不在仓库内，也不会因 Git 初始化而被修改。
4. 同一主车只能由一个程序下发最终控制指令。不要同时运行多个入口。
5. 成绩以平台实际测试为准；离线检查通过不代表正式成绩。

## Git 收录范围

安装目录内包含大量第三方软件和运行数据，因此 `.gitignore` 使用明确的文件白名单。只收录已确认的团队源码和脚本；SDK/DLL、安装程序、官方资源、日志/CSV、缓存、IDE 配置、进程状态文件和历史 `.bak` 副本均留在本机，不会被删除。

新增团队模块时，先确认文件不包含密码、Token、授权或第三方受限资料，再将具体路径加入 `.gitignore` 白名单。不要使用 `git add -f` 批量上传整个安装目录。

## 日常保存修改

在 `D:\Sim-One` 的终端中执行：

```powershell
git status
git diff
git add -- SimOneAPI/lib/Win64/AutoDrive.py
git commit -m "Describe the algorithm change"
git push
```

将 `git add` 后的路径替换成实际修改且已纳入白名单的文件。首次提交只保存当前文件快照，不会补建之前各次本地备份的历史。

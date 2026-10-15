# NEVC-Simone Python 环境

## 已配置内容

- Python 解释器：`C:\Python311\python.exe`
- API 目录：`D:\Sim-One\SimOneAPI\lib\Win64`
- PyCharm 项目目录：`D:/Sim-One/SimOneAPI/lib`
- `sitecustomize.py`：自动加入 API 目录和 DLL 搜索路径
- `environment_check.py`：检查 Python API 导入和 DLL 加载
- `TestDrive.py`：最小车辆控制示例

## 在 PyCharm 中首次打开

1. 打开 `D:/Sim-One/SimOneAPI/lib` 作为项目。
2. 进入 Settings → Project → Python Interpreter。
3. 选择 Existing interpreter：`C:\Python311\python.exe`。
4. 运行 `environment_check.py`，看到 `SimOne Python API imports: OK` 和 `DLL search path: OK` 即表示开发环境就绪。

## 运行仿真

1. 先启动并登录 NEVC-Simone 本地端。
2. 在平台“资源库 → 控制器”中配置：
   - 启动脚本：`D:\Sim-One\SimOneAPI\lib\Win64\StartPythonAlgorithm.bat`
   - 结束脚本：`D:\Sim-One\SimOneAPI\lib\Win64\KillPythonAlgorithm.bat`
3. 初次联调使用 `TestDrive.py`，确认车辆能接收 API 控制。
4. 再复制 `TestDrive.py` 编写自己的感知、决策和控制逻辑。

## 注意

- 首次配置时先运行 `environment_check.py`；正常联调时无需每次重复运行。
- 已通过 `SimOneEvaluationAPI.py` 封装当前 DLL 中的评价接口；正式参赛前仍应使用一个训练场景确认云端能正常显示得分。
- 正式比赛只能使用组委会提供的主车和赛题场景。

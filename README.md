# MIDBO Python 运行说明

本仓库包含 MATLAB 原版算法以及 Python 版的 `midbo_path_planner.py`。下面示例展示如何直接在本地运行 Python 版本进行航迹规划。

## 环境准备

```bash
python -m venv .venv
source .venv/bin/activate  # Windows 使用 .venv\Scripts\activate
pip install numpy  # 如需更平滑的样条插值，可额外安装 scipy
```

## 直接运行示例

`midbo_path_planner.py` 内置了一个简单示例，使用少量矩形障碍物进行三维航迹规划，可通过以下命令运行：

```bash
python midbo_path_planner.py
```

运行结束后会输出最佳代价、最佳中间航点向量以及收敛曲线长度。

## 在自定义环境中调用

如果想在脚本或 Notebook 中复用，可按照以下方式创建环境并调用求解器：

```python
import numpy as np
from midbo_path_planner import TrajectoryEnvironment, plan_path_with_midbo

# 自定义环境参数
env = TrajectoryEnvironment(
    start_pos=(0.0, 0.0, 10.0),
    goal_pos=(80.0, 80.0, 15.0),
    map_range=(100.0, 100.0, 50.0),
    obstacles=np.array([
        [20.0, 20.0, 0.0, 10.0, 10.0, 20.0],
        [50.0, 40.0, 0.0, 12.0, 20.0, 30.0],
    ]),
    waypoint_count=4,
)

best_cost, waypoint_vec, convergence = plan_path_with_midbo(
    env, population=30, iterations=200, random_state=42
)

print("best cost:", best_cost)
print("waypoint vector shape:", waypoint_vec.shape)
```

`TrajectoryEnvironment.bounds` 提供的上下界可与第三方优化器复用；`env.cost` 则是与 MATLAB 版本一致的代价函数。

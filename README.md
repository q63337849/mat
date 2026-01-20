# MIDBO Python 运行说明

本仓库包含 MATLAB 原版算法以及 Python 版的 `midbo_path_planner.py`。下面示例展示如何直接在本地运行 Python 版本进行航迹规划。

## 环境准备

```bash
python -m venv .venv
source .venv/bin/activate  # Windows 使用 .venv\Scripts\activate
pip install numpy matplotlib  # 如需更平滑且不易过冲的样条插值，可额外安装 scipy
```

## 直接运行示例

`midbo_path_planner.py` 内置了一个简单示例，使用少量矩形障碍物进行三维航迹规划，可通过以下命令运行：

```bash
python midbo_path_planner.py
```

运行结束后会输出最佳代价、最佳中间航点向量以及收敛曲线长度，并在当前目录保存一张示例航迹和收敛曲线的图片 `midbo_demo_path.png`（需要已安装 matplotlib）。

为了减少运行耗时，示例与默认调用使用较小的 `population=20`、`iterations=120`，并在示例环境中将 `sample_count=60`。如需更好的收敛质量，可再逐步提高这些参数。

如果已安装 matplotlib，也可以在自定义脚本里直接调用绘图辅助函数：

```python
from midbo_path_planner import plot_trajectory

image_path = plot_trajectory(env, waypoint_vec, convergence, save_path="my_path.png")
print("plot saved to", image_path)
```

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
    # 避免三次样条产生的过冲，可保持默认的 "pchip" 插值；若想要更加平滑的曲线，可将此参数改为 "cubic"。
    interpolation="pchip",
)

best_cost, waypoint_vec, convergence = plan_path_with_midbo(
    env, population=20, iterations=120, random_state=42
)

print("best cost:", best_cost)
print("waypoint vector shape:", waypoint_vec.shape)
```

`TrajectoryEnvironment.bounds` 提供的上下界可与第三方优化器复用；`env.cost` 则是与 MATLAB 版本一致的代价函数。

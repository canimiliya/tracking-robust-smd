"""Generate representative S1-R1 figures from unmodified raw trajectories."""

from pathlib import Path
import pickle

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RUNS = {
    "instances_simple": "2026-08-08-20-26-01",
    "instances_connected_room": "2026-08-08-21-12-20",
}


def main():
    figure_dir = ROOT / "experiments" / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    for map_name, timestamp in RUNS.items():
        num_agents = 9
        result_dir = (
            ROOT
            / "scripts"
            / "inference"
            / "results_s1_r1_core"
            / timestamp
            / "instance_name___EnvEmptyNoWait2DRobotCompositeNinePlanarDiskRandom"
            / "num_agents___9"
            / "planner___SMDComposite"
            / "single_agent_planner___SMDEnsemble"
            / "0"
        )
        paths = np.load(result_dir / "paths.npy")
        path_data = paths[0, :, : 2 * num_agents].reshape(
            paths.shape[1], num_agents, 2
        ).swapaxes(0, 1)
        map_records = pickle.loads(
            (ROOT / "instances_data" / f"{map_name}.pkl").read_bytes()
        )
        obstacles, robot_data = next(
            record for record in map_records[0] if len(record[1]) == num_agents
        )

        fig, ax = plt.subplots(figsize=(7.2, 6.2))
        for center, radius in obstacles:
            ax.add_patch(
                Circle(
                    center,
                    radius,
                    facecolor="#9aa0a6",
                    edgecolor="#5f6368",
                    alpha=0.75,
                    linewidth=0.7,
                )
            )
        colors = plt.get_cmap("tab10").colors
        for agent_idx, path in enumerate(path_data):
            color = colors[agent_idx % len(colors)]
            ax.plot(
                path[:, 0],
                path[:, 1],
                color=color,
                linewidth=1.6,
                label=f"agent {agent_idx + 1}",
            )
            start = np.asarray(robot_data[agent_idx][0])
            goal = np.asarray(robot_data[agent_idx][1])
            ax.scatter(
                start[0],
                start[1],
                color=color,
                marker="o",
                s=44,
                edgecolors="black",
                linewidths=0.5,
                zorder=5,
            )
            ax.scatter(
                goal[0],
                goal[1],
                color=color,
                marker="x",
                s=55,
                linewidths=1.4,
                zorder=5,
            )
        ax.set_title(f"Official SMD | 9 agents | {map_name} | instance_idx=0")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.2)
        ax.legend(
            ncol=3,
            fontsize=7,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.08),
            frameon=False,
        )
        fig.tight_layout()
        output = figure_dir / f"s1_r1_{map_name}_9agent_idx0.png"
        fig.savefig(output, dpi=180, bbox_inches="tight")
        plt.close(fig)
        print(f"FIGURE={output} BYTES={output.stat().st_size}")


if __name__ == "__main__":
    main()

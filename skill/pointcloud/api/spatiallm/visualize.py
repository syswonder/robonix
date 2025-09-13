import argparse
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from spatiallm import Layout
from spatiallm.pcd import load_o3d_pcd, get_points_and_colors

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Layout Visualization with rerun")
    parser.add_argument(
        "-p",
        "--point_cloud",
        type=str,
        required=True,
        help="Path to the input point cloud file",
    )
    parser.add_argument(
        "-l",
        "--layout",
        type=str,
        required=True,
        help="Path to the layout txt file",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.01,
        help="The radius of the points for visualization",
    )
    parser.add_argument(
        "--max_points",
        type=int,
        default=1000000,
        help="The maximum number of points for visualization",
    )
    rr.script_add_args(parser)
    args = parser.parse_args()

    with open(args.layout, "r") as f:
        layout_content = f.read()

    pcd = load_o3d_pcd(args.point_cloud)
    points, colors = get_points_and_colors(pcd)

    # parse layout_content
    layout = Layout(layout_content)
    floor_plan = layout.to_boxes()

    # ReRun visualization
    blueprint = rrb.Blueprint(
        rrb.Spatial3DView(name="3D", origin="/world", background=[255, 255, 255]),
        collapse_panels=True,
    )
    rr.script_setup(args, "rerun_spatiallm", default_blueprint=blueprint)

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    point_indices = np.arange(points.shape[0])
    np.random.shuffle(point_indices)
    point_indices = point_indices[: args.max_points]
    points = points[point_indices]
    colors = colors[point_indices]
    rr.log(
        "world/points",
        rr.Points3D(
            positions=points,
            colors=colors,
            radii=args.radius,
        ),
        static=True,
    )

    num_entities = len(floor_plan)
    seconds = 0.5
    for ti in range(num_entities + 1):
        sub_floor_plan = floor_plan[:ti]

        rr.set_time_seconds("time_sec", ti * seconds)
        for box in sub_floor_plan:
            uid = box["id"]
            group = box["class"]
            label = box["label"]

            rr.log(
                f"world/pred/{group}/{uid}",
                rr.Boxes3D(
                    centers=box["center"],
                    half_sizes=0.5 * box["scale"],
                    labels=label,
                ),
                rr.InstancePoses3D(mat3x3=box["rotation"]),
                static=False,
            )
    rr.script_teardown(args)

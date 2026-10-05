"""Shared CLI, reporting and animation for simulation-only Community demos."""
import argparse
import json
from pathlib import Path
import random

import numpy as np


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--headless', action='store_true', help='Compute and validate without a window')
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--output', type=Path, help='Write the validation result as JSON')
    return p


def seed_rng(seed):
    random.seed(seed)
    np.random.seed(seed)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def report(args, demo, **metrics):
    result = dict(demo=demo, status='passed', mode='simulation', seed=args.seed, **metrics)
    payload = json.dumps(result, indent=2, allow_nan=False)
    print(payload)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + '\n', encoding='utf-8')
    return result


def dense_path(path, step=.025):
    """Independently resample every joint-space edge, including both endpoints."""
    require(path is not None and len(path) > 0, 'Planner returned no path')
    points = [np.asarray(path[0], dtype=float)]
    for a, b in zip(path, path[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(np.asarray(b) - a)) / step)))
        points.extend(np.linspace(a, b, n + 1)[1:])
    return points


def validate_path(robot, component, path, start, goal, obstacles=()):
    require(path is not None, 'RRT-Connect could not find a path')
    require(np.allclose(path[0], start) and np.allclose(path[-1], goal), 'Path endpoints differ from request')
    samples = dense_path(path)
    limits = np.asarray(robot.get_jnt_ranges(component))
    for i, q in enumerate(samples):
        require(np.isfinite(q).all(), f'Nonfinite joint values at sample {i}')
        require(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]), f'Joint limit violation at sample {i}')
        robot.fk(component, q)
        require(not robot.is_collided(obstacle_list=list(obstacles)), f'Collision at path sample {i}')
    return samples


def plan_path(robot, component, start, goal, obstacles=()):
    from motion.probabilistic.rrt_connect import RRTConnect
    path = RRTConnect(robot).plan(component, start, goal, obstacle_list=list(obstacles),
                                 ext_dist=.08, max_iter=2000, max_time=15,
                                 smoothing_iterations=0)
    return validate_path(robot, component, path, start, goal, obstacles)


def show_robot(robot, states, apply_state, obstacles=(), lookat=(0, 0, .3), objects_at=None):
    """Animate already validated states. No hardware driver is imported."""
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    world = wd.World(cam_pos=[lookat[0] + 2.5, -2.5, lookat[2] + 1.8], lookat_pos=list(lookat))
    gm.gen_frame().attach_to(world)
    for obstacle in obstacles:
        obstacle.attach_to(world)
    current = []

    def update(task):
        index = int(task.time * 20) % len(states)
        for model in current:
            model.detach()
        current.clear()
        apply_state(states[index])
        mesh = robot.gen_meshmodel()
        mesh.attach_to(world)
        current.append(mesh)
        if objects_at:
            for obj in objects_at(index):
                obj.attach_to(world)
                current.append(obj)
        return task.cont

    world.taskMgr.add(update, 'community-playback')
    world.run()

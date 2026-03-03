from __future__ import annotations


def gazebo_world_name(world: str) -> str:
    """
    Normalize project world aliases to Gazebo world names.
    """
    world_s = str(world).strip()
    if world_s == "construction":
        return "office_construction"
    return world_s


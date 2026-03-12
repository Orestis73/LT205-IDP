from state_machine_3 import navigation, heading_name


def describe_step(step, next_node=None):
    node = step["node"]
    move = step["move"]
    hin = heading_name(step["heading_in"])
    hout = heading_name(step["heading_out"])

    if next_node is None:
        base = f"At node {node}, arrive facing {hin}, take action '{move}', leave facing {hout}."
    else:
        base = (
            f"At node {node}, arrive facing {hin}, take action '{move}', "
            f"leave facing {hout}, and go to {next_node}."
        )

    if "spin_dir" in step:
        spin = "rightward" if step["spin_dir"] > 0 else "leftward"
        base += f" 180 spin direction: {spin}."

    if "scan" in step:
        scan = step["scan"]
        turn_word = "right" if scan["turn"] > 0 else "left"
        base += (
            f" Scan point: stack={scan['stack']}, slot={scan['slot']}, "
            f"sensor side={scan['side']}, branch turn={turn_word}."
        )

    return base


def print_scan_summary(nav):
    mission = nav.build_scan_loop_mission()
    scans = [step for step in mission["steps"] if "scan" in step]

    print("=" * 90)
    print("SCAN LOOP SUMMARY")
    print("=" * 90)
    print(f"Total real scan points: {len(scans)}")
    print("Expected: 24 total = 6 per real stack (pd, pu, ou, od)")
    print()

    counts = {"pd": 0, "pu": 0, "ou": 0, "od": 0}
    for step in scans:
        counts[step["scan"]["stack"]] += 1

    print("Counts by stack:")
    for k in ("pd", "pu", "ou", "od"):
        print(f"  {k}: {counts[k]}")
    print()

    print("Real scan points in traversal order:")
    for i, step in enumerate(scans):
        print(f"  {i:02d}. {describe_step(step)}")
    print()

    print("Geometry spot-checks:")
    pu3 = next(s for s in scans if s["scan"]["stack"] == "pu" and s["scan"]["slot"] == 3)
    ou3 = next(s for s in scans if s["scan"]["stack"] == "ou" and s["scan"]["slot"] == 3)
    print(" ", describe_step(pu3))
    print(" ", describe_step(ou3))
    print()

    assert len(scans) == 24
    assert counts == {"pd": 6, "pu": 6, "ou": 6, "od": 6}
    assert pu3["scan"]["turn"] == +1 and pu3["scan"]["side"] == "right"
    assert ou3["scan"]["turn"] == -1 and ou3["scan"]["side"] == "left"

    print("Scan geometry checks passed.")
    print()


def find_scan_step(mission, stack_name, slot_index):
    for i, step in enumerate(mission["steps"]):
        scan = step.get("scan")
        if scan and scan["stack"] == stack_name and scan["slot"] == slot_index:
            return i, step
    return None, None


def print_delivery_mission(mission):
    print(f"Delivery mission has {len(mission['steps'])} planner steps.")
    print(f"It ends at node {mission['end_node']} facing {heading_name(mission['end_heading'])}.")
    print()

    steps = mission["steps"]
    for i, step in enumerate(steps):
        next_node = steps[i + 1]["node"] if i + 1 < len(steps) else mission["end_node"]
        print(f"  {i:02d}. {describe_step(step, next_node=next_node)}")
    print()


def print_return_mission(mission):
    print(f"Return-home mission has {len(mission['steps'])} planner steps.")
    print(f"It ends at node {mission['end_node']} facing {heading_name(mission['end_heading'])}.")
    print()

    steps = mission["steps"]
    for i, step in enumerate(steps):
        next_node = steps[i + 1]["node"] if i + 1 < len(steps) else mission["end_node"]
        print(f"  {i:02d}. {describe_step(step, next_node=next_node)}")
    print()


def simulate_cycle(nav, stack_name, slot_index, colour):
    print("#" * 90)
    print(f"SIMULATED CASE: stack={stack_name}, slot={slot_index}, identified colour={colour}")
    print("#" * 90)
    print()

    scan_mission = nav.build_scan_loop_mission()
    idx, scan_step = find_scan_step(scan_mission, stack_name, slot_index)
    if scan_step is None:
        raise RuntimeError(f"Could not find scan step for {stack_name} slot {slot_index}")

    print("1) SCAN DETECTION")
    print(f"   During the perimeter scan, this reel would be detected at scan step index {idx}.")
    print(f"   {describe_step(scan_step)}")
    print()

    print("2) POST-GRAB POSE")
    nav.register_reel_found(
        stack_name,
        slot_index,
        scan_step["scan"]["turn"],
        scan_step["node"],
        scan_step["heading_out"],
    )
    print(
        f"   After the local grab macro finishes, the planner assumes the robot is back at "
        f"node {nav.rejoin_node}, facing {heading_name(nav.rejoin_heading)}."
    )
    print(
        "   This is the key post-grab pose used to start the delivery mission."
    )
    print()

    print("3) DELIVERY TO BAY-APPROACH POINT")
    delivery_mission = nav.build_delivery_mission(colour)
    target_approach = nav.delivery_targets[colour]["approach"]
    print(
        f"   Because the reel colour is {colour}, the global planner must route to "
        f"approach node {target_approach}."
    )
    print_delivery_mission(delivery_mission)

    print("4) BAY HANDOVER")
    print(
        f"   Global navigation stops at {nav.place_approach_node}."
    )
    print(
        "   Only at that point does the local place/drop-off macro take over."
    )
    print()

    print("5) POST-PLACE POSE")
    nav.set_post_place_pose_from_current_colour()
    print(
        f"   After local place logic finishes, the planner assumes the robot is back at "
        f"{nav.post_place_node}, facing {heading_name(nav.post_place_heading)}."
    )
    print()

    print("6) RETURN-HOME MISSION")
    return_mission = nav.build_return_home_mission()
    print(
        "   Starting from the post-place pose above, the planner computes the route back home."
    )
    print_return_mission(return_mission)

    print("7) CYCLE CLEANUP")
    nav.complete_delivery_cycle()
    print(
        f"   Stack {stack_name} is now marked resolved = {nav.stack_info[stack_name]['resolved']}."
    )
    print(
        f"   Planner mode reset to '{nav.mode}', active_stack reset to {nav.active_stack}."
    )
    print()
    print()


def main():
    nav = navigation()

    print_scan_summary(nav)

    simulate_cycle(nav, "pd", 3, "yellow")
    simulate_cycle(nav, "pu", 3, "green")
    simulate_cycle(nav, "ou", 3, "red")
    simulate_cycle(nav, "od", 3, "blue")

    print("=" * 90)
    print("FINAL STACK STATUS")
    print("=" * 90)
    for stack_name in nav.scan_order:
        info = nav.stack_info[stack_name]
        print(
            f"{stack_name}: found_slot={info['found_slot']}, colour={info['colour']}, "
            f"resolved={info['resolved']}"
        )
    print()
    print(f"All resolved = {nav.all_resolved()}")


if __name__ == "__main__":
    main()
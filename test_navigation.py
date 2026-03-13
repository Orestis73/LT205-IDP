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


def print_steps(title, mission, stop_at_index=None):
    print(title)
    print(f"Number of planner steps: {len(mission['steps'])}")
    print(f"End node: {mission['end_node']}")
    print(f"End heading: {heading_name(mission['end_heading'])}")
    print()

    steps = mission["steps"]
    last_i = len(steps) - 1 if stop_at_index is None else stop_at_index

    for i, step in enumerate(steps[: last_i + 1]):
        is_last_printed = (i == last_i)

        if is_last_printed and stop_at_index is not None:
            text = describe_step(step, next_node=None)
            text += " Detection occurs here at this node."
        else:
            next_node = steps[i + 1]["node"] if i + 1 < len(steps) else mission["end_node"]
            text = describe_step(step, next_node=next_node)

        print(f"  {i:02d}. {text}")
    print()


def print_scan_summary(nav):
    mission = nav.scan_template
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

    pu3 = next(s for s in scans if s["scan"]["stack"] == "pu" and s["scan"]["slot"] == 3)
    ou3 = next(s for s in scans if s["scan"]["stack"] == "ou" and s["scan"]["slot"] == 3)

    assert len(scans) == 24
    assert counts == {"pd": 6, "pu": 6, "ou": 6, "od": 6}
    assert pu3["scan"]["turn"] == +1 and pu3["scan"]["side"] == "right"
    assert ou3["scan"]["turn"] == -1 and ou3["scan"]["side"] == "left"

    print("Scan geometry checks passed.")
    print()


def find_first_reel_detection_in_mission(nav, mission, reel_map):
    """
    Walk the mission until the first scan step that corresponds to a real reel.

    Returns:
        {
            "detection_index": int,
            "detection_step": step,
            "colour": str,
            "empties_before_detection": [(stack, slot), ...]
        }
    """
    empties = []

    for idx, step in enumerate(mission["steps"]):
        scan = step.get("scan")
        if scan is None:
            continue

        key = (scan["stack"], scan["slot"])

        if nav.slot_checked(scan["stack"], scan["slot"]):
            continue

        if key in reel_map:
            return {
                "detection_index": idx,
                "detection_step": step,
                "colour": reel_map[key],
                "empties_before_detection": empties,
            }

        empties.append(key)

    return None


def print_slot_list(label, slot_list):
    print(label)
    if not slot_list:
        print("  None")
    else:
        for stack_name, slot_index in slot_list:
            print(f"  {stack_name} slot {slot_index}")
    print()


def print_stack_status(nav):
    for stack_name in nav.scan_order:
        info = nav.stack_info[stack_name]
        print(
            f"{stack_name}: "
            f"checked={sorted(info['checked_slots'])}, "
            f"detected={sorted(info['detected_slots'])}, "
            f"delivered={sorted(info['delivered_slots'])}, "
            f"colours_by_slot={info['colours_by_slot']}"
        )


def main():
    nav = navigation(expected_total_reels=4)

    # Simulated real reels in the arena
    reel_map = {
        ("pd", 2): "yellow",
        ("pd", 5): "green",
        ("ou", 2): "red",
        ("ou", 5): "blue",
    }

    print_scan_summary(nav)

    print("=" * 90)
    print("FULL CAMPAIGN SIMULATION")
    print("=" * 90)
    print("This simulation now uses the corrected mission policy:")
    print("  - do NOT return to node 1 after every delivery")
    print("  - do NOT restart scanning from node 1 after every delivery")
    print("  - resume scanning from the true post-place pose")
    print("  - return to node 1 only once, after all 4 reels are delivered")
    print()

    campaign_phase = 1
    current_scan_mission = nav.build_initial_scan_mission()

    while True:
        print("#" * 90)
        print(f"SCAN / DETECTION PHASE {campaign_phase}")
        print("#" * 90)
        print()

        detection = find_first_reel_detection_in_mission(nav, current_scan_mission, reel_map)
        if detection is None:
            raise RuntimeError(
                "No future reel was found in the current scan mission. "
                "That means the simulated reel_map and scan policy are inconsistent."
            )

        detect_idx = detection["detection_index"]
        detect_step = detection["detection_step"]
        colour = detection["colour"]
        empties = detection["empties_before_detection"]

        print("1) SCAN ROUTE UNTIL THE NEXT REAL DETECTION")
        print_steps(
            "Scan mission prefix up to the next detection:",
            current_scan_mission,
            stop_at_index=detect_idx,
        )

        print_slot_list("2) Empty scan slots passed before this detection:", empties)

        # Mark empty scanned slots as checked
        for stack_name, slot_index in empties:
            nav.mark_slot_checked(stack_name, slot_index)

        scan = detect_step["scan"]
        print("3) DETECTED REEL")
        print(
            f"Detected reel at stack={scan['stack']}, slot={scan['slot']}, "
            f"colour={colour}."
        )
        print(describe_step(detect_step))
        print()

        print("4) POST-GRAB POSE")
        nav.register_reel_found(
            scan["stack"],
            scan["slot"],
            scan["turn"],
            detect_step["node"],
            detect_step["heading_out"],
        )
        print(
            f"After local grab, planner assumes robot is back at node {nav.rejoin_node}, "
            f"facing {heading_name(nav.rejoin_heading)}."
        )
        print()

        print("5) DELIVERY TO BAY-APPROACH POINT")
        delivery_mission = nav.build_delivery_mission(colour)
        print(
            f"Because the reel is {colour}, global navigation routes to "
            f"{nav.delivery_targets[colour]['approach']}."
        )
        print_steps("Delivery mission:", delivery_mission)
        print("6) BAY HANDOVER")
        print(f"Global navigation hands over at {nav.place_approach_node}.")
        print("Only there does the local place/drop-off macro begin.")
        print()

        print("7) POST-PLACE POSE")
        nav.set_post_place_pose_from_current_colour()
        print(
            f"After local place logic, planner assumes robot is back at "
            f"{nav.post_place_node}, facing {heading_name(nav.post_place_heading)}."
        )
        print()

        projected_total = nav.total_delivered() + 1

        if projected_total >= nav.expected_total_reels:
            print("8) FINAL DELIVERY REACHED")
            print(
                f"This is delivery {projected_total} / {nav.expected_total_reels}, "
                "so planner should now return home exactly once."
            )
            print()

            final_return_home = nav.build_return_home_mission()
            nav.complete_delivery_cycle()

            print("9) FINAL RETURN-HOME MISSION")
            print_steps("Return-home mission:", final_return_home)

            print("10) FINAL PLANNER STATE")
            print_stack_status(nav)
            print()
            print(f"Total delivered = {nav.total_delivered()} / {nav.expected_total_reels} required reels")
            print(f"Reels remaining to find = {nav.reels_remaining_to_find()}")
            print(f"All resolved = {nav.all_resolved()}")
            break

        print("8) RESUME SCAN INSTEAD OF RETURNING HOME")
        print(
            f"This is only delivery {projected_total} / {nav.expected_total_reels}, "
            "so planner must resume scanning, not go home."
        )
        resume_scan_mission = nav.build_resume_scan_mission()
        nav.complete_delivery_cycle()

        print_steps("Resume-scan mission:", resume_scan_mission)

        print("9) PLANNER STATE AFTER THIS DELIVERY")
        print_stack_status(nav)
        print()
        print(f"Total delivered so far = {nav.total_delivered()} / {nav.expected_total_reels}")
        print(f"Reels remaining to find = {nav.reels_remaining_to_find()}")
        print()

        current_scan_mission = resume_scan_mission
        campaign_phase += 1

main()
"""
pairwise-factory-system - FactorySystem

Factory automation core: conveyors, robotic arms, assemblers,
logistics routing and circuit networks.
"""

from collections import deque

_DIRECTIONS = {
    "N": (0, -1), "S": (0, 1), "E": (1, 0), "W": (-1, 0),
    "north": (0, -1), "south": (0, 1), "east": (1, 0), "west": (-1, 0),
}


class FactorySystem:
    def __init__(self):
        self.state = {
            "conveyors": {},
            "arms": {},
            "assemblers": {},
            "network": {"nodes": {}, "links": []},
            "circuits": {},
            "time": 0,
        }

    # ------------------------------------------------------------------
    # Conveyors: item movement + direction + per-belt speed
    # ------------------------------------------------------------------
    def update_conveyor(self, belts=None, dt=1.0):
        """Advance items along conveyor belts.

        Each belt: {"direction": "E"|(dx, dy), "speed": float,
                    "length": float, "items": [{"item", "position"}],
                    "next": belt_id or None, "capacity": int}
        Items move by speed*dt along the belt direction; items reaching
        the end transfer to the linked belt if it has room, otherwise
        they wait at the end (blocked).
        """
        if belts is None:
            belts = self.state["conveyors"]
        moved = {}
        for belt_id, belt in belts.items():
            direction = belt.get("direction", "E")
            if isinstance(direction, str):
                direction = _DIRECTIONS.get(direction, (1, 0))
            speed = float(belt.get("speed", 1.0))
            length = float(belt.get("length", 1.0))
            items = belt.setdefault("items", [])
            belt["direction"] = direction

            for entry in items:
                step = speed * dt
                entry["position"] = min(entry.get("position", 0.0) + step, length)
            moved[belt_id] = [
                {"item": e.get("item"), "position": e.get("position", 0.0),
                 "direction": direction, "speed": speed}
                for e in items
            ]
        for belt_id, belt in belts.items():
            items = belt.get("items", [])
            length = float(belt.get("length", 1.0))
            for entry in list(items):
                if entry.get("position", 0.0) < length:
                    continue
                nxt = belt.get("next")
                target = belts.get(nxt) if nxt is not None else None
                if target is None:
                    continue
                capacity = target.get("capacity")
                if capacity is None or len(target.setdefault("items", [])) < capacity:
                    items.remove(entry)
                    entry["position"] = 0.0
                    target["items"].append(entry)
        return moved

    # ------------------------------------------------------------------
    # Robotic arms: grab detection + path planning + priority
    # ------------------------------------------------------------------
    def update_arm(self, arm=None, items=None, obstacles=None):
        """Update a robotic arm for one tick.

        arm: {"position": (x, y), "range": float, "state": str,
              "priority": [item_type, ...], "holding": item or None}
        Items within range are detected; the target is chosen by the
        priority list first, then by nearest distance. A waypoint path
        is planned toward the target, routing around obstacles.
        """
        if arm is None:
            arm = self.state["arms"].setdefault("arm0", {
                "position": (0, 0), "range": 5.0, "state": "idle",
                "priority": [], "holding": None,
            })
        if items is None:
            items = arm.get("items", [])
        obstacles = obstacles or []

        ax, ay = arm.get("position", (0, 0))
        reach = float(arm.get("range", 5.0))
        priority = arm.get("priority") or []

        detected = []
        for entry in items:
            ix, iy = entry.get("position", (0, 0))
            dist = ((ix - ax) ** 2 + (iy - ay) ** 2) ** 0.5
            if dist <= reach:
                detected.append((dist, entry))

        def rank(entry):
            item_type = entry.get("item") or entry.get("type")
            return priority.index(item_type) if item_type in priority else len(priority)

        target = None
        if detected:
            detected.sort(key=lambda pair: (rank(pair[1]), pair[0]))
            target = detected[0][1]

        path = []
        grabbed = None
        if target is not None and arm.get("holding") is None:
            goal = tuple(target.get("position", (0, 0)))
            path = self._plan_path((ax, ay), goal, obstacles)
            arm["state"] = "grabbing"
            arm["holding"] = target.get("item", target)
            grabbed = target
            if target in items:
                items.remove(target)
        elif arm.get("holding") is not None:
            arm["state"] = "carrying"
        else:
            arm["state"] = "idle"

        return {"state": arm["state"], "detected": [e for _, e in detected],
                "target": target, "path": path, "holding": arm.get("holding"),
                "grabbed": grabbed}

    @staticmethod
    def _plan_path(start, goal, obstacles, grid=1):
        """A* path on a grid avoiding obstacle cells; straight line if clear."""
        blocked = {tuple(o) for o in obstacles}
        if start == goal:
            return [start]
        if goal not in blocked:
            sx, sy = start
            gx, gy = goal
            steps = int(max(abs(gx - sx), abs(gy - sy)))
            clear = steps > 0
            for i in range(1, steps + 1):
                x = round(sx + (gx - sx) * i / steps)
                y = round(sy + (gy - sy) * i / steps)
                if (x, y) in blocked:
                    clear = False
                    break
            if clear:
                return [start, goal]

        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}
        while frontier:
            frontier.sort(key=lambda pair: pair[0])
            _, current = frontier.pop(0)
            if current == goal:
                break
            cx, cy = current
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (cx + dx, cy + dy)
                if nxt in blocked:
                    continue
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    heuristic = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                    frontier.append((new_cost + heuristic, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return [start]
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Assemblers: recipe check + production time + output buffer
    # ------------------------------------------------------------------
    def update_assembler(self, assembler=None, dt=1.0):
        """Advance an assembler for dt time units.

        assembler: {"recipe": {"inputs": {item: n}, "output": item,
                                "time": float},
                    "inputs": {item: n}, "progress": float,
                    "output_buffer": [item, ...], "buffer_size": int}
        Production only starts when inputs exactly satisfy the recipe,
        takes recipe["time"] to finish, and the product goes into the
        output buffer; if the buffer is full the assembler stalls.
        """
        if assembler is None:
            assembler = self.state["assemblers"].setdefault("assembler0", {
                "recipe": None, "inputs": {}, "progress": 0.0,
                "output_buffer": [], "buffer_size": 10,
            })
        recipe = assembler.get("recipe")
        inputs = assembler.setdefault("inputs", {})
        buffer = assembler.setdefault("output_buffer", [])
        buffer_size = assembler.get("buffer_size", 10)
        status = "idle"

        if recipe is None:
            return {"status": "no_recipe", "progress": 0.0,
                    "output_buffer": buffer}

        required = recipe.get("inputs", {})
        has_materials = all(inputs.get(item, 0) >= count
                            for item, count in required.items())
        progress = float(assembler.get("progress", 0.0))
        producing = progress > 0.0

        if not producing and has_materials and len(buffer) < buffer_size:
            for item, count in required.items():
                inputs[item] = inputs.get(item, 0) - count
            producing = True
        elif not producing and not has_materials:
            status = "missing_materials"

        if producing:
            progress += dt
            status = "producing"
            if progress >= float(recipe.get("time", 1.0)):
                if len(buffer) < buffer_size:
                    buffer.append(recipe.get("output"))
                    progress = 0.0
                    status = "completed"
                else:
                    progress = float(recipe.get("time", 1.0))
                    status = "output_blocked"

        assembler["progress"] = progress
        return {"status": status, "progress": progress,
                "output_buffer": buffer, "inputs": dict(inputs)}

    # ------------------------------------------------------------------
    # Logistics: path selection + load balancing + item classification
    # ------------------------------------------------------------------
    def route_items(self, items=None, network=None):
        """Route items through the logistics network.

        network: {"nodes": [...], "links": [{"from", "to", "capacity",
                                             "load", "cost"}]}
        Each item: {"item"/"type", "source", "destination"}.
        Every item gets its own least-cost path; links at capacity are
        avoided so traffic balances across routes. Items are classified
        by type and reported per category.
        """
        if network is None:
            network = self.state["network"]
        if items is None:
            items = network.get("items", [])
        links = network.get("links", [])

        adjacency = {}
        for link in links:
            adjacency.setdefault(link.get("from"), []).append(link)

        routes = {}
        categories = {}
        for entry in items:
            item_type = entry.get("item") or entry.get("type") or "unknown"
            categories.setdefault(item_type, []).append(entry)
            source = entry.get("source")
            destination = entry.get("destination")
            path = self._find_route(adjacency, source, destination)
            routes[id(entry)] = {"item": item_type, "path": path}
            entry["route"] = path
            if path:
                for a, b in zip(path, path[1:]):
                    for link in adjacency.get(a, []):
                        if link.get("to") == b:
                            link["load"] = link.get("load", 0) + 1
                            break

        return {"routes": routes,
                "categories": {k: len(v) for k, v in categories.items()},
                "links": links}

    @staticmethod
    def _find_route(adjacency, source, destination):
        """Dijkstra over effective cost; saturated links are avoided."""
        if source is None or destination is None:
            return []
        if source == destination:
            return [source]
        dist = {source: 0.0}
        prev = {}
        queue = [(0.0, source)]
        while queue:
            queue.sort(key=lambda pair: pair[0])
            cost, node = queue.pop(0)
            if node == destination:
                break
            if cost > dist.get(node, float("inf")):
                continue
            for link in adjacency.get(node, []):
                capacity = link.get("capacity")
                load = link.get("load", 0)
                if capacity is not None and load >= capacity:
                    continue  # congested: route around
                weight = float(link.get("cost", 1.0))
                if capacity:
                    weight *= 1.0 + load / capacity  # balance load
                nxt = link.get("to")
                new_cost = cost + weight
                if new_cost < dist.get(nxt, float("inf")):
                    dist[nxt] = new_cost
                    prev[nxt] = node
                    queue.append((new_cost, nxt))
        if destination not in prev:
            return []
        path = [destination]
        while path[-1] != source:
            path.append(prev[path[-1]])
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Circuits: signal propagation + logic gates + delay
    # ------------------------------------------------------------------
    def update_circuit(self, circuit=None, ticks=1):
        """Propagate signals through a circuit network.

        circuit: {"components": {id: {"type": "switch"|"lamp"|"and"|
                                      "or"|"not"|"wire", "on": bool,
                                      "delay": int}},
                  "wires": [[from_id, to_id], ...]}
        Signals propagate component outputs to inputs each tick; gates
        (and/or/not) evaluate their inputs, and each component's delay
        (in ticks) is honored before its output changes.
        """
        if circuit is None:
            circuit = self.state["circuits"].setdefault("circuit0", {
                "components": {}, "wires": [],
            })
        components = circuit.setdefault("components", {})
        wires = circuit.setdefault("wires", [])
        for comp in components.values():
            comp.setdefault("on", False)

        incoming = {}
        for src, dst in wires:
            incoming.setdefault(dst, []).append(src)

        for _ in range(max(1, ticks)):
            pending = {}
            for comp_id, comp in components.items():
                ctype = comp.get("type", "wire")
                if ctype == "switch":
                    continue  # switches are manual sources
                inputs = [bool(components[src].get("on", False))
                          for src in incoming.get(comp_id, [])
                          if src in components]
                if ctype == "and":
                    value = all(inputs) if inputs else False
                elif ctype == "or":
                    value = any(inputs)
                elif ctype == "not":
                    value = not inputs[0] if inputs else True
                else:  # wire / lamp / passthrough
                    value = any(inputs)

                delay = int(comp.get("delay", 0))
                if value != bool(comp.get("on", False)):
                    remaining = comp.get("_delay_left")
                    if remaining is None or comp.get("_delay_target") != value:
                        comp["_delay_left"] = delay
                        comp["_delay_target"] = value
                        remaining = delay
                    if remaining <= 0:
                        pending[comp_id] = value
                        comp["_delay_left"] = None
                    else:
                        comp["_delay_left"] = remaining - 1
            for comp_id, value in pending.items():
                components[comp_id]["on"] = value

        return {comp_id: bool(comp.get("on", False))
                for comp_id, comp in components.items()}

import math


def normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def arc_command(x: float, y: float, yaw: float, target_x: float, target_y: float,
                speed: float, gain: float, max_angular: float) -> tuple[float, float]:
    """质点圆弧跟踪：由前视点横向误差直接求曲率，不做车体碰撞门控。"""
    dx = target_x - x
    dy = target_y - y
    c, s = math.cos(yaw), math.sin(yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    distance_sq = max(1.0e-6, local_x * local_x + local_y * local_y)
    curvature = 2.0 * local_y / distance_sq
    angular = max(-max_angular, min(max_angular, speed * curvature))
    heading_error = normalize(math.atan2(dy, dx) - yaw)
    angular += gain * heading_error
    angular = max(-max_angular, min(max_angular, angular))
    return speed, angular


def fillet_path(points: list[tuple[float, float]], radius: float,
                step: float = 0.015) -> list[tuple[float, float]]:
    """用相邻直线的公切圆弧替换折点，返回稠密质点轨迹。"""
    if len(points) < 3 or radius <= 0.0:
        return points[:]
    result = [points[0]]
    for index in range(1, len(points) - 1):
        ax, ay = points[index - 1]
        bx, by = points[index]
        cx, cy = points[index + 1]
        in_length = math.hypot(bx - ax, by - ay)
        out_length = math.hypot(cx - bx, cy - by)
        if in_length < 1.0e-6 or out_length < 1.0e-6:
            continue
        ux, uy = (bx - ax) / in_length, (by - ay) / in_length
        vx, vy = (cx - bx) / out_length, (cy - by) / out_length
        turn = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        if abs(turn) < 0.05 or abs(turn) > math.pi - 0.05:
            result.append((bx, by))
            continue
        tangent = min(radius * math.tan(abs(turn) * 0.5), 0.45 * in_length, 0.45 * out_length)
        actual_radius = tangent / math.tan(abs(turn) * 0.5)
        entry = (bx - ux * tangent, by - uy * tangent)
        exit_point = (bx + vx * tangent, by + vy * tangent)
        side = 1.0 if turn > 0.0 else -1.0
        center = (entry[0] - side * uy * actual_radius,
                  entry[1] + side * ux * actual_radius)
        start_angle = math.atan2(entry[1] - center[1], entry[0] - center[0])
        arc_length = abs(turn) * actual_radius
        samples = max(2, int(math.ceil(arc_length / max(0.005, step))))
        result.append(entry)
        for sample in range(1, samples):
            angle = start_angle + turn * sample / samples
            result.append((center[0] + actual_radius * math.cos(angle),
                           center[1] + actual_radius * math.sin(angle)))
        result.append(exit_point)
    result.append(points[-1])
    return result
